/*
** hw_bsp.cpp
**
** Main rendering loop / BSP traversal / visibility clipping
**
**---------------------------------------------------------------------------
**
** Copyright 2000-2016 Christoph Oelckers
** Copyright 2017-2025 GZDoom Maintainers and Contributors
** Copyright 2025-2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#include "p_lnspec.h"
#include "p_local.h"
#include "a_sharedglobal.h"
#include "g_levellocals.h"
#include "p_effect.h"
#include "po_man.h"
#include "m_fixed.h"
#include "ctpl.h"
#include "texturemanager.h"
#include "hwrenderer/scene/hw_fakeflat.h"
#include "hwrenderer/scene/hw_clipper.h"
#include "hwrenderer/scene/hw_drawstructs.h"
#include "hwrenderer/scene/hw_drawinfo.h"
#include "hwrenderer/scene/hw_portal.h"
#include "hw_clock.h"
#include "flatvertices.h"
#include "hw_vertexbuilder.h"
#include "hw_walldispatcher.h"

#include "p_trace.h"

#include "p_visualthinker.h"
#include "p_foliage.h"
#include "hw_hidethings.h"
#include "r_sky.h"
#include "p_3dfloors.h"
#include "hw_horizon.h"
#include <float.h>
#include <utility>

#ifdef ARCH_IA32
#include <immintrin.h>
#endif // ARCH_IA32

CVAR(Bool, gl_multithread, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)

EXTERN_CVAR(Float, r_actorspriteshadowdist)
EXTERN_CVAR(Bool, r_radarclipper)
EXTERN_CVAR(Bool, r_dithertransparency)
EXTERN_CVAR(Color, gl_cullcolor)

// Skip BSP subtrees and subsectors that are entirely outside the 3D view frustum.
CVAR(Bool, gl_frustum_cull_bsp, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Extra height added above and below every subsector's range, for things (models, sprites) that stick out of their sector.
CVAR(Float, gl_frustum_bsp_zpad, 64.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Only test BSP subtrees against the 3D frustum when the view is pitched at least this far up or down (degrees).
CVAR(Float, gl_frustum_bsp_pitch, 30.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Skip walls and flats hidden behind hills, buildings and low ceilings (occlusion horizon, main view only).
CVAR(Bool, gl_horizon_cull, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Only ground and ceilings within this distance of the eye are recorded as occluders (everything is still tested).
CVAR(Float, gl_horizon_range, 6144.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
EXTERN_CVAR(Bool, gl_no_skyclear)

cycle_t horizonTime;

//==========================================================================
//
// Height ranges of the BSP, for testing nodes and subsectors against the
// view frustum. The 2D bounding boxes come from the nodes; the heights are
// recomputed from the sectors' current planes once per frame, so moving
// floors and ceilings are always covered.
//
// A subsector's range covers everything that is drawn for it: its own
// sector, the render sector, the back sectors of its walls (upper, lower
// and middle parts reach their heights), Boom fake planes and 3D floors.
// Skies and plane portals make that side unbounded, because sky walls and
// portals reach far past the plane. Horizon lines make the whole subsector
// unbounded.
//
//==========================================================================

static constexpr float ZB_UNBOUNDED = 1e6f;

// Per-seg flags: can this edge of its subsector hide things behind it (for the horizon)?
enum
{
	SEGOCC_FLOOR = 1,	// a step up, or a ridge of sloped terrain
	SEGOCC_CEILING = 2,	// solid ceiling edge
};

struct FBspZBounds
{
	const node_t *nodesData = nullptr;
	unsigned numNodes = 0, numSubsectors = 0, numSectors = 0, numSegs = 0;
	bool updated = false;		// heights are current (UpdateBspZBounds ran with culling on)
	bool built = false;

	TArray<int> postorder;		// node indices, children before parents
	TArray<int> nodeParent;		// -1 for the root
	TArray<int> subParent;		// node owning each subsector
	TArray<int> subStart;		// subsector i uses sectors subSectors[subStart[i] .. subStart[i+1]-1]
	TArray<int> subSectors;
	TArray<int> secSubStart;	// sector i is used by subsectors secSubs[secSubStart[i] .. secSubStart[i+1]-1]
	TArray<int> secSubs;
	TArray<int> depStart;		// sector i is a heightsec or 3D-floor source for sectors deps[depStart[i] ..]
	TArray<int> deps;
	TArray<uint8_t> subUnbounded;
	TArray<uint8_t> segOccluder;	// SEGOCC_* per seg
	TArray<float> sectorZ;		// [2*i] lowest, [2*i+1] highest, per sector / subsector / node
	TArray<float> subZ;
	TArray<float> nodeZ;
	// The same without the sky allowance: sky planes count at their own height. Sky walls reach far past
	// that, but they only show the sky, which is drawn behind everything anyway. Used by the horizon test,
	// which never hides actors, so these need no padding for them either.
	TArray<float> sectorZGeom;
	TArray<float> subZGeom;
	TArray<float> nodeZGeom;

	// What a sector's range was computed from; compared every frame to find the sectors that changed.
	struct Signature
	{
		double floorD, ceilD, floorNZ, ceilNZ;
		FTextureID floorTex, ceilTex;
		unsigned portals[2];
		sector_t *heightsec;
		unsigned ffloors;
		bool operator!=(const Signature &o) const
		{
			return floorD != o.floorD || ceilD != o.ceilD || floorNZ != o.floorNZ || ceilNZ != o.ceilNZ || floorTex != o.floorTex || ceilTex != o.ceilTex ||
				portals[0] != o.portals[0] || portals[1] != o.portals[1] || heightsec != o.heightsec || ffloors != o.ffloors;
		}
	};
	TArray<Signature> signatures;
	TArray<int> dirtySectors;
	TArray<uint8_t> sectorDirty;

	bool Matches(FLevelLocals *Level) const
	{
		return built && nodesData == Level->nodes.Data() && numNodes == Level->nodes.Size()
			&& numSubsectors == Level->subsectors.Size() && numSectors == Level->sectors.Size() && numSegs == Level->segs.Size();
	}

	void Build(FLevelLocals *Level);
	void Update(FLevelLocals *Level);
	void ComputeSector(FLevelLocals *Level, unsigned i);
	void ComputeSubsector(FLevelLocals *Level, unsigned i);
	void ComputeNode(FLevelLocals *Level, int n);
	static Signature GetSignature(sector_t *sec);
};

void DeleteBspZBounds(FBspZBounds *bounds)
{
	delete bounds;
}

static void AddPlaneRange(const secplane_t &plane, sector_t *where, double &lo, double &hi)
{
	if (!plane.isSlope())
	{
		const double z = plane.ZatPoint(where->centerspot);
		lo = min(lo, z);
		hi = max(hi, z);
		return;
	}
	for (line_t *ln : where->Lines)
	{
		const double z1 = plane.ZatPoint(ln->v1), z2 = plane.ZatPoint(ln->v2);
		lo = min(lo, min(z1, z2));
		hi = max(hi, max(z1, z2));
	}
}

static bool PlaneUnbounded(sector_t *sec, int plane)
{
	return sec->GetTexture(plane) == skyflatnum || sec->Portals[plane] != 0;
}

FBspZBounds::Signature FBspZBounds::GetSignature(sector_t *sec)
{
	Signature sig;
	sig.floorD = sec->floorplane.fD();
	sig.ceilD = sec->ceilingplane.fD();
	sig.floorNZ = sec->floorplane.Normal().Z;
	sig.ceilNZ = sec->ceilingplane.Normal().Z;
	sig.floorTex = sec->GetTexture(sector_t::floor);
	sig.ceilTex = sec->GetTexture(sector_t::ceiling);
	sig.portals[0] = sec->Portals[0];
	sig.portals[1] = sec->Portals[1];
	sig.heightsec = sec->GetHeightSec();
	sig.ffloors = sec->e->XFloor.ffloors.Size();
	return sig;
}

void FBspZBounds::ComputeSector(FLevelLocals *Level, unsigned i)
{
	sector_t *sec = &Level->sectors[i];
	double lo = DBL_MAX, hi = -DBL_MAX;
	AddPlaneRange(sec->floorplane, sec, lo, hi);
	AddPlaneRange(sec->ceilingplane, sec, lo, hi);

	sector_t *hs = sec->GetHeightSec();
	if (hs != nullptr)
	{
		AddPlaneRange(hs->floorplane, sec, lo, hi);
		AddPlaneRange(hs->ceilingplane, sec, lo, hi);
	}
	for (F3DFloor *ff : sec->e->XFloor.ffloors)
	{
		if (ff->top.plane) AddPlaneRange(*ff->top.plane, sec, lo, hi);
		if (ff->bottom.plane) AddPlaneRange(*ff->bottom.plane, sec, lo, hi);
	}

	lo = max(lo, -(double)ZB_UNBOUNDED);
	hi = min(hi, (double)ZB_UNBOUNDED);
	const bool portalFloor = sec->Portals[sector_t::floor] != 0 || (hs && hs->Portals[sector_t::floor] != 0);
	const bool portalCeiling = sec->Portals[sector_t::ceiling] != 0 || (hs && hs->Portals[sector_t::ceiling] != 0);
	sectorZGeom[i * 2] = portalFloor ? -ZB_UNBOUNDED : (float)lo;
	sectorZGeom[i * 2 + 1] = portalCeiling ? ZB_UNBOUNDED : (float)hi;

	if (PlaneUnbounded(sec, sector_t::floor) || (hs && PlaneUnbounded(hs, sector_t::floor))) lo = -ZB_UNBOUNDED;
	if (PlaneUnbounded(sec, sector_t::ceiling) || (hs && PlaneUnbounded(hs, sector_t::ceiling))) hi = ZB_UNBOUNDED;
	sectorZ[i * 2] = (float)lo;
	sectorZ[i * 2 + 1] = (float)hi;
}

void FBspZBounds::ComputeSubsector(FLevelLocals *Level, unsigned i)
{
	float lo = FLT_MAX, hi = -FLT_MAX, glo = FLT_MAX, ghi = -FLT_MAX;
	if (subUnbounded[i])
	{
		lo = glo = -ZB_UNBOUNDED;
		hi = ghi = ZB_UNBOUNDED;
	}
	else for (int j = subStart[i]; j < subStart[i + 1]; j++)
	{
		const int s = subSectors[j];
		lo = min(lo, sectorZ[s * 2]);
		hi = max(hi, sectorZ[s * 2 + 1]);
		glo = min(glo, sectorZGeom[s * 2]);
		ghi = max(ghi, sectorZGeom[s * 2 + 1]);
	}
	// Render hacks draw planes taken from elsewhere: never hide them.
	if (Level->subsectors[i].hacked)
	{
		glo = -ZB_UNBOUNDED;
		ghi = ZB_UNBOUNDED;
	}
	subZ[i * 2] = lo;
	subZ[i * 2 + 1] = hi;
	subZGeom[i * 2] = glo;
	subZGeom[i * 2 + 1] = ghi;
}

void FBspZBounds::ComputeNode(FLevelLocals *Level, int n)
{
	node_t &node = Level->nodes[n];
	float lo = FLT_MAX, hi = -FLT_MAX, glo = FLT_MAX, ghi = -FLT_MAX;
	for (void *child : node.children)
	{
		const bool isSub = (size_t)child & 1;
		const int index = isSub ? ((subsector_t *)((uint8_t *)child - 1))->Index() : ((node_t *)child)->Index();
		const float *z = isSub ? &subZ[index * 2] : &nodeZ[index * 2];
		const float *g = isSub ? &subZGeom[index * 2] : &nodeZGeom[index * 2];
		lo = min(lo, z[0]);
		hi = max(hi, z[1]);
		glo = min(glo, g[0]);
		ghi = max(ghi, g[1]);
	}
	nodeZ[n * 2] = lo;
	nodeZ[n * 2 + 1] = hi;
	nodeZGeom[n * 2] = glo;
	nodeZGeom[n * 2 + 1] = ghi;
}

// Floor height of a sector at a point, for classifying edges.
static double FloorAt(sector_t *sec, const DVector2 &p) { return sec->floorplane.ZatPoint(p); }

void FBspZBounds::Build(FLevelLocals *Level)
{
	nodesData = Level->nodes.Data();
	numNodes = Level->nodes.Size();
	numSubsectors = Level->subsectors.Size();
	numSectors = Level->sectors.Size();
	numSegs = Level->segs.Size();
	built = true;

	// children-first order of all nodes, and every node's and subsector's parent
	postorder.Clear();
	nodeParent.Resize(numNodes);
	subParent.Resize(numSubsectors);
	for (auto &p : nodeParent) p = -1;
	for (auto &p : subParent) p = -1;
	if (numNodes > 0)
	{
		TArray<std::pair<node_t *, int>> stack;
		stack.Push({ (node_t *)Level->HeadNode(), 0 });
		while (stack.Size() > 0)
		{
			auto &top = stack.Last();
			if (top.second < 2)
			{
				node_t *parent = top.first;
				void *child = parent->children[top.second++];
				if (!((size_t)child & 1))
				{
					nodeParent[((node_t *)child)->Index()] = parent->Index();
					stack.Push({ (node_t *)child, 0 });
				}
				else subParent[((subsector_t *)((uint8_t *)child - 1))->Index()] = parent->Index();
			}
			else
			{
				postorder.Push(top.first->Index());
				stack.Pop();
			}
		}
	}

	subStart.Resize(numSubsectors + 1);
	subSectors.Clear();
	subUnbounded.Resize(numSubsectors);
	for (unsigned i = 0; i < numSubsectors; i++)
	{
		subsector_t &sub = Level->subsectors[i];
		subStart[i] = subSectors.Size();
		subUnbounded[i] = 0;
		auto add = [&](sector_t *sec)
		{
			if (sec == nullptr) return;
			const int index = sec->Index();
			for (unsigned j = subStart[i]; j < subSectors.Size(); j++) if (subSectors[j] == index) return;
			subSectors.Push(index);
		};
		add(sub.sector);
		add(sub.render_sector);
		for (uint32_t j = 0; j < sub.numlines; j++)
		{
			seg_t &seg = sub.firstline[j];
			add(seg.backsector);
			if (seg.linedef && seg.linedef->special == Line_Horizon) subUnbounded[i] = 1;
		}
		if (subSectors.Size() == (unsigned)subStart[i]) subUnbounded[i] = 1;	// no sector: don't guess
	}
	subStart[numSubsectors] = subSectors.Size();

	// inverse: sector -> subsectors whose range includes it
	secSubStart.Resize(numSectors + 1);
	for (auto &c : secSubStart) c = 0;
	for (int s : subSectors) secSubStart[s + 1]++;
	for (unsigned i = 0; i < numSectors; i++) secSubStart[i + 1] += secSubStart[i];
	secSubs.Resize(subSectors.Size());
	{
		TArray<int> fill;
		fill.Resize(numSectors);
		for (unsigned i = 0; i < numSectors; i++) fill[i] = secSubStart[i];
		for (unsigned i = 0; i < numSubsectors; i++)
			for (int j = subStart[i]; j < subStart[i + 1]; j++) secSubs[fill[subSectors[j]]++] = i;
	}

	// sectors whose planes feed other sectors' ranges (Boom fake floors, 3D floors)
	{
		TArray<std::pair<int, int>> pairs;	// (source, user)
		for (unsigned i = 0; i < numSectors; i++)
		{
			sector_t *sec = &Level->sectors[i];
			if (sector_t *hs = sec->GetHeightSec()) pairs.Push({ hs->Index(), (int)i });
			for (F3DFloor *ff : sec->e->XFloor.ffloors) if (ff->model) pairs.Push({ ff->model->Index(), (int)i });
		}
		depStart.Resize(numSectors + 1);
		for (auto &c : depStart) c = 0;
		for (auto &pr : pairs) depStart[pr.first + 1]++;
		for (unsigned i = 0; i < numSectors; i++) depStart[i + 1] += depStart[i];
		deps.Resize(pairs.Size());
		TArray<int> fill;
		fill.Resize(numSectors);
		for (unsigned i = 0; i < numSectors; i++) fill[i] = depStart[i];
		for (auto &pr : pairs) deps[fill[pr.first]++] = pr.second;
	}

	// Which edges can hide anything for the horizon. Leaving an edge out only means less gets hidden, never
	// that something visible is hidden, so this only needs to keep the ones that matter:
	// - floor: a step up from the other side (a building, a cliff), or a ridge where sloped ground falls away
	//   on both sides. Along any line of sight the steepest view over continuous ground is at a ridge or a
	//   step; flat ground and valleys never are.
	// - ceiling: every edge under a solid ceiling (rare outdoors, cheap indoors).
	segOccluder.Resize(numSegs);
	for (unsigned i = 0; i < numSegs; i++)
	{
		seg_t &seg = Level->segs[i];
		uint8_t flags = 0;
		sector_t *front = seg.frontsector, *back = seg.backsector;
		if (front && back && seg.v1 && seg.v2 && front != back)
		{
			const DVector2 a = seg.v1->fPos(), b = seg.v2->fPos();
			const double ff1 = FloorAt(front, a), ff2 = FloorAt(front, b), bf1 = FloorAt(back, a), bf2 = FloorAt(back, b);
			if (ff1 > bf1 + 0.5 || ff2 > bf2 + 0.5) flags |= SEGOCC_FLOOR;	// this side is higher
			else if (fabs(ff1 - bf1) <= 0.5 && fabs(ff2 - bf2) <= 0.5)
			{
				// continuous ground: a ridge if each side's plane, extended over the other, lies above it
				const double overBack = FloorAt(front, back->centerspot) - FloorAt(back, back->centerspot);
				const double overFront = FloorAt(back, front->centerspot) - FloorAt(front, front->centerspot);
				if (overBack > 0.5 || overFront > 0.5) flags |= SEGOCC_FLOOR;
			}
			if (front->GetTexture(sector_t::ceiling) != skyflatnum && front->Portals[sector_t::ceiling] == 0) flags |= SEGOCC_CEILING;
		}
		segOccluder[i] = flags;
	}

	sectorZ.Resize(numSectors * 2);
	subZ.Resize(numSubsectors * 2);
	sectorZGeom.Resize(numSectors * 2);
	subZGeom.Resize(numSubsectors * 2);
	nodeZGeom.Resize(numNodes * 2);
	nodeZ.Resize(numNodes * 2);
	// anything the walk can't reach stays unbounded
	for (unsigned i = 0; i < numNodes * 2; i++) nodeZ[i] = nodeZGeom[i] = (i & 1) ? ZB_UNBOUNDED : -ZB_UNBOUNDED;

	// everything once
	signatures.Resize(numSectors);
	sectorDirty.Resize(numSectors);
	for (unsigned i = 0; i < numSectors; i++)
	{
		signatures[i] = GetSignature(&Level->sectors[i]);
		sectorDirty[i] = 0;
		ComputeSector(Level, i);
	}
	for (unsigned i = 0; i < numSubsectors; i++) ComputeSubsector(Level, i);
	for (int n : postorder) ComputeNode(Level, n);
}

// Per frame: only sectors that changed (and the sectors using them for fake floors or 3D floors) are
// recomputed, then the subsectors showing them and the BSP nodes above those.
void FBspZBounds::Update(FLevelLocals *Level)
{
	dirtySectors.Clear();
	auto markDirty = [&](int s)
	{
		if (!sectorDirty[s]) { sectorDirty[s] = 1; dirtySectors.Push(s); }
	};
	for (unsigned i = 0; i < numSectors; i++)
	{
		const Signature sig = GetSignature(&Level->sectors[i]);
		if (sig != signatures[i])
		{
			signatures[i] = sig;
			markDirty(i);
			for (int j = depStart[i]; j < depStart[i + 1]; j++) markDirty(deps[j]);
		}
	}
	if (dirtySectors.Size() == 0) return;

	for (int s : dirtySectors) ComputeSector(Level, s);
	for (int s : dirtySectors)
	{
		sectorDirty[s] = 0;
		for (int j = secSubStart[s]; j < secSubStart[s + 1]; j++)
		{
			const int sub = secSubs[j];
			ComputeSubsector(Level, sub);
			for (int n = subParent[sub]; n >= 0; n = nodeParent[n]) ComputeNode(Level, n);
		}
	}
}

void UpdateBspZBounds(FLevelLocals *Level)
{
	FBspZBounds *zb = Level->BspZBounds;
	if ((!gl_frustum_cull_bsp && !gl_horizon_cull) || Level->subsectors.Size() == 0)
	{
		if (zb) zb->updated = false;
		return;
	}
	if (zb == nullptr) zb = Level->BspZBounds = new FBspZBounds;
	if (!zb->Matches(Level)) zb->Build(Level);
	else if (!zb->updated) zb->Build(Level);	// was switched off: anything may have changed meanwhile
	else zb->Update(Level);
	zb->updated = true;
}

//==========================================================================
//
// Is any part of this BSP child (node or subsector) inside the view frustum?
// bbox is the child's 2D box (node_t::bbox[side] or the subsector's).
//
//==========================================================================

bool HWDrawInfo::ChildInFrustum(void *child, const float *bbox)
{
	const float *z = ((size_t)child & 1)
		? &zBounds->subZ[((subsector_t *)((uint8_t *)child - 1))->Index() * 2]
		: &zBounds->nodeZ[((node_t *)child)->Index() * 2];
	return Frustum.MapBoxVisible(bbox[BOXLEFT] - 1., bbox[BOXRIGHT] + 1., bbox[BOXBOTTOM] - 1., bbox[BOXTOP] + 1.,
		z[0] - bspZPad, z[1] + bspZPad);
}

//==========================================================================
//
// Occlusion horizon (hw_horizon.h): hides the walls and flats of
// subsectors behind hills and under low ceilings. Main view only.
//
//==========================================================================

static HWHorizon mainHorizon;

static bool PlaneSolid(sector_t *sec, int plane)
{
	return sec->GetTexture(plane) != skyflatnum && sec->Portals[plane] == 0;
}

// Is every line of sight to the walls and flats of this BSP child (subsector or whole subtree) blocked by
// the ground and ceilings walked so far? bbox is the child's 2D box.
bool HWDrawInfo::HorizonHides(void *child, const float *bbox)
{
	const float *z = ((size_t)child & 1)
		? &zBounds->subZGeom[((subsector_t *)((uint8_t *)child - 1))->Index() * 2]
		: &zBounds->nodeZGeom[((node_t *)child)->Index() * 2];
	if (z[0] <= -ZB_UNBOUNDED || z[1] >= ZB_UNBOUNDED) return false;
	return horizon->BoxHidden(bbox[BOXLEFT] - 1., bbox[BOXRIGHT] + 1., bbox[BOXBOTTOM] - 1., bbox[BOXTOP] + 1., z[0] - 1., z[1] + 1.);
}

// Node-level version of the test, timed and counted. Subsectors are left to DoSubsector, which also has to
// feed the clipper and the horizon.
bool HWDrawInfo::HorizonChildHidden(void *child, const float *bbox)
{
	if ((size_t)child & 1) return false;
	horizonTime.Clock();
	const bool hidden = HorizonHides(child, bbox);
	horizonTime.Unclock();
	if (hidden) frustumStats.horizonNodes++;
	return hidden;
}

// The horizon costs time every frame it runs, and only pays when something is behind a hill or a
// building. When it hides almost nothing for a while, it sits out and only checks again now and then.
CVAR(Int, gl_horizon_minhidden, 32, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)	// subsectors per frame that make it worth running
static int horizonIdleFrames = 0;		// consecutive frames it hid less than that
static int horizonSkipFrames = 0;		// frames left to sit out
static constexpr int HORIZON_IDLE_LIMIT = 20;
static constexpr int HORIZON_RETRY_INTERVAL = 15;

static bool HorizonWorthRunning()
{
	if (gl_horizon_minhidden <= 0) return true;
	if (horizonSkipFrames > 0)
	{
		horizonSkipFrames--;
		frustumStats.horizonResting = true;
		return false;
	}
	return true;
}

static void HorizonFeedback(int hidden)
{
	if (hidden >= gl_horizon_minhidden)
	{
		horizonIdleFrames = 0;
		return;
	}
	if (++horizonIdleFrames >= HORIZON_IDLE_LIMIT)
	{
		horizonSkipFrames = HORIZON_RETRY_INTERVAL;	// try again after this, one frame at a time
		horizonIdleFrames = HORIZON_IDLE_LIMIT - 1;
	}
}

// Record this subsector's floor and ceiling as solid for everything behind it.
void HWDrawInfo::HorizonAdd(subsector_t *sub)
{
	sector_t *sec = sub->sector;
	// Render hacks (self-referencing sectors, Boom fake planes, faked render sectors) don't have solid planes.
	if (sub->hacked || sub->render_sector != sec || sec->GetHeightSec() != nullptr) return;
	const bool solidFloor = PlaneSolid(sec, sector_t::floor);
	const bool solidCeiling = PlaneSolid(sec, sector_t::ceiling);
	if (!solidFloor && !solidCeiling) return;

	// Far away ground hides little and costs as much as near ground, so only nearby ground is recorded.
	const double ex = horizon->ex, ey = horizon->ey, ez = horizon->ez;
	const double nx = ex < sub->bbox.Left() ? sub->bbox.Left() - ex : (ex > sub->bbox.Right() ? ex - sub->bbox.Right() : 0);
	const double ny = ey < sub->bbox.Bottom() ? sub->bbox.Bottom() - ey : (ey > sub->bbox.Top() ? ey - sub->bbox.Top() : 0);
	const double range = gl_horizon_range;
	if (range > 0 && nx * nx + ny * ny > range * range) return;

	const double cx = (sub->bbox.Left() + sub->bbox.Right()) * 0.5, cy = (sub->bbox.Bottom() + sub->bbox.Top()) * 0.5;
	for (uint32_t i = 0; i < sub->numlines; i++)
	{
		seg_t *seg = &sub->firstline[i];
		if (seg->v1 == nullptr || seg->v2 == nullptr) continue;
		// Only edges that can hide anything: steps, terrain ridges, solid ceilings (see FBspZBounds::Build).
		const uint8_t occ = zBounds->segOccluder[seg->Index()];
		if (!occ) continue;
		const double x1 = seg->v1->fX(), y1 = seg->v1->fY(), x2 = seg->v2->fX(), y2 = seg->v2->fY();
		const double fz = min(sec->floorplane.ZatPoint(seg->v1), sec->floorplane.ZatPoint(seg->v2));
		const double cz = max(sec->ceilingplane.ZatPoint(seg->v1), sec->ceilingplane.ZatPoint(seg->v2));

		// Every line of sight through the subsector crosses one near and one far edge. Only the one
		// that gives the tighter limit is needed: the far edge for ground below the eye, the near one
		// for ground above it (and the other way round for ceilings).
		const double lx = x2 - x1, ly = y2 - y1;
		const double sideEye = lx * (horizon->ey - y1) - ly * (horizon->ex - x1);
		const double sideCentre = lx * (cy - y1) - ly * (cx - x1);
		const bool nearEdge = (sideEye > 0) != (sideCentre > 0);
		const bool useFloor = solidFloor && (occ & SEGOCC_FLOOR) && (nearEdge ? fz >= ez : fz < ez);
		const bool useCeiling = solidCeiling && (occ & SEGOCC_CEILING) && (nearEdge ? cz <= ez : cz > ez);
		horizon->AddEdge(x1, y1, x2, y2, fz, cz, useFloor, useCeiling);
	}
}

thread_local bool isWorkerThread;
ctpl::thread_pool renderPool(1);
bool inited = false;

const int MAXDITHERACTORS = 20; // Maximum number of enemies that can set dither-transparency flags
AActor* RenderedTargets[MAXDITHERACTORS];
int RTnum;

void ClearDitherTargets()
{
	RTnum = 0; // Number of rendered enemies/targets
	for (int ii = 0; ii < MAXDITHERACTORS; ii++)
	  RenderedTargets[ii] = nullptr;
}

struct RenderJob
{
	enum
	{
		FlatJob,
		WallJob,
		SpriteJob,
		ParticleJob,
		PortalJob,
		TerminateJob	// inserted when all work is done so that the worker can return.
	};

	int type;
	subsector_t *sub;
	seg_t *seg;
	bool isculled;
};


class RenderJobQueue
{
	RenderJob pool[300000];	// Way more than ever needed. The largest ever seen on a single viewpoint is around 40000.
	std::atomic<int> readindex{};
	std::atomic<int> writeindex{};
public:
	void AddJob(int type, subsector_t *sub, seg_t *seg = nullptr, bool isculled = false)
	{
		// This does not check for array overflows. The pool should be large enough that it never hits the limit.

		pool[writeindex] = { type, sub, seg, isculled };
		writeindex++;	// update index only after the value has been written.
	}

	RenderJob *GetJob()
	{
		if (readindex < writeindex) return &pool[readindex++];
		return nullptr;
	}

	void ReleaseAll()
	{
		readindex = 0;
		writeindex = 0;
	}
};

static RenderJobQueue jobQueue;	// One static queue is sufficient here. This code will never be called recursively.

void HWDrawInfo::WorkerThread()
{
	sector_t *front, *back;
	HWWallDispatcher disp(this);

	WTTotal.Clock();
	isWorkerThread = true;	// for adding asserts in GL API code. The worker thread may never call any GL API.
	while (true)
	{
		auto job = jobQueue.GetJob();
		if (job == nullptr)
		{
#ifdef ARCH_IA32
			// The queue is empty. But yielding would be too costly here and possibly cause further delays down the line if the thread is halted.
			// So instead add a few pause instructions and retry immediately.
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
#endif // ARCH_IA32
		}
		// Note that the main thread MUST have prepared the fake sectors that get used below!
		// This worker thread cannot prepare them itself without costly synchronization.
		else switch (job->type)
		{
		case RenderJob::TerminateJob:
			WTTotal.Unclock();
			return;

		case RenderJob::WallJob:
		{
			HWWall wall;
			SetupWall.Clock();
			wall.sub = job->sub;

			front = hw_FakeFlat(job->sub->sector, in_area, false);
			auto seg = job->seg;
			auto backsector = seg->backsector;
			if (!backsector && seg->linedef->isVisualPortal() && seg->sidedef == seg->linedef->sidedef[0]) // For one-sided portals use the portal's destination sector as backsector.
			{
				auto portal = seg->linedef->getPortal();
				backsector = portal->mDestination->frontsector;
				back = hw_FakeFlat(backsector, in_area, true);
				if (front->floorplane.isSlope() || front->ceilingplane.isSlope() || back->floorplane.isSlope() || back->ceilingplane.isSlope())
				{
					// Having a one-sided portal like this with slopes is too messy so let's ignore that case.
					back = nullptr;
				}
			}
			else if (backsector)
			{
				if (front->sectornum == backsector->sectornum || (seg->sidedef->Flags & WALLF_POLYOBJ))
				{
					back = front;
				}
				else
				{
					back = hw_FakeFlat(backsector, in_area, true);
				}
			}
			else back = nullptr;

			wall.Process(&disp, job->seg, front, back, job->isculled);
			rendered_lines++;
			SetupWall.Unclock();
			break;
		}

		case RenderJob::FlatJob:
		{
			HWFlat flat;
			SetupFlat.Clock();
			flat.section = job->sub->section;
			front = hw_FakeFlat(job->sub->render_sector, in_area, false);
			flat.ProcessSector(this, front);
			SetupFlat.Unclock();
			break;
		}

		case RenderJob::SpriteJob:
			SetupSprite.Clock();
			front = hw_FakeFlat(job->sub->sector, in_area, false);
			RenderThings(job->sub, front);
			SetupSprite.Unclock();
			break;

		case RenderJob::ParticleJob:
			SetupSprite.Clock();
			front = hw_FakeFlat(job->sub->sector, in_area, false);
			RenderParticles(job->sub, front);
			SetupSprite.Unclock();
			break;

		case RenderJob::PortalJob:
			AddSubsectorToPortal((FSectorPortalGroup *)job->seg, job->sub);
			break;
		}

	}
}

static bool IsManuallyCulled(FRenderViewpoint vp, subsector_t* sub)
{
	int       count  = sub->numlines;
	seg_t    *seg    = sub->firstline;
	sector_t *sector = sub->sector;

	if (!sector || !vp.culledSectors.Size())
		return false;
	
	int  tagCount = level.tagManager.CountSectorTags(sector);
	for (int i = 0; i < tagCount; i++)
	{
		if (vp.culledSectors.Contains(level.tagManager.GetSectorTag(sector, i)))
			return true;
	}
	return false;
}

EXTERN_CVAR(Bool, gl_render_segs)

CVAR(Bool, gl_render_things, true, 0)
EXTERN_CVAR(Bool, gl_render_foliage)
CVAR(Bool, gl_render_walls, true, 0)
CVAR(Bool, gl_render_flats, true, 0)

void HWDrawInfo::UnclipSubsector(subsector_t *sub)
{
	int count = sub->numlines;
	seg_t * seg = sub->firstline;
	auto &clipper = *mClipper;

	while (count--)
	{
		angle_t startAngle = clipper.GetClipAngle(seg->v2);
		angle_t endAngle = clipper.GetClipAngle(seg->v1);

		// Back side, i.e. backface culling	- read: endAngle >= startAngle!
		if (startAngle-endAngle >= ANGLE_180)
		{
			clipper.SafeRemoveClipRange(startAngle, endAngle);
			clipper.SetBlocked(false);
		}
		seg++;
	}
}

//==========================================================================
//
// R_AddLine
// Clips the given segment
// and adds any visible pieces to the line list.
//
//==========================================================================

EXTERN_CVAR(Int, r_distance_cull_type)
EXTERN_CVAR(Float, r_line_distance_cull)

bool HWDrawInfo::IsDistanceCulled(seg_t *line)
{
	if (Viewpoint.culldistsq <= 0.0)
		return false;

	DVector2 VPos = Viewpoint.Pos.XY();
	if (Viewpoint.bDoOob && r_radarclipper && !(Level->flags3 & LEVEL3_NOFOGOFWAR))
	{
		VPos = Viewpoint.OffPos.XY();
	}
	DVector2 vec1 = (line->linedef->v1->fPos() - VPos);
	DVector2 vec2 = (line->linedef->v2->fPos() - VPos);
	double dist1 = vec1.LengthSquared();
	double dist2 = vec2.LengthSquared();
	if ((dist1 > Viewpoint.culldistsq) && (dist2 > Viewpoint.culldistsq))
	{
		if (vec1.dot(vec2) > 0.0) // Angle subtended by the line at the viewpoint less than 90-degrees
		{
			return true;
		}
		else
		{
			DVector2 linevec = line->linedef->Delta();
			double linelensq = linevec.LengthSquared();
			if (linelensq > 0.0)
			{
				double projlen = -vec1.dot(linevec) / linelensq;
				DVector2 closest = line->linedef->v1->fPos() + linevec * projlen;
				if ((VPos- closest).LengthSquared() > Viewpoint.culldistsq)
				{
					return true;
				}
			}
		}
	}
	return false;
}

//==========================================================================
//
// A two-sided line whose floors and ceilings meet exactly on both sides
// (the edges between triangles of a terrain mesh, for example) has no
// wall to draw and never blocks the view, so all the per-line work below
// can be skipped. Checked every time, so moving planes are handled.
//
//==========================================================================

CVAR(Bool, gl_seamless_lines, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)

bool IsSeamlessLine(seg_t *seg)
{
	line_t *line = seg->linedef;
	sector_t *front = seg->frontsector, *back = seg->backsector;
	if (line == nullptr || back == nullptr || front == nullptr || front == back) return false;
	if (line->special != 0 || line->isVisualPortal() || line->sidedef[0] == nullptr || line->sidedef[1] == nullptr) return false;
	for (side_t *side : line->sidedef)
	{
		if ((side->Flags & (WALLF_POLYOBJ | WALLF_BLOCKRENDERING)) || side->GetTexture(side_t::mid).isValid()) return false;
	}
	if (front->GetHeightSec() || back->GetHeightSec()) return false;
	if (front->e->XFloor.ffloors.Size() || back->e->XFloor.ffloors.Size()) return false;
	if (front->e->XFloor.attached.Size() || back->e->XFloor.attached.Size()) return false;
	if (front->Portals[sector_t::floor] != back->Portals[sector_t::floor] || front->Portals[sector_t::ceiling] != back->Portals[sector_t::ceiling]) return false;
	if ((front->GetTexture(sector_t::floor) == skyflatnum) != (back->GetTexture(sector_t::floor) == skyflatnum)) return false;
	if ((front->GetTexture(sector_t::ceiling) == skyflatnum) != (back->GetTexture(sector_t::ceiling) == skyflatnum)) return false;

	const double eps = 1. / 256.;
	for (vertex_t *v : { seg->v1, seg->v2 })
	{
		if (fabs(front->floorplane.ZatPoint(v) - back->floorplane.ZatPoint(v)) > eps) return false;
		if (fabs(front->ceilingplane.ZatPoint(v) - back->ceilingplane.ZatPoint(v)) > eps) return false;
	}
	return true;
}

void HWDrawInfo::AddLine (seg_t *seg, bool portalclip)
{
	const bool doOob = Viewpoint.bDoOob;
	bool isculled = false;

#ifdef _DEBUG
	if (seg->linedef && seg->linedef->Index() == 38)
	{
		int a = 0;
	}
#endif

	sector_t * backsector = nullptr;

	if (portalclip)
	{
		int clipres = mClipPortal->ClipSeg(seg, Viewpoint.Pos);
		if (clipres == PClip_InFront) return;
	}

	auto &clipper = *mClipper;
	angle_t startAngle = clipper.GetClipAngle(seg->v2);
	angle_t endAngle = clipper.GetClipAngle(seg->v1);
	auto &clipperr = *rClipper;
	angle_t startAngleR = 0;
	angle_t endAngleR = 0;
	angle_t paddingR = 0x00200000; // Make radar clipping more aggressive (reveal less)

	if(doOob && r_radarclipper && !(Level->flags3 & LEVEL3_NOFOGOFWAR))
	{
		startAngleR = clipperr.PointToPseudoAngle(seg->v2->fX(), seg->v2->fY());
		endAngleR = clipperr.PointToPseudoAngle(seg->v1->fX(), seg->v1->fY());

		if (startAngleR - endAngleR >= ANGLE_180)
		{
			if (!seg->backsector) clipperr.SafeAddClipRange(startAngleR - paddingR, endAngleR + paddingR);
			else if((seg->sidedef != nullptr) && !uint8_t(seg->sidedef->Flags & WALLF_POLYOBJ) && (currentsector->sectornum != seg->backsector->sectornum))
			{
				if (in_area == area_default) in_area = hw_CheckViewArea(seg->v1, seg->v2, seg->frontsector, seg->backsector);
				backsector = hw_FakeFlat(seg->backsector, in_area, true);
				if (hw_CheckClip(seg->sidedef, currentsector, backsector)) clipperr.SafeAddClipRange(startAngleR - paddingR, endAngleR + paddingR);
				backsector = nullptr;
			}
		}
	}

	// Back side, i.e. backface culling	- read: endAngle >= startAngle!
	if (startAngle-endAngle<ANGLE_180)
	{
		return;
	}

	if (seg->sidedef == nullptr)
	{
		if (!(currentsubsector->flags & SSECMF_DRAWN))
		{
			if (clipper.SafeCheckRange(startAngle, endAngle) && !doOob)
			{
			  currentsubsector->flags |= SSECMF_DRAWN;
			}
			if (doOob && ((r_radarclipper && !(Level->flags3 & LEVEL3_NOFOGOFWAR)) || (r_distance_cull_type > 0)) && clipperr.SafeCheckRange(startAngleR, endAngleR))
			{
			  currentsubsector->flags |= SSECMF_DRAWN;
			}
		}
		return;
	}

	if (!clipper.SafeCheckRange(startAngle, endAngle))
	{
		return;
	}

	if (!doOob || (!r_radarclipper || (Level->flags3 & LEVEL3_NOFOGOFWAR) || clipperr.SafeCheckRange(startAngleR, endAngleR)))
		currentsubsector->flags |= SSECMF_DRAWN;

	uint8_t ispoly = uint8_t(seg->sidedef->Flags & WALLF_POLYOBJ);

	if ((r_distance_cull_type > 0 && !isNoLineCullMode() && IsDistanceCulled(seg)) ||
	    IsManuallyCulled(this->Viewpoint, seg->Subsector))
	{
		isculled = true;
		if(doOob)
		{
			if (startAngleR == 0) startAngleR = clipperr.PointToPseudoAngle(seg->v2->fX(), seg->v2->fY());
			if (endAngleR == 0) endAngleR = clipperr.PointToPseudoAngle(seg->v1->fX(), seg->v1->fY());
			if (startAngleR - endAngleR >= ANGLE_180) clipperr.SafeAddClipRange(startAngleR - paddingR, endAngleR + paddingR);
		}
		else
		{
			clipper.SafeAddClipRange(startAngle, endAngle);
		}
		if (seg->frontsector->Colormap.FadeColor != 0) Level->cullcolor = seg->frontsector->Colormap.FadeColor; // Write can't be multithreaded
	}

	// Seamless: nothing to draw and nothing to clip.
	if (gl_seamless_lines && !ispoly && IsSeamlessLine(seg))
	{
		frustumStats.seamlessLines++;
		if (!clipOnly)
		{
			seg->linedef->flags |= ML_MAPPED;
			seg->linedef->validcount = validcount;
		}
		return;
	}

	// [XA] NOTE: ideally it'd be nice to collapse these checks into one,
	// but it's possible to add & remove WALLF_BLOCKRENDERING via zscript
	// so auto-setting it on 1s lines may result in a crash if someone
	// later tries to remove the flag from a line.
	if (!seg->backsector || (seg->sidedef->Flags & WALLF_BLOCKRENDERING))
	{
		if(!doOob)
			if (!(seg->sidedef->Flags & WALLF_DITHERTRANS_MID)) clipper.SafeAddClipRange(startAngle, endAngle);
	}
	else if (!ispoly)	// Two-sided polyobjects never obstruct the view
	{
		if (currentsector->sectornum == seg->backsector->sectornum)
		{
			if (!seg->linedef->isVisualPortal())
			{
				auto tex = TexMan.GetGameTexture(seg->sidedef->GetTexture(side_t::mid), true);
				if (!tex || !tex->isValid()) 
				{
					// nothing to do here!
					seg->linedef->validcount=validcount;
					return;
				}
			}
			backsector=currentsector;
		}
		else
		{
			// clipping checks are only needed when the backsector is not the same as the front sector
			if (in_area == area_default) in_area = hw_CheckViewArea(seg->v1, seg->v2, seg->frontsector, seg->backsector);

			backsector = hw_FakeFlat(seg->backsector, in_area, true);

			if (hw_CheckClip(seg->sidedef, currentsector, backsector))
			{
				if(!doOob && !(seg->sidedef->Flags & WALLF_DITHERTRANS_MID))
					clipper.SafeAddClipRange(startAngle, endAngle);
			}
		}
	}
	else
	{
		// Backsector for polyobj segs is always the containing sector itself
		backsector = currentsector;
	}

	// Hidden behind the horizon: the wall has done its job as an occluder for the clipper above, but isn't drawn.
	// The line isn't marked as processed, so its other side can still draw it if that one is visible.
	if (clipOnly) return;

	// lodonly (hw_worldlod.cpp): the world LOD model draws this line's walls at any distance. The line has clipped
	// above like any other, and that is all the walk does with it.
	if (lodOnly && !ispoly && HWLineLodOnly(seg->linedef))
	{
		seg->linedef->validcount = validcount;
		return;
	}

	seg->linedef->flags |= ML_MAPPED;

	if (ispoly || seg->linedef->validcount!=validcount)
	{
		if (!ispoly) seg->linedef->validcount=validcount;

		if (gl_render_walls)
		{
			if (multithread)
			{
				jobQueue.AddJob(RenderJob::WallJob, seg->Subsector, seg, isculled);
			}
			else
			{
				HWWall wall;
				HWWallDispatcher disp(this);
				SetupWall.Clock();
				wall.sub = seg->Subsector;
				wall.Process(&disp, seg, currentsector, backsector, isculled);
				rendered_lines++;
				SetupWall.Unclock();
			}
		}
	}
}

//==========================================================================
//
// R_Subsector
// Determine floor/ceiling planes.
// Add sprites of things in sector.
// Draw one or more line segments.
//
//==========================================================================

void HWDrawInfo::PolySubsector(subsector_t * sub)
{
	int count = sub->numlines;
	seg_t * line = sub->firstline;

	while (count--)
	{
		if (line->linedef)
		{
			AddLine (line, mClipPortal != nullptr);
		}
		line++;
	}
}

//==========================================================================
//
// RenderBSPNode
// Renders all subsectors below a given node,
//  traversing subtree recursively.
// Just call with BSP root.
//
//==========================================================================

void HWDrawInfo::RenderPolyBSPNode (void *node)
{
	while (!((size_t)node & 1))  // Keep going until found a subsector
	{
		node_t *bsp = (node_t *)node;

		// Decide which side the view point is on.
		int side = R_PointOnSide(viewx, viewy, bsp);

		// Recursively divide front space (toward the viewer).
		RenderPolyBSPNode (bsp->children[side]);

		// Possibly divide back space (away from the viewer).
		side ^= 1;

		// It is not necessary to use the slower precise version here
		if (!mClipper->CheckBox(bsp->bbox[side]))
		{
			return;
		}

		node = bsp->children[side];
	}
	PolySubsector ((subsector_t *)((uint8_t *)node - 1));
}

//==========================================================================
//
// Unlilke the software renderer this function will only draw the walls,
// not the flats. Those are handled as a whole by the parent subsector.
//
//==========================================================================

void HWDrawInfo::AddPolyobjs(subsector_t *sub)
{
	if (sub->BSP == nullptr || sub->BSP->bDirty)
	{
		sub->BuildPolyBSP();
	}
	if (sub->BSP->Nodes.Size() == 0)
	{
		PolySubsector(&sub->BSP->Subsectors[0]);
	}
	else
	{
		RenderPolyBSPNode(&sub->BSP->Nodes.Last());
	}
}


//==========================================================================
//
//
//
//==========================================================================

void HWDrawInfo::AddLines(subsector_t * sub, sector_t * sector)
{
	currentsector = sector;
	currentsubsector = sub;
	ClipWall.Clock();
	if (sub->polys != nullptr)
	{
		AddPolyobjs(sub);
	}
	else
	{
		int count = sub->numlines;
		seg_t * seg = sub->firstline;
		// With the line distance cull on, batched segs past the distance must still go through AddLine: a
		// distance-culled line closes its angle range in the clipper, and that is what stops the BSP walk at the
		// cull distance. Skipping them let the walk run on through all the open terrain beyond.
		const bool lineCull = r_distance_cull_type > 0 && !isNoLineCullMode();

		while (count--)
		{
			if (seg->linedef == nullptr)
			{
				if (!(sub->flags & SSECMF_DRAWN)) AddLine (seg, mClipPortal != nullptr);
			}
			else if (terrainBatches && SegBatched(seg) && !(lineCull && IsDistanceCulled(seg)))
			{
				// Between two batched floors with nothing in between: no wall, no clipping, no work.
				terrainStats.segs++;
				sub->flags |= SSECMF_DRAWN;
			}
			else if (!(seg->sidedef->Flags & WALLF_POLYOBJ))
			{
				AddLine (seg, mClipPortal != nullptr);
			}
			seg++;
		}
	}
	ClipWall.Unclock();
}

//==========================================================================
//
// Adds lines that lie directly on the portal boundary.
// Only two-sided lines will be handled here, and no polyobjects
//
//==========================================================================

inline bool PointOnLine(const DVector2 &pos, const linebase_t *line)
{
	double v = (pos.Y - line->v1->fY()) * line->Delta().X + (line->v1->fX() - pos.X) * line->Delta().Y;
	return fabs(v) <= EQUAL_EPSILON;
}

void HWDrawInfo::AddSpecialPortalLines(subsector_t * sub, sector_t * sector, linebase_t *line)
{
	currentsector = sector;
	currentsubsector = sub;

	ClipWall.Clock();
	int count = sub->numlines;
	seg_t * seg = sub->firstline;

	while (count--)
	{
		if (seg->linedef != nullptr && seg->PartnerSeg != nullptr)
		{
			if (PointOnLine(seg->v1->fPos(), line) && PointOnLine(seg->v2->fPos(), line))
				AddLine(seg, false);
		}
		seg++;
	}
	ClipWall.Unclock();
}


//==========================================================================
//
// R_RenderThings
//
//==========================================================================

void HWDrawInfo::RenderThings(subsector_t * sub, sector_t * sector)
{
	sector_t * sec=sub->sector;
	// Handle all things in sector.
	const auto &vp = Viewpoint;
	for (auto p = sec->touching_renderthings; p != nullptr; p = p->m_snext)
	{
		auto thing = p->m_thing;
		if (thing->validcount == validcount) continue;
		thing->validcount = validcount;
		if (HideThings::HideActive && HideThings::IsHidden(thing->GetClass())) continue;

		if(Viewpoint.bDoOob && thing->Sector->isSecret() && thing->Sector->wasSecret() && !r_radarclipper) continue; // This covers things that are touching non-secret sectors
		FIntCVar *cvar = thing->GetInfo()->distancecheck;
		bool      doDistCheck = (cvar != nullptr && *cvar >= 0);
		if (doDistCheck)
		{
			double dist = (thing->Pos() - (vp.bDoOob ? vp.OffPos : vp.Pos)).LengthSquared();
			double check = (double)**cvar;
			if (dist >= check * check)
			{
				continue;
			}
		}
		// Level of detail is picked in HWSprite::Process.
		if (r_distance_cull_type > 0)
		{
			bool renderradcheck = thing->RenderRadius() < 0.25 * vp.culldist;
			double dist2 = renderradcheck ? (thing->Pos() - (vp.bDoOob ? vp.OffPos : vp.Pos)).LengthSquared() :
				(thing->Pos() - (vp.bDoOob ? vp.OffPos : vp.Pos)).Length() - 1.414 * thing->RenderRadius();
			if (dist2 >= (renderradcheck ? vp.culldistsq : vp.culldist))
			{
				continue;
			}
		}
		// If this thing is in a map section that's not in view it can't possibly be visible
		if (CurrentMapSections[thing->subsector->mapsection])
		{
			HWSprite sprite;

			// [Nash] draw sprite shadow
			if (R_ShouldDrawSpriteShadow(thing))
			{
				double dist = (thing->Pos() - vp.Pos).LengthSquared();
				double check = r_actorspriteshadowdist;
				if (dist <= check * check)
				{
					sprite.Process(this, thing, sector, in_area, false, true);
				}
			}

			sprite.Process(this, thing, sector, in_area, false);
		}
	}

	for (msecnode_t *node = sec->sectorportal_thinglist; node; node = node->m_snext)
	{
		AActor *thing = node->m_thing;
		if (HideThings::HideActive && HideThings::IsHidden(thing->GetClass())) continue;
		FIntCVar *cvar = thing->GetInfo()->distancecheck;
		if (cvar != nullptr && *cvar >= 0)
		{
			double dist = (thing->Pos() - vp.Pos).LengthSquared();
			double check = (double)**cvar;
			if (dist >= check * check)
			{
				continue;
			}
		}

		HWSprite sprite;

		// [Nash] draw sprite shadow
		if (R_ShouldDrawSpriteShadow(thing))
		{
			double dist = (thing->Pos() - vp.Pos).LengthSquared();
			double check = r_actorspriteshadowdist;
			if (dist <= check * check)
			{
				sprite.Process(this, thing, sector, in_area, true, true);
			}
		}

		sprite.Process(this, thing, sector, in_area, true);
	}
}

void HWDrawInfo::RenderParticles(subsector_t *sub, sector_t *front)
{
	SetupSprite.Clock();
	for (uint32_t i = 0; i < sub->sprites.Size(); i++)
	{
		DVisualThinker *sp = sub->sprites[i];
		if (!sp || sp->ObjectFlags & OF_EuthanizeMe)
			continue;
		if (HideThings::HideActive && HideThings::IsHidden(sp->GetClass())) continue;
		if (r_distance_cull_type > 0)
		{
			double dist = (sp->PT.Pos - (Viewpoint.bDoOob ? Viewpoint.OffPos : Viewpoint.Pos)).LengthSquared();
			if (dist >= Viewpoint.culldistsq)
			{
				continue;
			}
		}
		if (mClipPortal)
		{
			int clipres = mClipPortal->ClipPoint(sp->PT.Pos.XY());
			if (clipres == PClip_InFront) continue;
		}

		HWSprite sprite;
		sprite.ProcessParticle(this, &sp->PT, front, sp);
	}
	for (int i = HideThings::HideParticles ? NO_PARTICLE : Level->ParticlesInSubsec[sub->Index()]; i != NO_PARTICLE; i = Level->Particles[i].snext)
	{
		if (r_distance_cull_type > 0)
		{
			double dist2 = (Level->Particles[i].Pos - (Viewpoint.bDoOob ? Viewpoint.OffPos : Viewpoint.Pos)).LengthSquared();
			if (dist2 >= Viewpoint.culldistsq)
			{
				continue;
			}
		}
		if (mClipPortal)
		{
			int clipres = mClipPortal->ClipPoint(Level->Particles[i].Pos.XY());
			if (clipres == PClip_InFront) continue;
		}

		HWSprite sprite;
		sprite.ProcessParticle(this, &Level->Particles[i], front, nullptr);
	}
	SetupSprite.Unclock();
}


//==========================================================================
//
// R_Subsector
// Determine floor/ceiling planes.
// Add sprites of things in sector.
// Draw one or more line segments.
//
//==========================================================================

void HWDrawInfo::DoSubsector(subsector_t * sub)
{
	const bool doOob = Viewpoint.bDoOob;
	sector_t * sector;
	sector_t * fakesector;

#ifdef _DEBUG
	if (sub->sector->sectornum==931)
	{
		int a = 0;
	}
#endif

	sector=sub->sector;
	if (!sector) return;

	// If the mapsections differ this subsector can't possibly be visible from the current view point
	if (!CurrentMapSections[sub->mapsection]) return;
	if (sub->flags & SSECF_POLYORG) return;	// never render polyobject origin subsectors because their vertices no longer are where one may expect.

	if (ss_renderflags[sub->Index()] & SSRF_SEEN)
	{
		// This means that we have reached a subsector in a portal that has been marked 'seen'
		// from the other side of the portal. This means we must clear the clipper for the
		// range this subsector spans before going on.
		UnclipSubsector(sub);
	}
	if (mClipper->IsBlocked()) return;	// if we are inside a stacked sector portal which hasn't unclipped anything yet.

	// Entirely outside the 3D view frustum. Polyobjects move, and subsectors marked 'seen' through a portal
	// must still unclip (above), so those are left alone.
	if (frustum3D && sub->polys == nullptr && !(ss_renderflags[sub->Index()] & SSRF_SEEN))
	{
		const float box[4] = { (float)sub->bbox.Top(), (float)sub->bbox.Bottom(), (float)sub->bbox.Left(), (float)sub->bbox.Right() };
		if (!ChildInFrustum((uint8_t *)sub + 1, box))
		{
			frustumStats.subsectors++;
			return;
		}
	}

	// Inside a subtree hidden behind the horizon only actors, particles and foliage are wanted.
	if (thingsOnly)
	{
		horizonHiddenCount++;
		const bool hasParticles = sub->sprites.Size() > 0 || Level->ParticlesInSubsec[sub->Index()] != NO_PARTICLE;
		const bool hasFoliage = Level->Foliage != nullptr && Level->Foliage->SubsectorHasFoliage(sub->Index());
		const bool hasThings = sector->validcount != validcount && (sector->touching_renderthings || sector->sectorportal_thinglist);
		if (!gl_render_things || (!hasParticles && !hasFoliage && !hasThings)) return;
	}

	fakesector=hw_FakeFlat(sector, in_area, false);
	if(doOob && sector->isSecret() && sector->wasSecret() && !r_radarclipper) return;

	// cull everything if subsector outside all relevant clippers
	if (doOob && (sub->polys == nullptr))
	{
		auto &clipper = *mClipper;
		auto &clipperv = *vClipper;
		auto &clipperr = *rClipper;
		int count = sub->numlines;
		seg_t * seg = sub->firstline;
		bool anglevisible = false;
		bool pitchvisible = false;
		bool radarvisible = !r_radarclipper || (Level->flags3 & LEVEL3_NOFOGOFWAR) || ((sub->flags & SSECMF_DRAWN) && !deathmatch);
		const int reflect = mCurrentPortal ? mCurrentPortal->GetMirrorSide() : 0; // -1 = ceiling, 1 = floor, 0 = none
		const double planez = (reflect < 0 ? sector->ceilingplane.ZatPoint(Viewpoint.Pos) : sector->floorplane.ZatPoint(Viewpoint.Pos));
		angle_t pitchtemp;
		angle_t pitchmin = ANGLE_90;
		angle_t pitchmax = 0;

		while (count--)
		{
			if((seg->v1 != nullptr) && (seg->v2 != nullptr))
			{
				if (!anglevisible)
				{
					angle_t startAngle = clipper.GetClipAngle(seg->v2);
					angle_t endAngle = clipper.GetClipAngle(seg->v1);
					if (startAngle-endAngle >= ANGLE_180) anglevisible |= clipper.SafeCheckRange(startAngle, endAngle);
				}
				if (!radarvisible)
				{
					angle_t startAngleR = clipperr.PointToPseudoAngle(seg->v2->fX(), seg->v2->fY());
					angle_t endAngleR = clipperr.PointToPseudoAngle(seg->v1->fX(), seg->v1->fY());
					if (startAngleR-endAngleR >= ANGLE_180) radarvisible |= clipperr.SafeCheckRange(startAngleR, endAngleR);
				}

				if (!pitchvisible)
				{
					pitchmin = clipperv.PointToPseudoPitch(seg->v1->fX(), seg->v1->fY(),
														   (reflect != 0) ?
														   2 * planez - sector->floorplane.ZatPoint(seg->v1) :
														   sector->floorplane.ZatPoint(seg->v1));
					pitchmax = clipperv.PointToPseudoPitch(seg->v1->fX(), seg->v1->fY(),
														   (reflect != 0) ?
														   2 * planez - sector->ceilingplane.ZatPoint(seg->v1) :
														   sector->ceilingplane.ZatPoint(seg->v1));
					pitchvisible |= clipperv.SafeCheckRange(pitchmin, pitchmax);
				}
				if (pitchvisible && anglevisible && radarvisible) break;
				if (!pitchvisible)
				{
					pitchtemp = clipperv.PointToPseudoPitch(seg->v2->fX(), seg->v2->fY(),
															(reflect != 0) ?
															2 * planez - sector->floorplane.ZatPoint(seg->v2) :
															sector->floorplane.ZatPoint(seg->v2));
					if (int(pitchmin) > int(pitchtemp)) pitchmin = pitchtemp;
					pitchtemp = clipperv.PointToPseudoPitch(seg->v2->fX(), seg->v2->fY(),
															(reflect != 0) ?
															2 * planez - sector->ceilingplane.ZatPoint(seg->v2) :
															sector->ceilingplane.ZatPoint(seg->v2));
					if (int(pitchmax) < int(pitchtemp)) pitchmax = pitchtemp;
					pitchvisible |= clipperv.SafeCheckRange(pitchmin, pitchmax);
				}
				if (pitchvisible && anglevisible && radarvisible) break;
			}
			seg++;
		}
		// Skip subsector if outside vertical or horizontal clippers or is in unexplored territory (fog of war)
		if(!pitchvisible || !anglevisible || (!radarvisible && r_radarclipper)) return;
	}

	if (mClipPortal)
	{
		int clipres = mClipPortal->ClipSubsector(sub);
		if (clipres == PClip_InFront)
		{
			auto line = mClipPortal->ClipLine();
			// The subsector is out of range, but we still have to check lines that lie directly on the boundary and may expose their upper or lower parts.
			if (line) AddSpecialPortalLines(sub, fakesector, line);
			return;
		}
	}

	// Behind a hill or under a low ceiling: skip the walls and flats, but keep the actors, particles and
	// foliage, which have their own culling and can stick up above the ground in front.
	bool horizonHidden = thingsOnly;
	if (horizon != nullptr && !thingsOnly)
	{
		horizonTime.Clock();
		if (sub->polys == nullptr && !(ss_renderflags[sub->Index()] & SSRF_SEEN))
		{
			const float box[4] = { (float)sub->bbox.Top(), (float)sub->bbox.Bottom(), (float)sub->bbox.Left(), (float)sub->bbox.Right() };
			if (HorizonHides((uint8_t *)sub + 1, box))
			{
				horizonHidden = true;
				frustumStats.horizonSubsectors++;
				horizonHiddenCount++;
			}
		}
		HorizonAdd(sub);
		horizonTime.Unclock();
	}

	if (sector->validcount != validcount)
	{
		CheckUpdate(screen->mVertexData, sector);
	}

	// [RH] Add particles
	if (gl_render_things && (sub->sprites.Size() > 0 || Level->ParticlesInSubsec[sub->Index()] != NO_PARTICLE))
	{
		if (multithread)
		{
			jobQueue.AddJob(RenderJob::ParticleJob, sub, nullptr);
		}
		else
		{
			SetupSprite.Clock();
			RenderParticles(sub, fakesector);
			SetupSprite.Unclock();
		}
	}

	// Foliage clusters: just note the subsector; DrawFoliage processes its plants in one pass.
	if (gl_render_things && gl_render_foliage && Level->Foliage != nullptr && Level->Foliage->SubsectorHasFoliage(sub->Index()))
	{
		FoliageSubsectors.Push(sub->Index());
	}

	// A hidden subsector's solid walls still block everything behind them in the clipper, so add them
	// to it without building the walls.
	// (A whole hidden subtree skips this too: what's behind it gets its own horizon test.)
	if (!thingsOnly)
	{
		clipOnly = horizonHidden;
		AddLines(sub, fakesector);
		clipOnly = false;
	}

	// BSP is traversed by subsector.
	// A sector might have been split into several
	//	subsectors during BSP building.
	// Thus we check whether it was already added.
	if (sector->validcount != validcount)
	{
		// Well, now it will be done.
		sector->validcount = validcount;
		sector->MoreFlags |= SECMF_DRAWN;

		if (gl_render_things && (sector->touching_renderthings || sector->sectorportal_thinglist))
		{
			if (multithread)
			{
				jobQueue.AddJob(RenderJob::SpriteJob, sub, nullptr);
			}
			else
			{
				SetupSprite.Clock();
				RenderThings(sub, fakesector);
				SetupSprite.Unclock();
			}
		}
		if (r_dithertransparency && doOob && (RTnum < MAXDITHERACTORS) && mCurrentPortal == nullptr)
		{
			// [DVR] Not parallelizable due to variables RTnum and RenderedTargets[]
			for (auto p = sector->touching_renderthings; p != nullptr; p = p->m_snext)
			{
				auto thing = p->m_thing;
				if (thing->validcount == validcount) continue; // Don't double count
				if (((thing->flags3 & MF3_ISMONSTER) && !(thing->flags & MF_CORPSE)) || (thing->flags & MF_MISSILE))
				{
					if (RTnum < MAXDITHERACTORS) RenderedTargets[RTnum++] = thing;
					else break;
				}
			}
		}
	}

	// lodonly sectors: the world LOD model draws their planes (hw_worldlod.cpp).
	if (gl_render_flats && !horizonHidden && !(lodOnly && HWSectorLodOnly(sub->render_sector)))
	{
		// Subsectors with only 2 lines cannot have any area
		if (sub->numlines>2 || (sub->hacked&1))
		{
			// Exclude the case when it tries to render a sector with a heightsec
			// but undetermined heightsec state. This can only happen if the
			// subsector is obstructed but not excluded due to a large bounding box.
			// Due to the way a BSP works such a subsector can never be visible
			if (!sector->GetHeightSec() || in_area!=area_default)
			{
				if (sector != sub->render_sector)
				{
					sector = sub->render_sector;
					// the planes of this subsector are faked to belong to another sector
					// This means we need the heightsec parts and light info of the render sector, not the actual one.
					fakesector = hw_FakeFlat(sector, in_area, false);
				}

				uint8_t &srf = section_renderflags[Level->sections.SectionIndex(sub->section)];
				if (!(srf & SSRF_PROCESSED))
				{
					srf |= SSRF_PROCESSED;

					if (multithread)
					{
						jobQueue.AddJob(RenderJob::FlatJob, sub);
					}
					else
					{
						HWFlat flat;
						flat.section = sub->section;
						SetupFlat.Clock();
						flat.ProcessSector(this, fakesector);
						SetupFlat.Unclock();
					}
				}
				// mark subsector as processed - but mark for rendering only if it has an actual area.
				ss_renderflags[sub->Index()] =
					(sub->numlines > 2) ? SSRF_PROCESSED|SSRF_RENDERALL : SSRF_PROCESSED;
				if (sub->hacked & 1) AddHackedSubsector(sub);

				// This is for portal coverage.
				FSectorPortalGroup *portal;

				// AddSubsectorToPortal cannot be called here when using multithreaded processing,
				// because the wall processing code in the worker can also modify the portal state.
				// To avoid costly synchronization for every access to the portal list,
				// the call to AddSubsectorToPortal will be deferred to the worker.
				// (GetPortalGruop only accesses static sector data so this check can be done here, restricting the new job to the minimum possible extent.)
				portal = fakesector->GetPortalGroup(sector_t::ceiling);
				if (portal != nullptr)
				{
					if (multithread)
					{
						jobQueue.AddJob(RenderJob::PortalJob, sub, (seg_t *)portal);
					}
					else
					{
						AddSubsectorToPortal(portal, sub);
					}
				}

				portal = fakesector->GetPortalGroup(sector_t::floor);
				if (portal != nullptr)
				{
					if (multithread)
					{
						jobQueue.AddJob(RenderJob::PortalJob, sub, (seg_t *)portal);
					}
					else
					{
						AddSubsectorToPortal(portal, sub);
					}
				}
			}
		}
	}
}




//==========================================================================
//
// RenderBSPNode
// Renders all subsectors below a given node,
//  traversing subtree recursively.
// Just call with BSP root.
//
//==========================================================================

void HWDrawInfo::RenderBSPNode (void *node)
{
	if (Level->nodes.Size() == 0)
	{
		DoSubsector (&Level->subsectors[0]);
		return;
	}
	// A subtree hidden behind the horizon is walked for its actors only; restore the mode on the way out.
	struct ThingsOnlyScope { bool &flag; bool saved; ~ThingsOnlyScope() { flag = saved; } } thingsOnlyScope{ thingsOnly, thingsOnly };

	while (!((size_t)node & 1))  // Keep going until found a subsector
	{
		node_t *bsp = (node_t *)node;

		// Decide which side the view point is on.
		int side = R_PointOnSide(viewx, viewy, bsp);
		// Nodes seen through a portal must be walked to unclip them, so the frustum test leaves them alone.
		const bool cull3D = frustum3D && !(no_renderflags[bsp->Index()] & SSRF_SEEN);

		// Recursively divide front space (toward the viewer).
		// Entirely lodonly: drawn by the world LOD model, nothing for the walk to do (hw_worldlod.cpp).
		if (LodOnlyChild(bsp->children[side])) {}
		else if (!cull3D || ChildInFrustum(bsp->children[side], bsp->bbox[side]))
		{
			const bool wasThingsOnly = thingsOnly;
			if (horizon && !thingsOnly && HorizonChildHidden(bsp->children[side], bsp->bbox[side])) thingsOnly = true;
			RenderBSPNode (bsp->children[side]);
			thingsOnly = wasThingsOnly;
		}
		else frustumStats.nodes++;

		// Possibly divide back space (away from the viewer).
		side ^= 1;

		// It is not necessary to use the slower precise version here
		if (!mClipper->CheckBox(bsp->bbox[side]))
		{
			if (!(no_renderflags[bsp->Index()] & SSRF_SEEN))
				return;
		}
		if (Viewpoint.bDoOrtho)
		{
			if (!vClipper->CheckBoxOrthoPitch(bsp->bbox[side]))
			{
				if (!(no_renderflags[bsp->Index()] & SSRF_SEEN))
					return;
			}
		}
		if (cull3D && !ChildInFrustum(bsp->children[side], bsp->bbox[side]))
		{
			frustumStats.nodes++;
			return;
		}
		if (LodOnlyChild(bsp->children[side])) return;
		if (horizon && !thingsOnly && HorizonChildHidden(bsp->children[side], bsp->bbox[side])) thingsOnly = true;

		node = bsp->children[side];
	}
	DoSubsector((subsector_t *)((uint8_t *)node - 1));
}

// No need for clipping inside frustum if no fog of war (How is this faster!)
void HWDrawInfo::RenderOrthoNoFog()
{
	if (Viewpoint.bDoOrtho && ((Level->flags3 & LEVEL3_NOFOGOFWAR) || !r_radarclipper))
	{
		double vxdbl = Viewpoint.OffPos.X;
		double vydbl = Viewpoint.OffPos.Y;
		double ext = Viewpoint.camera->ViewPos->Offset.Length() ?
			3.0 * Viewpoint.camera->ViewPos->Offset.Length() * tan (Viewpoint.FieldOfView.Radians()*0.5) : 100.0;
		FBoundingBox viewbox(vxdbl, vydbl, ext);

		for (unsigned int kk = 0; kk < Level->subsectors.Size(); kk++)
		{
			if (Level->subsectors[kk].bbox.CheckOverlap(viewbox))
			{
				DoSubsector (&Level->subsectors[kk]);
			}
		}
	}
}

void HWDrawInfo::RenderBSP(void *node, bool drawpsprites)
{
	ClearDitherTargets();
	Bsp.Clock();
	// Give the DrawInfo the viewpoint in fixed point because that's what the nodes are.
	viewx = FLOAT2FIXED(Viewpoint.Pos.X);
	viewy = FLOAT2FIXED(Viewpoint.Pos.Y);

	if (Viewpoint.sector)
	{
		Viewpoint.culledSectors = Viewpoint.sector->CulledSectors();
	}
	else
	{
		Viewpoint.culledSectors.clear();
	}
	

	if (r_radarclipper && !(Level->flags3 & LEVEL3_NOFOGOFWAR) && Viewpoint.bDoOob)
	{
		viewx = FLOAT2FIXED(Viewpoint.OffPos.X);
		viewy = FLOAT2FIXED(Viewpoint.OffPos.Y);
	}

	validcount++;	// used for processing sidedefs only once by the renderer.
	SetupTerrainBatches();
	SetupWorldLOD();

	// Height ranges for the 3D frustum test and the horizon: need this frame's ranges and a perspective view.
	zBounds = nullptr;
	frustum3D = false;
	FBspZBounds *zb = Level->BspZBounds;
	if (Frustum.valid && !Viewpoint.bDoOob && zb && zb->updated && zb->Matches(Level) && Level->nodes.Size() > 0)
	{
		zBounds = zb;
		bspZPad = (float)clamp<double>(gl_frustum_bsp_zpad, 0., 4096.);
		// Looking roughly level, the yaw clipper already rejects what's behind and the 3D test finds nothing
		// more, so it only runs once the view is pitched far enough up or down.
		frustum3D = gl_frustum_cull_bsp && fabs(Viewpoint.HWAngles.Pitch.Degrees()) >= gl_frustum_bsp_pitch;
	}

	// The horizon needs the height ranges, the main view (no mirrors or portals), the sky drawn behind
	// everything, and an eye that is really inside its sector's open space.
	horizon = nullptr;
	thingsOnly = clipOnly = false;
	horizonHiddenCount = 0;
	if (gl_horizon_cull && zBounds && outer == nullptr && mCurrentPortal == nullptr && !gl_no_skyclear && Viewpoint.sector && HorizonWorthRunning())
	{
		sector_t *vs = Viewpoint.sector;
		const DVector3 &eye = Viewpoint.Pos;
		const bool aboveFloor = !PlaneSolid(vs, sector_t::floor) || eye.Z >= vs->floorplane.ZatPoint(eye.XY());
		const bool belowCeiling = !PlaneSolid(vs, sector_t::ceiling) || eye.Z <= vs->ceilingplane.ZatPoint(eye.XY());
		if (aboveFloor && belowCeiling)
		{
			horizon = &mainHorizon;
			horizon->Clear(eye.X, eye.Y, eye.Z);
		}
	}

	multithread = gl_multithread;
	if (multithread)
	{
		jobQueue.ReleaseAll();
		auto future = renderPool.push([&](int id) {
			WorkerThread();
		});
		if (Viewpoint.bDoOrtho && ((Level->flags3 & LEVEL3_NOFOGOFWAR) || !r_radarclipper)) RenderOrthoNoFog();
		else RenderBSPNode(node);

		jobQueue.AddJob(RenderJob::TerminateJob, nullptr, nullptr);
		Bsp.Unclock();
		MTWait.Clock();
		future.wait();
		MTWait.Unclock();
	}
	else
	{
		if (Viewpoint.bDoOrtho && ((Level->flags3 & LEVEL3_NOFOGOFWAR) || !r_radarclipper)) RenderOrthoNoFog();
		else RenderBSPNode(node);
		Bsp.Unclock();
	}

	if (horizon) HorizonFeedback(horizonHiddenCount);

	// Make rendered targets set dither transparency flags on level geometry for next pass
	// Can't do this inside DoSubsector() because both Trace() and P_CheckSight() affect 'validcount' global variable
	for (int ii = 0; ii < MAXDITHERACTORS; ii++)
	{
		if ( RenderedTargets[ii] && P_CheckSight(players[consoleplayer].mo, RenderedTargets[ii], 0) )
		{
			SetDitherTransFlags(RenderedTargets[ii]);
		}
	}

	// Process all the sprites on the current portal's back side which touch the portal.
	if (mCurrentPortal != nullptr) mCurrentPortal->RenderAttached(this);

	if (drawpsprites)
		PreparePlayerSprites(Viewpoint.sector, in_area);
}
