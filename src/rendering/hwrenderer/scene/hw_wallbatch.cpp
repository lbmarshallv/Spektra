/*
** hw_wallbatch.cpp
**
** Batched drawing of outdoor walls. Every wall part that faces an outdoor
** sector (sky ceiling) and needs nothing special (no translucency, no 3D
** floor light splits, no glow, no gradient, no portal) is built once with
** the normal wall code (HWWall::Process in mesh mode) and copied into a static
** model-format vertex buffer, grouped by map chunk and by material (texture,
** clamping, light, colours). A visible chunk then costs one draw per material
** instead of one per wall, and distant walls stay visible.
**
** The BSP walk still runs for everything: lines keep feeding the clipper, but
** a line whose walls are all in a batch skips the wall setup entirely, and
** a line with only some of its parts batched skips those parts.
**
** Each frame the sides and sectors the batched walls were built from are
** compared with what was baked. A side that changed (texture, offsets, a
** moving floor or ceiling next to it, a dithered-transparency hit) is taken
** out of its batch for good: its vertices are collapsed, the buffer is
** uploaded again, and the side is drawn the normal way from then on.
** Light and colour (sector light level, colormap and fog, wall colours) are
** not baked: each batch takes them from one of its walls every frame, the
** same way the wall code computes them. So a day/night cycle, lightning or
** a fade that changes all its sectors alike keeps the batches. A batch whose
** sectors' light or colour drift apart is drawn the normal way until they
** agree again.
**
** Batched walls are drawn with back-face culling, which is what the BSP's
** facing test does for the normal walls.
**
**---------------------------------------------------------------------------
**
** Copyright 2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
*/

#include <float.h>
#include <stddef.h>
#include <algorithm>
#include "g_levellocals.h"
#include "r_defs.h"
#include "r_sky.h"
#include "p_lnspec.h"
#include "a_dynlight.h"
#include "texturemanager.h"
#include "v_video.h"
#include "c_cvars.h"
#include "hw_material.h"
#include "hw_renderstate.h"
#include "hw_lightbuffer.h"
#include "hw_dynlightdata.h"
#include "hw_modelvertexbuffer.h"
#include "hw_cvars.h"
#include "hw_lighting.h"
#include "hwrenderer/scene/hw_portal.h"
#include "hwrenderer/scene/hw_drawinfo.h"
#include "hwrenderer/scene/hw_drawstructs.h"
#include "hwrenderer/scene/hw_walldispatcher.h"
#include "flatvertices.h"
#include "hw_clock.h"
#include "stats.h"
#include "printf.h"

CVAR(Bool, gl_wall_batch, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Draw batched walls with back-face culling (only their front side is ever visible, as with normal walls).
CVAR(Bool, gl_wall_batch_cull, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Print why walls are taken out of their batches (the first few of each kind per level).
CVAR(Bool, gl_wall_batch_debug, false, 0)
EXTERN_CVAR(Int, gl_terrain_chunksize)
EXTERN_CVAR(Bool, gl_seamless)
EXTERN_CVAR(Int, r_distance_cull_type)
EXTERN_CVAR(Bool, gl_render_walls)

FWallBatchStats wallBatchStats;

void ResetWallBatchStats()
{
	wallBatchStats = {};
}

ADD_STAT(wallbatch)
{
	FString out;
	out.Format("Walls: %d chunks in %d draws, %d lines skipped, %d sides batched, %d taken out (%d sector, %d side, %d dither)",
		wallBatchStats.chunks, wallBatchStats.draws, wallBatchStats.lines, wallBatchStats.sides, wallBatchStats.evicted,
		wallBatchStats.evictedSector, wallBatchStats.evictedSide, wallBatchStats.evictedDither);
	return out;
}

//==========================================================================
//
// Data
//
//==========================================================================

struct FWallMaterial
{
	FGameTexture *texture;
	int clamp;
	int lightlevel, rellight;
	FColormap colormap;
	bool ext;				// side has its own tier colours (WALLF_EXTCOLOR)
	PalEntry color, addColor;
	TextureManipulation fx;

	bool operator==(const FWallMaterial &o) const { return !memcmp((const void *)this, (const void *)&o, sizeof(*this)); }
};

struct FWallGroup
{
	int material;
	int chunk;
	unsigned firstVertex, numVertices;
	unsigned firstLight, numLights;	// in FWallBatches::groupLights: the sectors the walls face (their light and colours)
	unsigned liveParts;				// parts not taken out; a group with none isn't drawn
	int repSide, repType;			// the wall whose light and colours the group is drawn with
	bool valid;						// the sectors' light and colours still agree with each other (this frame)
};

// Light and colours of a sector as they reach its walls.
struct FWallLight
{
	int lightlevel;
	FColormap colormap;
	PalEntry top, bottom, add;

	void From(const sector_t *sec)
	{
		memset((void *)this, 0, sizeof(*this));
		lightlevel = sec->lightlevel;
		memcpy((void *)&colormap, &sec->Colormap, sizeof(colormap));
		top = sec->SpecialColors[sector_t::walltop];
		bottom = sec->SpecialColors[sector_t::wallbottom];
		add = sec->AdditiveColors[sector_t::walltop];
	}
	bool SameColors(const FWallLight &o) const
	{
		return !memcmp((const void *)&colormap, (const void *)&o.colormap, sizeof(colormap)) && top == o.top && bottom == o.bottom && add == o.add;
	}
};

struct FWallLightDep
{
	int sector;
	FWallLight baked;
};

struct FWallChunk
{
	float x1, x2, y1, y2, z1, z2;
	TArray<int> groups;
	TArray<side_t *> sides;		// for finding the dynamic lights touching the chunk
	int lightIndex = -1;
	int lightFrame = -1;
	int portalGroup = 0;
};

// One batched wall part: where its vertices are, for taking it out again.
struct FWallPart
{
	int side;
	int group;
	int type;
	unsigned firstVertex, numVertices;
};

// Everything about a side that its walls depend on, as it was when baked.
struct FSideSignature
{
	uint8_t textures[sizeof(side_t::part) * 3];
	int16_t light;
	int16_t tierLights[3];
	uint16_t flags;
	double alpha;
	uint32_t lineFlags;
	int lineSpecial;
	double lineAlpha;
	sector_t *sector;
	LightmapSurface *lightmap;

	void From(const side_t *side)
	{
		memset((void *)this, 0, sizeof(*this));
		memcpy(textures, (const void *)side->textures, sizeof(textures));
		light = side->Light;
		memcpy(tierLights, side->TierLights, sizeof(tierLights));
		flags = side->Flags & ~(WALLF_DITHERTRANS_TOP | WALLF_DITHERTRANS_MID | WALLF_DITHERTRANS_BOTTOM);
		alpha = side->alpha;
		lineFlags = side->linedef->flags & ~ML_MAPPED;
		lineSpecial = side->linedef->special;
		lineAlpha = side->linedef->alpha;
		sector = side->sector;
		lightmap = side->lightmap;
	}
	bool operator!=(const FSideSignature &o) const { return memcmp((const void *)this, (const void *)&o, sizeof(*this)) != 0; }
};

// Everything about a sector that the walls next to it depend on.
struct FSectorSignature
{
	DVector3 floorN, ceilN;
	double floorD, ceilD;
	FTextureID texture[2];		// of the planes: decides sky walls and missing-texture fills
	double texZ[2];				// wall texture alignment
	PalEntry glowColor[2];
	float glowHeight[2];
	sector_t *heightsec;
	unsigned ffloors, lightlist;
	uint32_t moreFlags;

	void From(const sector_t *sec)
	{
		memset((void *)this, 0, sizeof(*this));
		floorN = sec->floorplane.Normal();
		ceilN = sec->ceilingplane.Normal();
		floorD = sec->floorplane.fD();
		ceilD = sec->ceilingplane.fD();
		// Light, colours and fog are not here: batches take them from the sector every frame.
		// Flat placement (offsets, scale, rotation, scrolling) doesn't affect walls.
		for (int p = 0; p < 2; p++)
		{
			texture[p] = sec->planes[p].Texture;
			texZ[p] = sec->planes[p].TexZ;
			glowColor[p] = sec->planes[p].GlowColor;
			glowHeight[p] = sec->planes[p].GlowHeight;
		}
		heightsec = sec->GetHeightSec();
		ffloors = sec->e->XFloor.ffloors.Size();
		lightlist = sec->e->XFloor.lightlist.Size();
		moreFlags = sec->MoreFlags & ~SECMF_DRAWN;	// set by the renderer the first time the sector is drawn
	}
	bool operator!=(const FSectorSignature &o) const { return memcmp((const void *)this, (const void *)&o, sizeof(*this)) != 0; }
};

struct FWallBatches
{
	const sector_t *sectorsData = nullptr;
	const side_t *sidesData = nullptr;
	unsigned numSectors = 0, numSides = 0, numSegs = 0;
	int chunkSize = 0;
	bool seamless = false;
	bool built = false;
	bool dirty = false;			// vertices changed, buffer needs uploading again
	int frame = 0, checkedFrame = -1;
	int batchedSides = 0, evictedSides = 0;
	int evictReasons[3] = {};
	int debugPrints = 0;		// EVICT_SECTOR, EVICT_SIDE, EVICT_DITHER
	bool lightFallback = false;		// some batches were invalid last frame: side masks need recomputing

	TArray<FWallMaterial> materials;
	TArray<FWallChunk> chunks;
	TArray<FWallGroup> groups;
	TArray<FModelVertex> vertices;	// CPU copy, for taking walls out
	TArray<FWallPart> parts;		// sorted by side
	TArray<int> sidePartStart;		// side i owns parts[sidePartStart[i] .. sidePartStart[i+1]-1]
	TArray<FWallLightDep> groupLights;
	TArray<uint8_t> sideMask;		// per side: (1 << RENDERWALL_xxx) for every part drawn by a batch this frame
	TArray<uint8_t> sideFull;		// per side: the batch draws everything this side has this frame, so the walk can skip it
	TArray<uint8_t> sideAll;		// per side: when baked, everything the side draws went into batches
	TArray<uint8_t> sideEvicted;	// per side: taken out for good
	TArray<int> batchedSideList;	// sides with batched parts that haven't been taken out
	TArray<FSideSignature> sideSigs;	// parallel to batchedSideList

	TArray<int> refSectors;			// sectors the batched walls depend on
	TArray<FSectorSignature> sectorSigs;
	TArray<uint8_t> sectorDone;		// every side depending on it has been taken out
	TArray<int> refSideStart;		// sector refSectors[i] affects sides refSides[refSideStart[i] .. refSideStart[i+1]-1]
	TArray<int> refSides;

	FModelVertexBuffer *vb = nullptr;

	~FWallBatches() { delete vb; }

	bool Matches(FLevelLocals *Level) const
	{
		return sectorsData == Level->sectors.Data() && sidesData == Level->sides.Data() && numSectors == Level->sectors.Size()
			&& numSides == Level->sides.Size() && numSegs == Level->segs.Size()
			&& chunkSize == clamp<int>(gl_terrain_chunksize, 128, 16384) && seamless == !!gl_seamless;
	}

	void Build(FLevelLocals *Level);
	void Check(FLevelLocals *Level);
	enum { EVICT_SECTOR, EVICT_SIDE, EVICT_DITHER };
	void Evict(int side, int reason);
	void Upload();
};

void DeleteWallBatches(FWallBatches *batches)
{
	delete batches;
}

//==========================================================================
//
// Which sides and wall parts can be batched
//
//==========================================================================

static bool SectorPlain(const sector_t *sec)
{
	return sec->GetHeightSec() == nullptr && sec->e->XFloor.ffloors.Size() == 0 && sec->e->XFloor.lightlist.Size() == 0
		&& sec->e->XFloor.attached.Size() == 0 && sec->special != GLSector_Skybox;
}

static bool SideBatchable(side_t *side)
{
	if (side->Flags & WALLF_POLYOBJ) return false;
	if (side->numsegs <= 0 || side->segs == nullptr) return false;
	line_t *line = side->linedef;
	if (line == nullptr) return false;
	if (line->special == Line_Horizon || line->special == Line_Mirror) return false;
	if (line->isVisualPortal() || line->GetTransferredPortal()) return false;
	sector_t *front = side->sector;
	if (front == nullptr || !SectorPlain(front)) return false;
	if (front->GetTexture(sector_t::ceiling) != skyflatnum) return false;	// outdoors only: indoors the BSP's occlusion matters
	side_t *other = line->sidedef[line->sidedef[0] == side ? 1 : 0];
	if (other && other->sector && !SectorPlain(other->sector)) return false;
	seg_t *seg = side->segs[0];
	if (seg == nullptr || seg->sidedef != side || seg->Subsector == nullptr || seg->frontsector == nullptr) return false;
	return true;
}

static bool WallBatchable(const HWWall &w, side_t *side)
{
	if (w.type != RENDERWALL_TOP && w.type != RENDERWALL_BOTTOM && w.type != RENDERWALL_M1S) return false;
	if (w.flags & (HWWall::HWF_TRANSLUCENT | HWWall::HWF_GLOW)) return false;
	if (w.lightlist != nullptr) return false;
	if (w.texture == nullptr || !w.texture->isValid()) return false;
	if (w.RenderStyle != STYLE_Normal || w.alpha != 1.f) return false;
	if (side->Flags & WALLF_EXTCOLOR)
	{
		const int tier = w.type == RENDERWALL_TOP ? side_t::top : w.type == RENDERWALL_BOTTOM ? side_t::bottom : side_t::mid;
		if (side->GetSpecialColor(tier, side_t::walltop, w.frontsector) != side->GetSpecialColor(tier, side_t::wallbottom, w.frontsector)) return false;	// gradient
	}
	return true;
}

static void MaterialFromWall(FWallMaterial &m, const HWWall &w, side_t *side)
{
	memset((void *)&m, 0, sizeof(m));
	m.texture = w.texture;
	m.clamp = w.flags & 3;
	m.lightlevel = w.lightlevel;
	m.rellight = w.rellight;
	memcpy((void *)&m.colormap, &w.Colormap, sizeof(m.colormap));
	if (side->Flags & WALLF_EXTCOLOR)
	{
		const int tier = w.type == RENDERWALL_TOP ? side_t::top : w.type == RENDERWALL_BOTTOM ? side_t::bottom : side_t::mid;
		m.ext = true;
		m.color = side->GetSpecialColor(tier, side_t::walltop, w.frontsector);
		m.addColor = side->GetAdditiveColor(tier, w.frontsector);
		memcpy((void *)&m.fx, &side->textures[tier].TextureFx, sizeof(m.fx));
	}
}

//==========================================================================
//
// Build: runs the normal wall setup for every eligible side and keeps the
// vertices of the parts that can be batched.
//
//==========================================================================

void FWallBatches::Build(FLevelLocals *Level)
{
	delete vb;
	vb = nullptr;
	materials.Clear();
	chunks.Clear();
	groups.Clear();
	vertices.Clear();
	parts.Clear();
	groupLights.Clear();
	batchedSideList.Clear();
	sideSigs.Clear();
	refSectors.Clear();
	sectorSigs.Clear();
	sectorDone.Clear();
	refSideStart.Clear();
	refSides.Clear();
	sectorsData = Level->sectors.Data();
	sidesData = Level->sides.Data();
	numSectors = Level->sectors.Size();
	numSides = Level->sides.Size();
	numSegs = Level->segs.Size();
	chunkSize = clamp<int>(gl_terrain_chunksize, 128, 16384);
	seamless = gl_seamless;
	built = true;
	dirty = false;
	batchedSides = evictedSides = 0;
	evictReasons[0] = evictReasons[1] = evictReasons[2] = 0;
	debugPrints = 0;
	lightFallback = false;
	checkedFrame = -1;
	sideMask.Resize(numSides);
	sideFull.Resize(numSides);
	sideAll.Resize(numSides);
	sideEvicted.Resize(numSides);
	for (unsigned i = 0; i < numSides; i++) sideMask[i] = sideFull[i] = sideAll[i] = sideEvicted[i] = 0;

	// Lines at each vertex: with seamless walls a wall's edges get extra vertices where neighbouring sectors' planes meet.
	TArray<int> vertLineStart, vertLines;
	if (seamless)
	{
		const unsigned numVerts = Level->vertexes.Size();
		vertLineStart.Resize(numVerts + 1);
		for (auto &v : vertLineStart) v = 0;
		for (auto &line : Level->lines) { vertLineStart[line.v1->Index() + 1]++; vertLineStart[line.v2->Index() + 1]++; }
		for (unsigned i = 0; i < numVerts; i++) vertLineStart[i + 1] += vertLineStart[i];
		vertLines.Resize(vertLineStart[numVerts]);
		TArray<int> fill;
		fill.Resize(numVerts);
		for (unsigned i = 0; i < numVerts; i++) fill[i] = vertLineStart[i];
		for (auto &line : Level->lines) { vertLines[fill[line.v1->Index()]++] = line.Index(); vertLines[fill[line.v2->Index()]++] = line.Index(); }
	}

	struct Entry { int side, chunk, material, type; unsigned first, count; };	// first/count in temp
	TArray<Entry> entries;
	TArray<FModelVertex> temp;
	TArray<FFlatVertex> fan;
	TMap<uint64_t, int> chunkMap;
	TArray<int> sideDepStart, sideDeps;		// per batched side (index into batchedSideList): sectors it depends on
	TArray<int> sectorRef;
	sectorRef.Resize(numSectors);
	for (auto &r : sectorRef) r = -1;
	const ELightMode lightmode = getRealLightmode(Level, true);
	HWMeshHelper mh;

	for (unsigned si = 0; si < numSides; si++)
	{
		side_t *side = &Level->sides[si];
		if (!SideBatchable(side)) continue;

		seg_t *seg = side->segs[0];
		sector_t *front = seg->frontsector;
		sector_t *back = seg->backsector;
		if (back && back->sectornum == front->sectornum) back = front;

		mh.list.Clear();
		mh.translucent.Clear();
		mh.portals.Clear();
		mh.lower.Clear();
		mh.upper.Clear();
		HWWallDispatcher disp(Level, &mh, lightmode);
		HWWall wall;
		wall.sub = seg->Subsector;
		wall.Process(&disp, seg, front, back, false);

		bool others = mh.translucent.Size() || mh.portals.Size() || mh.lower.Size() || mh.upper.Size();
		uint8_t mask = 0;
		const unsigned firstEntry = entries.Size();
		for (auto &w : mh.list)
		{
			if (!WallBatchable(w, side) || (mask & (1 << w.type)))
			{
				others = true;
				continue;
			}
			// The same vertices the normal path makes (HWWall::MakeVertices), as a triangle list.
			const bool split = gl_seamless && !(w.flags & HWWall::HWF_NOSPLIT);
			fan.Resize(split ? w.CountVertices() : 4);
			FFlatVertex *ptr = fan.Data();
			const int count = w.CreateVertices(ptr, split);
			if (count < 3) continue;
			assert((unsigned)count <= fan.Size());
			mask |= 1 << w.type;

			const float mx = (w.glseg.x1 + w.glseg.x2) * 0.5f, my = (w.glseg.y1 + w.glseg.y2) * 0.5f;
			const int cx = (int)floor(mx / chunkSize), cy = (int)floor(my / chunkSize);
			const uint64_t ckey = (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy);
			int chunk;
			if (int *found = chunkMap.CheckKey(ckey)) chunk = *found;
			else
			{
				chunk = chunks.Reserve(1);
				auto &c = chunks[chunk];
				c.x1 = c.y1 = c.z1 = FLT_MAX;
				c.x2 = c.y2 = c.z2 = -FLT_MAX;
				c.portalGroup = front->PortalGroup;
				chunkMap[ckey] = chunk;
			}

			FWallMaterial m;
			MaterialFromWall(m, w, side);
			int material = -1;
			for (unsigned j = 0; j < materials.Size(); j++) if (materials[j] == m) { material = j; break; }
			if (material < 0) material = materials.Push(m);

			const FVector3 n = w.glseg.Normal();
			auto &c = chunks[chunk];
			const unsigned first = temp.Size();
			for (int k = 1; k + 1 < count; k++)
			{
				const int idx[3] = { 0, k, k + 1 };
				for (int i : idx)
				{
					const FFlatVertex &fv = fan[i];
					FModelVertex mv;
					mv.Set(fv.x, fv.z, fv.y, fv.u, fv.v);	// renderer convention: (map X, height, map Y)
					mv.SetNormal(n.X, n.Y, n.Z);
					mv.lu = fv.lu;
					mv.lv = fv.lv;
					mv.lindex = fv.lindex;
					mv.SetBoneSelector(0, 0, 0, 0);
					mv.SetBoneWeight(0, 0, 0, 0);
					temp.Push(mv);
					c.x1 = min(c.x1, fv.x); c.x2 = max(c.x2, fv.x);
					c.y1 = min(c.y1, fv.y); c.y2 = max(c.y2, fv.y);
					c.z1 = min(c.z1, fv.z); c.z2 = max(c.z2, fv.z);
				}
			}
			entries.Push({ (int)si, chunk, material, (int)w.type, first, temp.Size() - first });
		}
		if (mask == 0) continue;

		sideMask[si] = mask;
		sideAll[si] = sideFull[si] = !others;
		batchedSideList.Push(si);
		FSideSignature sig;
		sig.From(side);
		sideSigs.Push(sig);
		for (unsigned e = firstEntry; e < entries.Size(); e++)
		{
			auto &c = chunks[entries[e].chunk];
			if (c.sides.Size() == 0 || c.sides.Last() != side) c.sides.Push(side);
		}

		// Sectors this side's walls depend on: its own, the one behind it, and with seamless walls every sector at its two vertices.
		sideDepStart.Push(sideDeps.Size());
		const unsigned depFirst = sideDeps.Size();
		auto addDep = [&](sector_t *sec)
		{
			if (sec == nullptr) return;
			int &ref = sectorRef[sec->Index()];
			if (ref < 0)
			{
				ref = refSectors.Push(sec->Index());
				FSectorSignature ss;
				ss.From(sec);
				sectorSigs.Push(ss);
			}
			for (unsigned d = depFirst; d < sideDeps.Size(); d++) if (sideDeps[d] == ref) return;
			sideDeps.Push(ref);
		};
		addDep(front);
		addDep(seg->backsector);
		if (seamless)
		{
			for (vertex_t *v : { side->linedef->v1, side->linedef->v2 })
			{
				for (int l = vertLineStart[v->Index()]; l < vertLineStart[v->Index() + 1]; l++)
				{
					line_t *line = &Level->lines[vertLines[l]];
					addDep(line->frontsector);
					addDep(line->backsector);
				}
			}
		}
	}
	sideDepStart.Push(sideDeps.Size());
	batchedSides = batchedSideList.Size();
	if (entries.Size() == 0) { built = false; return; }

	// Sector -> sides that depend on it (as side numbers).
	sectorDone.Resize(refSectors.Size());
	for (auto &d : sectorDone) d = 0;
	refSideStart.Resize(refSectors.Size() + 1);
	for (auto &r : refSideStart) r = 0;
	for (auto ref : sideDeps) refSideStart[ref + 1]++;
	for (unsigned i = 0; i < refSectors.Size(); i++) refSideStart[i + 1] += refSideStart[i];
	refSides.Resize(sideDeps.Size());
	{
		TArray<int> fill;
		fill.Resize(refSectors.Size());
		for (unsigned i = 0; i < refSectors.Size(); i++) fill[i] = refSideStart[i];
		for (unsigned b = 0; b < batchedSideList.Size(); b++)
		{
			for (int d = sideDepStart[b]; d < sideDepStart[b + 1]; d++) refSides[fill[sideDeps[d]]++] = batchedSideList[b];
		}
	}

	// Every (chunk, material) group contiguous.
	std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b)
	{
		return a.chunk != b.chunk ? a.chunk < b.chunk : a.material != b.material ? a.material < b.material : a.side < b.side;
	});

	vertices.Resize(temp.Size());
	unsigned v = 0;
	int curGroup = -1;
	for (auto &e : entries)
	{
		if (curGroup < 0 || groups[curGroup].chunk != e.chunk || groups[curGroup].material != e.material)
		{
			curGroup = groups.Reserve(1);
			auto &g = groups[curGroup];
			g.material = e.material;
			g.chunk = e.chunk;
			g.firstVertex = v;
			g.numVertices = 0;
			g.firstLight = groupLights.Size();
			g.numLights = 0;
			g.valid = true;
			g.liveParts = 0;
			g.repSide = e.side;
			g.repType = e.type;
			chunks[e.chunk].groups.Push(curGroup);
		}
		auto &g = groups[curGroup];
		memcpy(&vertices[v], &temp[e.first], e.count * sizeof(FModelVertex));
		parts.Push({ e.side, curGroup, e.type, v, e.count });
		g.liveParts++;
		g.numVertices += e.count;
		v += e.count;

		// Wall light comes from the sector the side faces.
		sector_t *sec = Level->sides[e.side].sector;
		bool known = false;
		for (unsigned i = g.firstLight; i < groupLights.Size(); i++) if (groupLights[i].sector == sec->Index()) { known = true; break; }
		if (!known)
		{
			FWallLightDep dep;
			dep.sector = sec->Index();
			dep.baked.From(sec);
			groupLights.Push(dep);
			g.numLights++;
		}
	}

	std::sort(parts.begin(), parts.end(), [](const FWallPart &a, const FWallPart &b) { return a.side < b.side; });
	sidePartStart.Resize(numSides + 1);
	for (auto &s : sidePartStart) s = 0;
	for (auto &p : parts) sidePartStart[p.side + 1]++;
	for (unsigned i = 0; i < numSides; i++) sidePartStart[i + 1] += sidePartStart[i];

	dirty = true;
	Upload();
}

void FWallBatches::Upload()
{
	delete vb;	// the backends delete GPU buffers only once the frames using them are done
	vb = new FModelVertexBuffer(false, true);
	FModelVertex *out = vb->LockVertexBuffer(vertices.Size());
	memcpy(out, vertices.Data(), vertices.Size() * sizeof(FModelVertex));
	vb->UnlockVertexBuffer();
	dirty = false;
}

//==========================================================================
//
// Taking a side out of its batches: its triangles collapse to nothing and
// the normal path draws it again.
//
//==========================================================================

void FWallBatches::Evict(int side, int reason)
{
	if (sideEvicted[side]) return;
	sideEvicted[side] = 1;
	evictReasons[reason]++;
	for (int p = sidePartStart[side]; p < sidePartStart[side + 1]; p++)
	{
		const FWallPart &part = parts[p];
		groups[part.group].liveParts--;
		const FModelVertex &first = vertices[part.firstVertex];
		const float x = first.x, y = first.y, z = first.z;
		for (unsigned k = 0; k < part.numVertices; k++)
		{
			auto &mv = vertices[part.firstVertex + k];
			mv.x = x; mv.y = y; mv.z = z;
		}
	}
	sideMask[side] = 0;
	sideFull[side] = 0;
	evictedSides++;
	dirty = true;
}

//==========================================================================
//
// gl_wall_batch_debug: which part of a sector or side changed
//
//==========================================================================

#define WB_FIELD(sig, field) { #field, offsetof(sig, field), sizeof(((sig *)nullptr)->field) }
struct FSigField { const char *name; size_t offset, size; };

static FString ChangedFields(const FSigField *fields, size_t count, const void *a, const void *b)
{
	FString out;
	for (size_t i = 0; i < count; i++)
	{
		if (memcmp((const uint8_t *)a + fields[i].offset, (const uint8_t *)b + fields[i].offset, fields[i].size))
		{
			if (out.Len()) out += ", ";
			out += fields[i].name;
		}
	}
	return out.Len() ? out : FString("padding");
}

static void DebugSectorChange(int sector, const FSectorSignature &baked, const FSectorSignature &now)
{
	static const FSigField fields[] = {
		WB_FIELD(FSectorSignature, floorN), WB_FIELD(FSectorSignature, ceilN), WB_FIELD(FSectorSignature, floorD), WB_FIELD(FSectorSignature, ceilD),
		WB_FIELD(FSectorSignature, texture), WB_FIELD(FSectorSignature, texZ), WB_FIELD(FSectorSignature, glowColor), WB_FIELD(FSectorSignature, glowHeight),
		WB_FIELD(FSectorSignature, heightsec), WB_FIELD(FSectorSignature, ffloors), WB_FIELD(FSectorSignature, lightlist), WB_FIELD(FSectorSignature, moreFlags),
	};
	FString what = ChangedFields(fields, countof(fields), &baked, &now);
	Printf("wallbatch: sector %d changed (%s), its walls are taken out\n", sector, what.GetChars());
}

static void DebugSideChange(int side, const FSideSignature &baked, const FSideSignature &now)
{
	static const FSigField fields[] = {
		WB_FIELD(FSideSignature, textures), WB_FIELD(FSideSignature, light), WB_FIELD(FSideSignature, tierLights), WB_FIELD(FSideSignature, flags),
		WB_FIELD(FSideSignature, alpha), WB_FIELD(FSideSignature, lineFlags), WB_FIELD(FSideSignature, lineSpecial), WB_FIELD(FSideSignature, lineAlpha),
		WB_FIELD(FSideSignature, sector), WB_FIELD(FSideSignature, lightmap),
	};
	FString what = ChangedFields(fields, countof(fields), &baked, &now);
	if (what.Compare("flags") == 0) what.AppendFormat(" %04x -> %04x", baked.flags, now.flags);
	if (what.Compare("lineFlags") == 0) what.AppendFormat(" %08x -> %08x", baked.lineFlags, now.lineFlags);
	Printf("wallbatch: side %d changed (%s), taken out\n", side, what.GetChars());
}

// Once per frame, after this frame's dithered-transparency flags are set (they're checked with the side's flags).
void FWallBatches::Check(FLevelLocals *Level)
{
	for (unsigned r = 0; r < refSectors.Size(); r++)
	{
		if (sectorDone[r]) continue;
		FSectorSignature ss;
		ss.From(&Level->sectors[refSectors[r]]);
		if (ss != sectorSigs[r])
		{
			if (gl_wall_batch_debug && debugPrints++ < 20) DebugSectorChange(refSectors[r], sectorSigs[r], ss);
			for (int i = refSideStart[r]; i < refSideStart[r + 1]; i++) Evict(refSides[i], EVICT_SECTOR);
			sectorDone[r] = 1;
		}
	}

	const uint16_t dither = WALLF_DITHERTRANS_TOP | WALLF_DITHERTRANS_MID | WALLF_DITHERTRANS_BOTTOM;
	for (unsigned b = 0; b < batchedSideList.Size(); )
	{
		const int si = batchedSideList[b];
		const side_t *side = &Level->sides[si];
		bool gone = sideEvicted[si];
		if (!gone)
		{
			FSideSignature sig;
			sig.From(side);
			if (sig != sideSigs[b])
			{
				if (gl_wall_batch_debug && debugPrints++ < 20) DebugSideChange(si, sideSigs[b], sig);
				Evict(si, EVICT_SIDE);
				gone = true;
			}
			else if (side->Flags & dither)
			{
				Evict(si, EVICT_DITHER);
				gone = true;
			}
		}
		if (gone)
		{
			// Not checked again.
			batchedSideList[b] = batchedSideList.Last();
			sideSigs[b] = sideSigs.Last();
			batchedSideList.Pop();
			sideSigs.Pop();
		}
		else b++;
	}

	if (dirty) Upload();

	// Light and colours: a batch is drawn with its representative wall's current light, which is right for
	// all its walls as long as their sectors changed alike (same light level change, same colours).
	// Otherwise it is drawn the normal way this frame.
	bool allValid = true;
	for (auto &g : groups)
	{
		g.valid = true;
		if (g.numLights <= 1 || g.liveParts == 0) continue;
		const FWallLightDep &r = groupLights[g.firstLight];
		FWallLight rnow;
		rnow.From(&Level->sectors[r.sector]);
		const int rdelta = rnow.lightlevel - r.baked.lightlevel;
		const bool rsame = rnow.SameColors(r.baked);
		for (unsigned i = 1; i < g.numLights; i++)
		{
			const FWallLightDep &d = groupLights[g.firstLight + i];
			FWallLight now;
			now.From(&Level->sectors[d.sector]);
			bool ok = now.lightlevel - d.baked.lightlevel == rdelta;
			if (ok)
			{
				if (d.baked.SameColors(r.baked)) ok = now.SameColors(rnow);
				else ok = rsame && now.SameColors(d.baked);
			}
			if (!ok) { g.valid = false; allValid = false; break; }
		}
	}
	if (!allValid || lightFallback)
	{
		for (int si : batchedSideList)
		{
			uint8_t mask = 0;
			bool all = true;
			for (int p = sidePartStart[si]; p < sidePartStart[si + 1]; p++)
			{
				if (groups[parts[p].group].valid) mask |= 1 << parts[p].type;
				else all = false;
			}
			sideMask[si] = mask;
			sideFull[si] = sideAll[si] && all;
		}
	}
	lightFallback = !allValid;
}

void UpdateWallBatches(FLevelLocals *Level)
{
	FWallBatches *wb = Level->WallBatches;
	if (!gl_wall_batch || Level->sides.Size() == 0) return;
	if (wb == nullptr) wb = Level->WallBatches = new FWallBatches;
	if (!wb->Matches(Level)) wb->Build(Level);
	wb->frame++;
}

// Called from the line processing: does a batch draw everything this side has?
bool HWDrawInfo::SideBatched(const side_t *side) const
{
	return wallBatches && wallBatches->sideFull[side->Index()] && side->AttachedDecals == nullptr;
}

// Called from HWWall::PutWall (also on the worker thread): is this part of the wall drawn by a batch?
bool HWDrawInfo::WallPartBatched(const seg_t *seg, int type) const
{
	return wallBatches && seg->sidedef && (wallBatches->sideMask[seg->sidedef->Index()] & (1 << type));
}

void HWDrawInfo::SetupWallBatches()
{
	wallBatches = nullptr;
	// Main views only: portals and mirrors render their walls the normal way.
	FWallBatches *wb = Level->WallBatches;
	if (gl_wall_batch && wb && wb->built && wb->Matches(Level) && outer == nullptr && mCurrentPortal == nullptr)
	{
		if (wb->checkedFrame != wb->frame)
		{
			wb->checkedFrame = wb->frame;
			wb->Check(Level);
		}
		wallBatches = wb;
		wallBatchStats.sides = wb->batchedSides - wb->evictedSides;
		wallBatchStats.evicted = wb->evictedSides;
		wallBatchStats.evictedSector = wb->evictReasons[FWallBatches::EVICT_SECTOR];
		wallBatchStats.evictedSide = wb->evictReasons[FWallBatches::EVICT_SIDE];
		wallBatchStats.evictedDither = wb->evictReasons[FWallBatches::EVICT_DITHER];
	}
}

//==========================================================================
//
// Draw the visible batches (opaque pass, after the plain walls and flats).
//
//==========================================================================

// Same as CalcRelLight in hw_walls.cpp.
static int WallRelLight(int lightlevel, int orglightlevel, int rel)
{
	if (orglightlevel >= 253) return 0;
	else if (lightlevel - rel > 256) return 256 - lightlevel + rel;
	else return rel;
}

static TArray<int> visibleWallGroups;
static FDynLightData wallLightData;
static TArray<FDynamicLight *> wallLights;

void HWDrawInfo::DrawWallBatches(FRenderState &state)
{
	FWallBatches *wb = wallBatches;
	if (wb == nullptr || wb->vb == nullptr || !gl_render_walls) return;

	const DVector3 &eye = Viewpoint.Pos;
	const double culldistsq = (r_distance_cull_type > 0) ? Viewpoint.culldistsq : 0.;
	const bool fullbright = isFullbrightScene();
	const bool lights = Level->HasDynamicLights && !fullbright;

	visibleWallGroups.Clear();
	if (lights) screen->mLights->Map();
	for (auto &c : wb->chunks)
	{
		if (!Frustum.MapBoxVisible(c.x1, c.x2, c.y1, c.y2, c.z1, c.z2)) continue;
		if (culldistsq > 0)
		{
			const double dx = eye.X < c.x1 ? c.x1 - eye.X : (eye.X > c.x2 ? eye.X - c.x2 : 0);
			const double dy = eye.Y < c.y1 ? c.y1 - eye.Y : (eye.Y > c.y2 ? eye.Y - c.y2 : 0);
			if (dx * dx + dy * dy >= culldistsq) continue;
		}
		bool any = false;
		for (int g : c.groups) if (wb->groups[g].valid && wb->groups[g].liveParts > 0) { visibleWallGroups.Push(g); any = true; }
		wallBatchStats.chunks += any;

		// Dynamic lights touching any wall of the chunk, gathered once per frame.
		if (any && lights && c.lightFrame != wb->frame)
		{
			c.lightFrame = wb->frame;
			c.lightIndex = -1;
			wallLights.Clear();
			wallLightData.Clear();
			for (side_t *side : c.sides)
			{
				auto list = Level->lightlists.wall_dlist.CheckKey(side);
				if (list == nullptr) continue;
				TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Iterator it(*list);
				TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Pair *pair;
				while (it.NextPair(pair))
				{
					auto node = pair->Value.get();
					if (node == nullptr) continue;
					FDynamicLight *light = node->lightsource;
					if (!light->IsActive() || light->DontLightMap()) continue;
					if (std::find(wallLights.begin(), wallLights.end(), light) != wallLights.end()) continue;
					wallLights.Push(light);
					AddLightToList(wallLightData, c.portalGroup, light, true);
				}
			}
			if (wallLights.Size() > 0) c.lightIndex = screen->mLights->UploadLights(wallLightData);
		}
	}
	if (lights) screen->mLights->Unmap();
	if (visibleWallGroups.Size() == 0) return;

	// Fewest state changes: same material together.
	std::sort(visibleWallGroups.begin(), visibleWallGroups.end(), [=](int a, int b)
	{
		return wb->groups[a].material != wb->groups[b].material ? wb->groups[a].material < wb->groups[b].material : a < b;
	});

	const int extra = getExtraLight();
	state.SetVertexBuffer(wb->vb->vertexBuffer(), 0, 0);
	state.SetInterpolationFactor(0.f);
	state.SetBoneIndexBase(-1);
	state.SetDynLight(0, 0, 0);
	state.EnableTextureMatrix(false);
	if (gl_wall_batch_cull) state.SetCulling(portalState.isMirrored() ? Cull_CCW : Cull_CW);

	int curMaterial = -1;
	for (int gi : visibleWallGroups)
	{
		const FWallGroup &g = wb->groups[gi];
		if (g.material != curMaterial)
		{
			curMaterial = g.material;
			const FWallMaterial &m = wb->materials[g.material];
			if (m.ext) state.ApplyTextureManipulation(&wb->materials[g.material].fx);
			else state.ApplyTextureManipulation(nullptr);
			state.SetMaterial(m.texture, UF_Texture, 0, m.clamp, NO_TRANSLATION, -1, nullptr);
		}

		// Light and colours as HWWall::Process computes them, from the group's representative wall.
		{
			side_t *side = &Level->sides[g.repSide];
			sector_t *sec = side->sector;
			const int tier = g.repType == RENDERWALL_TOP ? side_t::top : g.repType == RENDERWALL_BOTTOM ? side_t::bottom : side_t::mid;
			const int orglightlevel = hw_ClampLight(sec->lightlevel);
			const bool foggy = !sec->Colormap.FadeColor.isBlack() || (Level->flags & LEVEL_HASFADETABLE);
			int rel = 0;
			const int lightlevel = RescaleLightLevel(side->GetLightLevel(foggy, orglightlevel, tier, false, &rel));
			const int rellight = WallRelLight(lightlevel, orglightlevel, rel) + extra;
			FColormap cm = sec->Colormap;
			if (fullbright) cm.Clear();
			SetFog(state, Level, lightmode, lightlevel, rellight, fullbright, &cm, false);
			SetColor(state, Level, lightmode, lightlevel, rellight, fullbright, cm, 1.f);
			if (side->Flags & WALLF_EXTCOLOR)
			{
				state.SetObjectColor(side->GetSpecialColor(tier, side_t::walltop, sec));
				state.SetObjectColor2(0);
				state.SetAddColor(side->GetAdditiveColor(tier, sec));
			}
			else
			{
				state.SetObjectColor(0xffffffff);
				state.SetObjectColor2(0);
				state.SetAddColor(0);
			}
		}
		state.SetLightIndex(lights ? wb->chunks[g.chunk].lightIndex : -1);
		state.Draw(DT_Triangles, g.firstVertex, g.numVertices);
		wallBatchStats.draws++;
		vertexcount += g.numVertices;
	}

	if (gl_wall_batch_cull) state.SetCulling(Cull_None);
	state.SetObjectColor(0xffffffff);
	state.SetObjectColor2(0);
	state.SetAddColor(0);
	state.ApplyTextureManipulation(nullptr);
	state.SetLightIndex(-1);
	state.SetVertexBuffer(screen->mVertexData);
}
