/*
** hw_terrainbatch.cpp
**
** Batched drawing of outdoor floors (terrain meshes made of many small
** sectors). Every eligible floor is copied once into a static model-format
** vertex buffer, grouped by map chunk and by material (texture, texture
** placement, light, colours). A visible chunk then costs one draw per
** material instead of one per sector, and the vertices carry their own
** normals so lighting is unchanged.
**
** The BSP walk still runs for everything else. For batched floors it skips
** the flat, and it skips seamless lines between two batched floors
** without any per-line work.
**
** Each frame every batched floor is compared with what was baked. A floor
** that changed (it moved, or its texture or placement changed) is taken out
** of its batch for good: its triangles collapse, the buffer is uploaded again,
** and the BSP draws it the normal way from then on.
**
** Light and colour (light level, fog, flat colours) are not baked: each batch
** takes them from one of its floors every frame. So a time-of-day script that
** changes all outdoor sectors alike keeps the batches. A batch whose floors'
** light or colour drift apart falls back to normal rendering until they agree.
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
#include <algorithm>
#include <limits.h>
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
#include "hwrenderer/scene/hw_drawinfo.h"
#include "hwrenderer/scene/hw_drawstructs.h"
#include "flatvertices.h"
#include "hw_clock.h"
#include "stats.h"
#include "p_spec.h"
#include "a_scroll.h"

CVAR(Bool, gl_terrain_batch, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
CVAR(Int, gl_terrain_chunksize, 4096, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
EXTERN_CVAR(Int, r_distance_cull_type)

FTerrainStats terrainStats;

void ResetTerrainStats()
{
	terrainStats = {};
}

ADD_STAT(terrain)
{
	FString out;
	out.Format("Terrain: %d chunks in %d draws, %d lines skipped", terrainStats.chunks, terrainStats.draws, terrainStats.segs);
	return out;
}

bool hw_SetPlaneTextureRotation(const HWSectorPlane *secplane, FGameTexture *gltexture, VSMatrix &mat);
bool IsSeamlessLine(seg_t *seg);	// hw_bsp.cpp

//==========================================================================
//
// Data
//
//==========================================================================

struct FTerrainMaterial
{
	FTextureID texture;
	float offsX, offsY, scaleX, scaleY, angle;
	int lightlevel;
	FColormap colormap;
	PalEntry flatColor, addColor;
	TextureManipulation fx;

	void FromSector(sector_t *sec)
	{
		texture = sec->GetTexture(sector_t::floor);
		offsX = (float)sec->GetXOffset(sector_t::floor);
		offsY = (float)sec->GetYOffset(sector_t::floor);
		scaleX = (float)sec->GetXScale(sector_t::floor);
		scaleY = (float)sec->GetYScale(sector_t::floor);
		angle = (float)sec->GetAngle(sector_t::floor).Degrees();
		lightlevel = RescaleLightLevel(sec->GetFloorLight());
		colormap = sec->Colormap;
		flatColor = sec->SpecialColors[sector_t::floor];
		addColor = sec->AdditiveColors[sector_t::floor];
		fx = sec->planes[sector_t::floor].TextureFx;
	}

	bool operator==(const FTerrainMaterial &o) const
	{
		return texture == o.texture && offsX == o.offsX && offsY == o.offsY && scaleX == o.scaleX && scaleY == o.scaleY && angle == o.angle &&
			lightlevel == o.lightlevel && !memcmp(&colormap, &o.colormap, sizeof(colormap)) &&
			flatColor == o.flatColor && addColor == o.addColor && !memcmp(&fx, &o.fx, sizeof(fx));
	}
};

struct FTerrainGroup
{
	int material;
	int chunk;
	unsigned firstVertex, numVertices;
	unsigned firstSector, numSectors;	// in FTerrainBatches::groupSectors
	unsigned liveFloors;				// floors not taken out
	unsigned rep;						// the floor (of firstSector..) whose light the group is drawn with
	bool valid;							// its floors' light and colours agree (this frame)
};

struct FTerrainChunk
{
	uint64_t key;						// map cell (see HWCellKey)
	float x1, x2, y1, y2, z1, z2;
	TArray<int> groups;
	TArray<FSection *> sections;		// for finding the dynamic lights touching the chunk
	int lightIndex = -1;
	int lightFrame = -1;
	int portalGroup = 0;
};

// Light and colours of a floor, as they reach the flat.
struct FFloorLight
{
	int lightlevel;
	FColormap colormap;
	PalEntry flatColor, addColor;

	void FromSector(const sector_t *sec)
	{
		memset((void*)this, 0, sizeof(*this));
		lightlevel = sec->GetFloorLight();
		memcpy((void*)&colormap, &sec->Colormap, sizeof(colormap));
		flatColor = sec->SpecialColors[sector_t::floor];
		addColor = sec->AdditiveColors[sector_t::floor];
	}
	bool SameColors(const FFloorLight &o) const
	{
		return !memcmp((const void*)&colormap, (const void*)&o.colormap, sizeof(colormap)) && flatColor == o.flatColor && addColor == o.addColor;
	}
};

// Everything the floor's shape and texturing depends on, as it was when baked. Compared byte for byte every frame.
struct FBakedFloor
{
	int sector;
	DVector3 normal;
	double d;
	sector_t::splane plane;	// texture, placement, alpha, glow, texture effects, ... (light offset zeroed: light is not baked)
	FFloorLight light;		// as baked: the group's floors must change alike
	unsigned firstVertex, numVertices;
	bool evicted;			// changed since the build: taken out for good

	void FromSector(int index, sector_t *sec)
	{
		memset((void*)this, 0, sizeof(*this));
		sector = index;
		normal = sec->floorplane.Normal();
		d = sec->floorplane.fD();
		memcpy((void*)&plane, &sec->planes[sector_t::floor], sizeof(plane));
		plane.Light = 0;
		light.FromSector(sec);
	}
};

struct FTerrainBatches
{
	const sector_t *sectorsData = nullptr;
	unsigned numSectors = 0, numSegs = 0;
	int chunkSize = 0;
	bool built = false;

	TArray<FTerrainMaterial> materials;
	TArray<FTerrainChunk> chunks;
	TArray<FTerrainGroup> groups;
	TArray<FBakedFloor> groupSectors;
	TArray<int> sectorGroup;			// per sector: its group, -1 if not batched
	TArray<uint8_t> sectorBatched;		// per sector: drawn by a batch this frame
	TArray<uint8_t> segSeamless;		// per seg: seamless line between two batched floors (when both are batched)
	TArray<FModelVertex> vertices;		// CPU copy, for taking floors out
	bool dirty = false;
	FModelVertexBuffer *vb = nullptr;
	int frame = 0;
	int checkedTic = INT_MIN;			// game tic of the last Update (floors and light only change on tics)

	~FTerrainBatches() { delete vb; }

	void Upload()
	{
		delete vb;	// the backends delete GPU buffers only once the frames using them are done
		vb = new FModelVertexBuffer(false, true);
		FModelVertex *out = vb->LockVertexBuffer(vertices.Size());
		memcpy(out, vertices.Data(), vertices.Size() * sizeof(FModelVertex));
		vb->UnlockVertexBuffer();
		dirty = false;
	}

	void Evict(FTerrainGroup &g, FBakedFloor &b)
	{
		b.evicted = true;
		g.liveFloors--;
		const FModelVertex &first = vertices[b.firstVertex];
		const float x = first.x, y = first.y, z = first.z;
		for (unsigned k = 0; k < b.numVertices; k++)
		{
			auto &mv = vertices[b.firstVertex + k];
			mv.x = x; mv.y = y; mv.z = z;
		}
		dirty = true;
	}

	bool Matches(FLevelLocals *Level) const
	{
		return sectorsData == Level->sectors.Data() && numSectors == Level->sectors.Size() && numSegs == Level->segs.Size()
			&& chunkSize == clamp<int>(gl_terrain_chunksize, 128, 16384);
	}

	void Build(FLevelLocals *Level);
	void Update(FLevelLocals *Level);
};

void DeleteTerrainBatches(FTerrainBatches *batches)
{
	delete batches;
}

//==========================================================================
//
// Which floors can be batched: outdoor (sky ceiling), plain floors with
// nothing that needs per-sector rendering.
//
//==========================================================================

static bool FloorBatchable(sector_t *sec)
{
	if (sec->GetTexture(sector_t::ceiling) != skyflatnum) return false;	// outdoors only: indoors the BSP's occlusion matters
	if (sec->GetTexture(sector_t::floor) == skyflatnum) return false;
	if (sec->Portals[sector_t::floor] != 0 || sec->GetReflect(sector_t::floor) != 0) return false;
	if (sec->GetHeightSec() || sec->e->XFloor.ffloors.Size() || sec->e->XFloor.attached.Size()) return false;
	if (sec->special == GLSector_Skybox) return false;
	if (sec->floorplane.Normal().Z <= 0) return false;
	if (sec->floorplane.dithertransflag) return false;
	auto tex = TexMan.GetGameTexture(sec->GetTexture(sector_t::floor), true);
	if (!tex || !tex->isValid()) return false;
	return true;
}

// Is the floor still shaped and textured like it was baked? (Cheap: this runs for every batched floor every frame.)
static bool FloorUnchanged(sector_t *sec, const FBakedFloor &b)
{
	if (sec->floorplane.fD() != b.d || sec->floorplane.Normal() != b.normal) return false;
	sector_t::splane plane;
	memcpy((void*)&plane, &sec->planes[sector_t::floor], sizeof(plane));
	plane.Light = 0;
	if (memcmp((const void*)&plane, (const void*)&b.plane, sizeof(b.plane))) return false;
	if (sec->Portals[sector_t::floor] != 0 || sec->GetReflect(sector_t::floor) != 0 || sec->floorplane.dithertransflag) return false;
	if (sec->GetHeightSec() || sec->e->XFloor.ffloors.Size()) return false;
	return true;
}

//==========================================================================
//
// Build: copies the floors' triangles out of the flat vertex data.
//
//==========================================================================

// Floors moved by scroller thinkers: kept out of the batches so they animate (a scrolling floor would
// otherwise be taken out of its batch on the first tic).
static void ScrolledFloors(FLevelLocals *Level, TArray<uint8_t> &out)
{
	out.Resize(Level->sectors.Size());
	for (auto &b : out) b = 0;
	auto it = Level->GetThinkerIterator<DScroller>(NAME_None, STAT_SCROLLER);
	while (DScroller *s = it.Next())
	{
		if (s->IsType(EScroll::sc_floor) && s->GetSector()) out[s->GetSector()->Index()] = 1;
	}
}

void FTerrainBatches::Build(FLevelLocals *Level)
{
	delete vb;
	vb = nullptr;
	materials.Clear();
	chunks.Clear();
	groups.Clear();
	groupSectors.Clear();
	sectorsData = Level->sectors.Data();
	numSectors = Level->sectors.Size();
	numSegs = Level->segs.Size();
	chunkSize = clamp<int>(gl_terrain_chunksize, 128, 16384);
	built = true;
	sectorGroup.Resize(numSectors);
	sectorBatched.Resize(numSectors);
	for (unsigned i = 0; i < numSectors; i++) { sectorGroup[i] = -1; sectorBatched[i] = 0; }
	segSeamless.Resize(numSegs);
	for (auto &s : segSeamless) s = 0;

	auto fvb = screen->mVertexData;
	if (fvb == nullptr || fvb->ibo_data.Size() == 0) { built = false; return; }

	// Collect batchable floors with their chunk and material.
	struct Entry { int sector, chunk, material; };
	TArray<Entry> entries;
	TMap<uint64_t, int> chunkMap;
	TArray<uint8_t> scrolled;
	ScrolledFloors(Level, scrolled);
	for (unsigned i = 0; i < numSectors; i++)
	{
		sector_t *sec = &Level->sectors[i];
		if (!FloorBatchable(sec) || scrolled[i]) continue;
		auto sections = Level->sections.SectionsForSector(sec);
		if (sections.Size() == 0) continue;
		bool skip = false;
		for (auto &section : sections) if (section.flags & FSection::DONTRENDERFLOOR) skip = true;
		if (skip) continue;

		const uint64_t ckey = HWCellKey(sec->centerspot.X, sec->centerspot.Y, chunkSize);
		int chunk;
		if (int *found = chunkMap.CheckKey(ckey)) chunk = *found;
		else
		{
			chunk = chunks.Reserve(1);
			auto &c = chunks[chunk];
			c.x1 = c.y1 = c.z1 = FLT_MAX;
			c.x2 = c.y2 = c.z2 = -FLT_MAX;
			c.portalGroup = sec->PortalGroup;
			c.key = ckey;
			chunkMap[ckey] = chunk;
		}

		FTerrainMaterial m;
		m.FromSector(sec);
		int material = -1;
		for (unsigned j = 0; j < materials.Size(); j++) if (materials[j] == m) { material = j; break; }
		if (material < 0) material = materials.Push(m);

		entries.Push({ (int)i, chunk, material });
	}
	if (entries.Size() == 0) { built = false; return; }

	// Sort so that every (chunk, material) group is contiguous.
	std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b)
	{
		return a.chunk != b.chunk ? a.chunk < b.chunk : a.material != b.material ? a.material < b.material : a.sector < b.sector;
	});

	// Count vertices first.
	unsigned totalVerts = 0;
	for (auto &e : entries)
	{
		for (auto &section : Level->sections.SectionsForSector(e.sector)) totalVerts += section.vertexcount;
	}

	vertices.Resize(totalVerts);
	FModelVertex *out = vertices.Data();
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
			g.firstSector = groupSectors.Size();
			g.numSectors = 0;
			g.liveFloors = 0;
			g.rep = 0;
			g.valid = true;
			chunks[e.chunk].groups.Push(curGroup);
		}
		auto &g = groups[curGroup];
		auto &c = chunks[e.chunk];
		sector_t *sec = &Level->sectors[e.sector];
		const secplane_t &plane = sec->floorplane;
		const DVector3 n = plane.Normal();
		const unsigned floorFirst = v;

		for (auto &section : Level->sections.SectionsForSector(sec))
		{
			c.sections.Push(&section);
			const unsigned first = sec->iboindex[sector_t::floor] + section.vertexindex;
			for (int k = 0; k < section.vertexcount; k++)
			{
				const FFlatVertex &fv = fvb->vbo_shadowdata[fvb->ibo_data[first + k]];
				const float z = (float)plane.ZatPoint(fv.x, fv.y);
				FModelVertex &mv = out[v++];
				mv.Set(fv.x, z, fv.y, fv.u, fv.v);	// renderer convention: (map X, height, map Y)
				mv.SetNormal((float)n.X, (float)n.Z, (float)n.Y);
				mv.lu = fv.lu;
				mv.lv = fv.lv;
				mv.lindex = fv.lindex;
				mv.SetBoneSelector(0, 0, 0, 0);
				mv.SetBoneWeight(0, 0, 0, 0);
				c.x1 = min(c.x1, fv.x); c.x2 = max(c.x2, fv.x);
				c.y1 = min(c.y1, fv.y); c.y2 = max(c.y2, fv.y);
				c.z1 = min(c.z1, z); c.z2 = max(c.z2, z);
			}
			g.numVertices += section.vertexcount;
		}
		FBakedFloor baked;
		baked.FromSector(e.sector, sec);
		baked.firstVertex = floorFirst;
		baked.numVertices = v - floorFirst;
		groupSectors.Push(baked);
		g.numSectors++;
		g.liveFloors++;
		sectorGroup[e.sector] = curGroup;
	}
	Upload();

	// Seamless lines between two batched floors: skipped by the BSP walk while both batches are valid.
	for (unsigned i = 0; i < numSegs; i++)
	{
		seg_t *seg = &Level->segs[i];
		if (seg->frontsector && seg->backsector && sectorGroup[seg->frontsector->Index()] >= 0 && sectorGroup[seg->backsector->Index()] >= 0
			&& IsSeamlessLine(seg))
		{
			segSeamless[i] = 1;
		}
	}
}

//==========================================================================
//
// Per frame: which batches still match their floors.
//
//==========================================================================

void FTerrainBatches::Update(FLevelLocals *Level)
{
	frame++;
	for (auto &g : groups)
	{
		// Floors that changed leave the batch for good.
		for (unsigned i = 0; i < g.numSectors; i++)
		{
			FBakedFloor &b = groupSectors[g.firstSector + i];
			if (!b.evicted && !FloorUnchanged(&Level->sectors[b.sector], b)) Evict(g, b);
		}

		g.valid = g.liveFloors > 0;
		if (g.valid)
		{
			// Drawn with the first remaining floor's light: the others must have changed alike.
			g.rep = 0;
			while (groupSectors[g.firstSector + g.rep].evicted) g.rep++;
			const FBakedFloor &r = groupSectors[g.firstSector + g.rep];
			FFloorLight rnow;
			rnow.FromSector(&Level->sectors[r.sector]);
			const int rdelta = rnow.lightlevel - r.light.lightlevel;
			const bool rsame = rnow.SameColors(r.light);
			for (unsigned i = g.rep + 1; i < g.numSectors; i++)
			{
				const FBakedFloor &b = groupSectors[g.firstSector + i];
				if (b.evicted) continue;
				FFloorLight now;
				now.FromSector(&Level->sectors[b.sector]);
				bool ok = now.lightlevel - b.light.lightlevel == rdelta;
				if (ok) ok = b.light.SameColors(r.light) ? now.SameColors(rnow) : (rsame && now.SameColors(b.light));
				if (!ok) { g.valid = false; break; }
			}
		}
		for (unsigned i = 0; i < g.numSectors; i++)
		{
			const FBakedFloor &b = groupSectors[g.firstSector + i];
			sectorBatched[b.sector] = g.valid && !b.evicted;
		}
	}
	if (dirty) Upload();
}

void UpdateTerrainBatches(FLevelLocals *Level)
{
	FTerrainBatches *tb = Level->TerrainBatches;
	if (!gl_terrain_batch || Level->sectors.Size() == 0)
	{
		if (tb) for (auto &b : tb->sectorBatched) b = 0;
		return;
	}
	if (tb == nullptr) tb = Level->TerrainBatches = new FTerrainBatches;
	if (!tb->Matches(Level))
	{
		// Wait for the level's scripts to have run a couple of tics: floors are grouped by their light, and light set
		// up by a script on its first tic (a day/night cycle, darker shadow sectors) has to be in place first, or floors
		// lit differently from then on share batches that can never be drawn.
		if (Level->maptime < 2) return;
		tb->Build(Level);
		tb->checkedTic = INT_MIN;
	}
	if (!tb->built) return;
	// Once per game tic: nothing it checks changes between tics.
	if (tb->checkedTic != Level->maptime)
	{
		tb->checkedTic = Level->maptime;
		tb->Update(Level);
	}
	else tb->frame++;
}

// Called from the flat and line processing: is this floor drawn by a batch in this view?
bool HWDrawInfo::FloorBatched(const sector_t *sec) const
{
	return terrainBatches && terrainBatches->sectorBatched[sec->Index()];
}

bool HWDrawInfo::SegBatched(const seg_t *seg) const
{
	return terrainBatches && terrainBatches->segSeamless[seg->Index()]
		&& terrainBatches->sectorBatched[seg->frontsector->Index()] && terrainBatches->sectorBatched[seg->backsector->Index()];
}

void HWDrawInfo::SetupTerrainBatches()
{
	terrainBatches = nullptr;
	// Main views only: portals (skyboxes in particular) render their floors the normal way.
	FTerrainBatches *tb = Level->TerrainBatches;
	if (gl_terrain_batch && tb && tb->built && tb->Matches(Level) && outer == nullptr && mCurrentPortal == nullptr)
	{
		terrainBatches = tb;
	}
}

//==========================================================================
//
// Draw the visible batches (opaque pass, after the plain flats).
//
//==========================================================================

static TArray<int> visibleGroups;
static FDynLightData terrainLightData;
static TArray<FDynamicLight *> terrainLights;

void HWDrawInfo::DrawTerrainBatches(FRenderState &state)
{
	FTerrainBatches *tb = terrainBatches;
	if (tb == nullptr || tb->vb == nullptr) return;

	const DVector3 &eye = Viewpoint.Pos;
	const double culldistsq = r_distance_cull_type > 0 ? Viewpoint.culldistsq : 0.;
	const bool lights = Level->HasDynamicLights && !isFullbrightScene();

	visibleGroups.Clear();
	if (lights) screen->mLights->Map();
	for (auto &c : tb->chunks)
	{
		if (!Frustum.MapBoxVisible(c.x1, c.x2, c.y1, c.y2, c.z1, c.z2)) continue;
		if (culldistsq > 0)
		{
			const double dx = eye.X < c.x1 ? c.x1 - eye.X : (eye.X > c.x2 ? eye.X - c.x2 : 0);
			const double dy = eye.Y < c.y1 ? c.y1 - eye.Y : (eye.Y > c.y2 ? eye.Y - c.y2 : 0);
			if (dx * dx + dy * dy >= culldistsq) continue;	// past the line cull distance
		}
		bool any = false;
		for (int g : c.groups) if (tb->groups[g].valid) { visibleGroups.Push(g); any = true; }
		terrainStats.chunks += any;

		// Dynamic lights touching any floor of the chunk, gathered once per frame.
		if (any && lights && c.lightFrame != tb->frame)
		{
			c.lightFrame = tb->frame;
			c.lightIndex = -1;
			terrainLights.Clear();
			terrainLightData.Clear();
			for (FSection *section : c.sections)
			{
				auto list = Level->lightlists.flat_dlist.CheckKey(section);
				if (list == nullptr) continue;
				TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Iterator it(*list);
				TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Pair *pair;
				while (it.NextPair(pair))
				{
					auto node = pair->Value.get();
					if (node == nullptr) continue;
					FDynamicLight *light = node->lightsource;
					if (std::find(terrainLights.begin(), terrainLights.end(), light) != terrainLights.end()) continue;
					terrainLights.Push(light);
					AddLightToList(terrainLightData, c.portalGroup, light, true);
				}
			}
			if (terrainLights.Size() > 0) c.lightIndex = screen->mLights->UploadLights(terrainLightData);
		}
	}
	if (lights) screen->mLights->Unmap();
	if (visibleGroups.Size() == 0) return;

	// Fewest state changes: same material together.
	std::sort(visibleGroups.begin(), visibleGroups.end(), [=](int a, int b)
	{
		return tb->groups[a].material != tb->groups[b].material ? tb->groups[a].material < tb->groups[b].material : a < b;
	});

	const int rel = getExtraLight();
	const bool fullbright = isFullbrightScene();
	state.SetVertexBuffer(tb->vb->vertexBuffer(), 0, 0);
	state.SetInterpolationFactor(0.f);
	state.SetBoneIndexBase(-1);
	state.SetDynLight(0, 0, 0);

	int curMaterial = -1;
	bool textureMatrix = false;
	FGameTexture *texture = nullptr;
	for (int gi : visibleGroups)
	{
		const FTerrainGroup &g = tb->groups[gi];
		if (g.material != curMaterial)
		{
			curMaterial = g.material;
			const FTerrainMaterial &m = tb->materials[g.material];
			texture = TexMan.GetGameTexture(m.texture, true);
			state.ApplyTextureManipulation(&tb->materials[g.material].fx);
			state.SetMaterial(texture, UF_Texture, 0, CLAMP_NONE, NO_TRANSLATION, -1, nullptr);

			HWSectorPlane plane;
			plane.Offs = { m.offsX, m.offsY };
			plane.Scale = { m.scaleX, m.scaleY };
			plane.Angle = m.angle;
			textureMatrix = hw_SetPlaneTextureRotation(&plane, texture, state.mTextureMatrix);
			state.EnableTextureMatrix(textureMatrix);
		}

		// Light and colours as HWFlat::ProcessSector computes them, from the group's first remaining floor.
		{
			sector_t *sec = &Level->sectors[tb->groupSectors[g.firstSector + g.rep].sector];
			int lightlevel = RescaleLightLevel(sec->GetFloorLight());
			FColormap cm = sec->Colormap;
			if (texture->isFullbright())
			{
				cm.MakeWhite();
				lightlevel = 255;
			}
			SetColor(state, Level, lightmode, lightlevel, rel, fullbright, cm, 1.f);
			SetFog(state, Level, lightmode, lightlevel, rel, fullbright, &cm, false);
			state.SetObjectColor(sec->SpecialColors[sector_t::floor] | 0xff000000);
			state.SetAddColor(sec->AdditiveColors[sector_t::floor] | 0xff000000);
		}
		state.SetLightIndex(lights ? tb->chunks[g.chunk].lightIndex : -1);
		state.Draw(DT_Triangles, g.firstVertex, g.numVertices);
		terrainStats.draws++;
		flatvertices += g.numVertices;
		flatprimitives++;
	}

	state.EnableTextureMatrix(false);
	state.SetObjectColor(0xffffffff);
	state.SetAddColor(0);
	state.ApplyTextureManipulation(nullptr);
	state.SetLightIndex(-1);
	state.SetVertexBuffer(screen->mVertexData);
}
