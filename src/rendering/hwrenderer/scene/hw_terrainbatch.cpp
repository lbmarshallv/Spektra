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
** Each frame every batched floor is compared with what was baked. If one
** changed (a moving floor, a light change, a texture change), its whole
** batch falls back to normal rendering until it matches again.
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
	bool valid;							// every floor still matches what was baked (this frame)
};

struct FTerrainChunk
{
	float x1, x2, y1, y2, z1, z2;
	TArray<int> groups;
	TArray<FSection *> sections;		// for finding the dynamic lights touching the chunk
	int lightIndex = -1;
	int lightFrame = -1;
	int portalGroup = 0;
};

// Everything the floor's appearance depends on, as it was when baked. Compared byte for byte every frame.
struct FBakedFloor
{
	int sector;
	DVector3 normal;
	double d;
	sector_t::splane plane;	// texture, placement, light offset, alpha, glow, texture effects, ...
	int lightlevel;
	FColormap colormap;
	PalEntry flatColor, addColor;

	void FromSector(int index, sector_t *sec)
	{
		memset((void*)this, 0, sizeof(*this));
		sector = index;
		normal = sec->floorplane.Normal();
		d = sec->floorplane.fD();
		memcpy((void*)&plane, &sec->planes[sector_t::floor], sizeof(plane));
		lightlevel = sec->lightlevel;
		colormap = sec->Colormap;
		flatColor = sec->SpecialColors[sector_t::floor];
		addColor = sec->AdditiveColors[sector_t::floor];
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
	FModelVertexBuffer *vb = nullptr;
	int frame = 0;

	~FTerrainBatches() { delete vb; }

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

// Does the floor still look exactly like it was baked? (Cheap: this runs for every batched floor every frame.)
static bool FloorUnchanged(sector_t *sec, const FBakedFloor &b)
{
	if (sec->floorplane.fD() != b.d || sec->floorplane.Normal() != b.normal) return false;
	if (memcmp(&sec->planes[sector_t::floor], &b.plane, sizeof(b.plane))) return false;
	if (sec->lightlevel != b.lightlevel || sec->SpecialColors[sector_t::floor] != b.flatColor || sec->AdditiveColors[sector_t::floor] != b.addColor) return false;
	if (memcmp(&sec->Colormap, &b.colormap, sizeof(b.colormap))) return false;
	if (sec->Portals[sector_t::floor] != 0 || sec->GetReflect(sector_t::floor) != 0 || sec->floorplane.dithertransflag) return false;
	if (sec->GetHeightSec() || sec->e->XFloor.ffloors.Size()) return false;
	return true;
}

//==========================================================================
//
// Build: copies the floors' triangles out of the flat vertex data.
//
//==========================================================================

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
	for (unsigned i = 0; i < numSectors; i++)
	{
		sector_t *sec = &Level->sectors[i];
		if (!FloorBatchable(sec)) continue;
		auto sections = Level->sections.SectionsForSector(sec);
		if (sections.Size() == 0) continue;
		bool skip = false;
		for (auto &section : sections) if (section.flags & FSection::DONTRENDERFLOOR) skip = true;
		if (skip) continue;

		const int cx = (int)floor(sec->centerspot.X / chunkSize), cy = (int)floor(sec->centerspot.Y / chunkSize);
		const uint64_t ckey = (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy);
		int chunk;
		if (int *found = chunkMap.CheckKey(ckey)) chunk = *found;
		else
		{
			chunk = chunks.Reserve(1);
			auto &c = chunks[chunk];
			c.x1 = c.y1 = c.z1 = FLT_MAX;
			c.x2 = c.y2 = c.z2 = -FLT_MAX;
			c.portalGroup = sec->PortalGroup;
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

	vb = new FModelVertexBuffer(false, true);
	FModelVertex *out = vb->LockVertexBuffer(totalVerts);
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
			g.valid = true;
			chunks[e.chunk].groups.Push(curGroup);
		}
		auto &g = groups[curGroup];
		auto &c = chunks[e.chunk];
		sector_t *sec = &Level->sectors[e.sector];
		const secplane_t &plane = sec->floorplane;
		const DVector3 n = plane.Normal();

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
		groupSectors.Push(baked);
		g.numSectors++;
		sectorGroup[e.sector] = curGroup;
	}
	vb->UnlockVertexBuffer();

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
		g.valid = true;
		for (unsigned i = 0; i < g.numSectors; i++)
		{
			const FBakedFloor &b = groupSectors[g.firstSector + i];
			if (!FloorUnchanged(&Level->sectors[b.sector], b)) { g.valid = false; break; }
		}
		for (unsigned i = 0; i < g.numSectors; i++) sectorBatched[groupSectors[g.firstSector + i].sector] = g.valid;
	}
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
	if (!tb->Matches(Level)) tb->Build(Level);
	if (tb->built) tb->Update(Level);
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
	const double culldistsq = (r_distance_cull_type > 0) ? Viewpoint.culldistsq : 0.;
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
			if (dx * dx + dy * dy >= culldistsq) continue;
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
	for (int gi : visibleGroups)
	{
		const FTerrainGroup &g = tb->groups[gi];
		if (g.material != curMaterial)
		{
			curMaterial = g.material;
			const FTerrainMaterial &m = tb->materials[g.material];
			FGameTexture *texture = TexMan.GetGameTexture(m.texture, true);
			int lightlevel = m.lightlevel;
			FColormap cm = m.colormap;
			if (texture->isFullbright())
			{
				cm.MakeWhite();
				lightlevel = 255;
			}
			SetColor(state, Level, lightmode, lightlevel, rel, fullbright, cm, 1.f);
			SetFog(state, Level, lightmode, lightlevel, rel, fullbright, &cm, false);
			state.SetObjectColor(m.flatColor | 0xff000000);
			state.SetAddColor(m.addColor | 0xff000000);
			state.ApplyTextureManipulation(&tb->materials[g.material].fx);
			state.SetMaterial(texture, UF_Texture, 0, CLAMP_NONE, NO_TRANSLATION, -1, nullptr);

			HWSectorPlane plane;
			plane.Offs = { m.offsX, m.offsY };
			plane.Scale = { m.scaleX, m.scaleY };
			plane.Angle = m.angle;
			textureMatrix = hw_SetPlaneTextureRotation(&plane, texture, state.mTextureMatrix);
			state.EnableTextureMatrix(textureMatrix);
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
