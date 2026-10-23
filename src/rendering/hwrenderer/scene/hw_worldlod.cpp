/*
** hw_worldlod.cpp
**
** World LOD: a static, low-cost model of the outdoor level, seen past the
** line cull distance.
**
** The line cull (r_distance_cull_type, r_line_distance_cull) works exactly as
** in standard UZDoom: the BSP walk stops at the cull distance. Without this,
** nothing is seen past it. With it (gl_worldlod):
**
**   - On the first frame of a level, every outdoor sector (sky ceiling, or a
**     closed sector next to one) is snapshotted: its floors, ceilings, opaque
**     walls and opaque 3D floors. The geometry never changes after that:
**     moving floors, doors and scrolling textures are not followed.
**   - The model comes in two versions over the same triangles. The textured
**     model is drawn from the cull distance out to gl_worldlod_colordist times
**     it; past that the colour model takes over, where every surface has the
**     average colour of its texture (from a small generated colour texture),
**     so that only light splits its draws. Each pixel comes from exactly one
**     of them (the shader discards the rest), so the handover is seamless.
**   - Pixels nearer than the cull distance are discarded too: the real level
**     covers everything nearer.
**   - Light, fog and colours are live: each draw takes them from one of the
**     surfaces it holds, looked up again every gl_worldlod_envupdatetic tics, the way that surface's own sector lights
**     it now. Surfaces lit alike when the level was loaded share draws, so
**     light that changes alike for them (a day/night cycle) is followed
**     exactly; surfaces whose light changes on its own (a flickering sector)
**     follow their draw's representative instead.
**   - Sectors whose floor or ceiling moves away from where it was snapshotted
**     (a lift, a raising bridge) take their parts out of the model, and the
**     walls that depend on their height with them: better nothing than a
**     copy in the wrong place. They come back when the sector is back where it
**     was. Sectors marked nolod in UDMF are never in the model.
**     worldlod_rebuild / Level.RebuildWorldLOD() take a new snapshot.
**   - Sectors marked lodonly in UDMF (and the walls between them and their
**     neighbours) are in the model whether outdoor or not, and drawn from it
**     at any distance, not just past the cull distance. The BSP walk skips
**     every subtree that is entirely lodonly; elsewhere their lines only
**     clip. The sky behind everything is one backdrop around the camera, so
**     skipped lines need no sky walls of their own (and culled lines neither).
**   - Where the line cull closes the view (a culled line facing an outdoor
**     sector), the sky is drawn at the far plane instead of on the line, so
**     the model shows in front of it.
**
** Nothing of it runs per game tic; per frame only the chunk tests, the light
** lookups and the draws.
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
#include <limits.h>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <vector>
#include "g_levellocals.h"
#include "r_defs.h"
#include "r_sky.h"
#include "p_3dfloors.h"
#include "texturemanager.h"
#include "v_video.h"
#include "c_cvars.h"
#include "hw_material.h"
#include "hw_renderstate.h"
#include "hw_modelvertexbuffer.h"
#include "hw_cvars.h"
#include "hw_lighting.h"
#include "hwrenderer/scene/hw_drawinfo.h"
#include "hwrenderer/scene/hw_drawstructs.h"
#include "hwrenderer/scene/hw_walldispatcher.h"
#include "hwrenderer/scene/hw_portal.h"
#include "flatvertices.h"
#include "hw_clock.h"
#include "stats.h"
#include "printf.h"
#include "i_time.h"
#include "image.h"
#include "bitmap.h"
#include "palutil.h"
#include "vm.h"
#include "c_dispatch.h"

// On/off. Needs a line cull distance (r_distance_cull_type > 0).
CVAR(Bool, gl_worldlod, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Size of the map squares the model is split into for the frustum and distance tests (map units).
CVAR(Int, gl_worldlod_chunk, 4096, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Pushes the model back in depth so the real level wins where both are drawn (just past the cull distance).
CVAR(Float, gl_worldlod_bias, 128.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Past this many times the line cull distance, the colour model replaces the textured one. 1 or less: textures all the way.
CVAR(Float, gl_worldlod_colordist, 2.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// Testing switch for UDMF lodonly: off draws lodonly sectors like any other; the model is rebuilt when it changes
// (see FWorldLOD::Matches). TODO: remove before committing.
CVAR(Bool, gl_worldlod_lodonly, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)
// How often light, colour and fog are looked up again, in game tics (35 per second). 0: every frame.
CVAR(Int, gl_worldlod_envupdatetic, 1, CVAR_ARCHIVE | CVAR_GLOBALCONFIG)

EXTERN_CVAR(Int, r_distance_cull_type)
EXTERN_CVAR(Bool, gl_seamless)
EXTERN_CVAR(Bool, gl_render_walls)
EXTERN_CVAR(Bool, gl_render_flats)

bool hw_SetPlaneTextureRotation(const HWSectorPlane *secplane, FGameTexture *gltexture, VSMatrix &mat);

static cycle_t lodDrawTime;
static double lodBuildMS = 0;
static int lodStatTexDraws = 0, lodStatColorDraws = 0, lodStatTexChunks = 0, lodStatColorChunks = 0;
static int lodStatEnvUpdates = 0;	// light lookups since the last stat display
static double lodEnvMS = 0;			// how long the last light lookup took

//==========================================================================
//
// Outdoor sectors
//
//==========================================================================

// Sky ceiling (or a sky/horizon portal there).
static bool SkyCeiling(const sector_t *sec)
{
	if (sec->GetTexture(sector_t::ceiling) == skyflatnum) return true;
	if (sec->Portals[sector_t::ceiling] != 0)
	{
		const FSectorPortal *p = const_cast<sector_t *>(sec)->GetPortal(sector_t::ceiling);
		return p && (p->mType == PORTS_SKYVIEWPOINT || p->mType == PORTS_HORIZON);
	}
	return false;
}

// Outdoor: the ceiling is sky, or the sector is closed (a solid block or a shut door) next to an outdoor one.
bool HWSectorOutdoor(const sector_t *sec)
{
	if (SkyCeiling(sec)) return true;
	const DVector2 c = sec->centerspot;
	if (sec->ceilingplane.ZatPoint(c) > sec->floorplane.ZatPoint(c)) return false;
	for (auto line : sec->Lines)
	{
		const sector_t *other = line->frontsector == sec ? line->backsector : line->frontsector;
		if (other && other != sec && SkyCeiling(other)) return true;
	}
	return false;
}

//==========================================================================
//
// Light
//
//==========================================================================

// Light and colours a surface is drawn with.
struct FLodLight
{
	int lightlevel, rellight;
	FColormap colormap;
	PalEntry objColor, addColor;

	void Clear() { memset((void *)this, 0, sizeof(*this)); }
	bool operator==(const FLodLight &o) const { return !memcmp((const void *)this, (const void *)&o, sizeof(*this)); }
};

enum { LOD_WALL, LOD_FILL, LOD_FLOOR, LOD_CEILING, LOD_3D };

// Where a surface's light comes from, so it can be looked up again every frame.
struct FLodLightSource
{
	uint8_t kind;		// LOD_*
	bool fixed;			// the lookup doesn't give what the surface was built with: keep the snapshot
	bool bottom;		// LOD_3D: the 3D floor's bottom
	uint8_t tier;		// LOD_WALL: side_t::top/mid/bottom
	bool ext;			// LOD_WALL: the side has its own tier colours
	int sector;			// LOD_FILL, LOD_FLOOR, LOD_CEILING, LOD_3D (the sector the plane is in)
	int side;			// LOD_WALL
	int ffloor;			// LOD_3D: index in the sector's 3D floors
	int band;			// LOD_WALL in a sector with 3D floors: the entry of src.sector's light list that lights it (-1: none)
};

// Same as CalcRelLight in hw_walls.cpp.
static int LodRelLight(int lightlevel, int orglightlevel, int rel)
{
	if (orglightlevel >= 253) return 0;
	else if (lightlevel - rel > 256) return 256 - lightlevel + rel;
	else return rel;
}

// The light the surface gets now, as the normal wall and flat code works it out. False if it can't be found.
static bool LookupLight(FLevelLocals *Level, const FLodLightSource &src, FLodLight &out)
{
	out.Clear();
	switch (src.kind)
	{
	case LOD_WALL:
	{
		if ((unsigned)src.side >= Level->sides.Size()) return false;
		side_t *side = &Level->sides[src.side];
		sector_t *sec = side->sector;
		const int orglight = hw_ClampLight(sec->lightlevel);
		const bool foggy = !sec->Colormap.FadeColor.isBlack() || (Level->flags & LEVEL_HASFADETABLE);
		int rel = 0;
		out.lightlevel = RescaleLightLevel(side->GetLightLevel(foggy, orglight, src.tier, false, &rel));
		out.rellight = LodRelLight(out.lightlevel, orglight, rel);
		FColormap cm = sec->Colormap;
		if (src.band >= 0)
		{
			// Lit by an entry of a 3D-floor light list, as HWWall::Put3DWall does it.
			if ((unsigned)src.sector >= Level->sectors.Size()) return false;
			auto &ll = Level->sectors[src.sector].e->XFloor.lightlist;
			if ((unsigned)src.band >= ll.Size()) return false;
			lightlist_t *light = &ll[src.band];
			if (light->p_lightlevel != &sec->lightlevel) out.lightlevel = RescaleLightLevel(*light->p_lightlevel);
			CopyFrom3DLight(cm, light);
		}
		memcpy((void *)&out.colormap, &cm, sizeof(out.colormap));
		out.objColor = 0xffffffff;
		if (src.ext)
		{
			out.objColor = side->GetSpecialColor(src.tier, side_t::walltop, sec);
			out.addColor = side->GetAdditiveColor(src.tier, sec);
		}
		return true;
	}
	case LOD_FILL:
	{
		if ((unsigned)src.sector >= Level->sectors.Size()) return false;
		sector_t *sec = &Level->sectors[src.sector];
		out.lightlevel = RescaleLightLevel(hw_ClampLight(sec->lightlevel));
		memcpy((void *)&out.colormap, &sec->Colormap, sizeof(out.colormap));
		out.objColor = 0xffffffff;
		return true;
	}
	case LOD_FLOOR:
	case LOD_CEILING:
	{
		if ((unsigned)src.sector >= Level->sectors.Size()) return false;
		sector_t *sec = &Level->sectors[src.sector];
		const int plane = src.kind == LOD_FLOOR ? sector_t::floor : sector_t::ceiling;
		out.lightlevel = RescaleLightLevel(plane == sector_t::floor ? sec->GetFloorLight() : sec->GetCeilingLight());
		memcpy((void *)&out.colormap, &sec->Colormap, sizeof(out.colormap));
		out.objColor = sec->SpecialColors[plane] | 0xff000000;
		out.addColor = sec->AdditiveColors[plane] | 0xff000000;
		return true;
	}
	case LOD_3D:
	{
		if ((unsigned)src.sector >= Level->sectors.Size()) return false;
		sector_t *sec = &Level->sectors[src.sector];
		auto &ffloors = sec->e->XFloor.ffloors;
		if ((unsigned)src.ffloor >= ffloors.Size()) return false;
		F3DFloor *rover = ffloors[src.ffloor];
		F3DFloor::planeref &pr = src.bottom ? rover->bottom : rover->top;
		lightlist_t *light = P_GetPlaneLight(sec, pr.plane, src.bottom);
		out.lightlevel = RescaleLightLevel(*light->p_lightlevel);
		FColormap cm = sec->Colormap;
		CopyFrom3DLight(cm, light);
		memcpy((void *)&out.colormap, &cm, sizeof(out.colormap));
		out.objColor = pr.model->SpecialColors[pr.isceiling] | 0xff000000;
		out.addColor = pr.model->AdditiveColors[pr.isceiling] | 0xff000000;
		return true;
	}
	}
	return false;
}

//==========================================================================
//
// Colour texture: one texel per average texture colour
//
//==========================================================================

static constexpr int LOD_PALETTE_SIZE = 64;	// 64 x 64 texels: up to 4096 colours

class FLodPaletteImage : public FImageSource
{
public:
	TArray<uint8_t> bgra;

	FLodPaletteImage()
	{
		Width = Height = LOD_PALETTE_SIZE;
		bgra.Resize(Width * Height * 4);
		memset(bgra.Data(), 255, bgra.Size());
	}

	int CopyPixels(FBitmap *bmp, int conversion, int frame = 0) override
	{
		bmp->CopyPixelDataRGB(0, 0, bgra.Data(), Width, Height, 4, Width * 4, 0, CF_BGRA);
		return 0;
	}
};

//==========================================================================
//
// Data
//
//==========================================================================

// How a part of the textured model is drawn: texture, placement, and light and colours as snapshotted.
struct FLodMaterial
{
	FGameTexture *texture;
	int kind;					// LOD_*
	int clamp;					// walls: HWF_CLAMPX | HWF_CLAMPY
	bool masked;				// drawn in the masked pass (two-sided middle textures and 3D-floor sides with holes)
	bool lodonly;				// lodonly sector or line: drawn at any distance, not just past the cull distance
	bool m2s;					// two-sided middle texture (clamped vertically like HWWall does)
	bool ext;					// wall with its own texture effects
	TextureManipulation fx;
	float offsX, offsY, scaleX, scaleY, angle;	// flats
	FLodLight light;
	int slot;					// its light slot (see FLodSlot)

	bool operator==(const FLodMaterial &o) const { return !memcmp((const void *)this, (const void *)&o, sizeof(*this)); }
};

// A group's triangles in one chunk: vertices [first, first + count).
struct FLodRange
{
	int chunk;
	unsigned first, count;
};

// Light slot: surfaces whose light was the same at load and whose light is bound the same way (same sector tag,
// same kind of surface, same fake contrast and tier or plane light offsets), so it will stay the same. One light
// lookup per slot serves every draw of it.
struct FLodSlot
{
	FLodLight light;			// as snapshotted
	FLodLightSource source;		// where its light comes from now
	FLodLight current;			// as last looked up (see gl_worldlod_envupdatetic)
};

// All of a draw group's triangles, by chunk (ranges in chunk order, consecutive in the vertex buffer).
// Textured model: one group per material. Colour model: one group per light slot.
struct FLodGroup
{
	int material;				// textured model; -1 in the colour model
	int slot;					// its light
	unsigned firstRange, numRanges;
	bool masked;
	bool wall;
	bool lodonly;
};

struct FLodChunk
{
	float x1 = FLT_MAX, x2 = -FLT_MAX, y1 = FLT_MAX, y2 = -FLT_MAX, z1 = FLT_MAX, z2 = -FLT_MAX;
	bool texVisible = false;		// this frame: textured model in view here
	bool colorVisible = false;		// this frame: colour model in view here
	bool nearVisible = false;		// this frame: lodonly pieces of the textured model in view here (any distance)
};

struct FWorldLOD
{
	const sector_t *sectorsData = nullptr;
	unsigned numSectors = 0, numSides = 0;
	int chunkSize = 0;
	bool seamless = false;
	bool built = false;
	int checkedFrame = -1;		// chunk tests run once per frame
	int envTic = INT_MIN;		// game tic of the last light lookup (INT_MIN: never)
	int frame = 0;

	TArray<FLodMaterial> materials;
	TArray<FLodChunk> chunks;
	TArray<FLodGroup> texGroups;	// masked materials last
	TArray<FLodGroup> colorGroups;
	TArray<FLodRange> ranges;
	TArray<FLodSlot> slots;
	unsigned numVertices = 0, numColors = 0;
	int outdoorSectors = 0, fixedLights = 0, nolodSectors = 0;

	// Moving sectors. Every piece of the model (an entry: one wall part or one section of a plane) knows where its
	// vertices are in the buffer and which sectors' heights it depends on. A sector whose planes are no longer where
	// they were snapshotted hides the pieces depending on it (their vertices are zeroed in the buffer); they are put
	// back from the CPU copy of the textured vertices when it returns.
	TArray<double> planes;			// per sector: floor D, floor normal Z, ceiling D, ceiling normal Z, as snapshotted
	TArray<int> watched;			// sectors that pieces depend on
	TArray<int> depStart, depEntries;	// per sector: the pieces depending on its height
	TArray<unsigned> entryTex, entryColor, entryCount;	// per piece: its vertices in the textured and colour model (~0u: none)
	TArray<int> entryTexel;			// per piece: its colour texel (colour model)
	TArray<unsigned> entryDepFirst, entryDepCount;
	TArray<int> entryDeps;
	TArray<uint8_t> sectorMoved, entryHidden;
	TArray<FModelVertex> texVerts;	// CPU copy of the textured model's vertices, for putting hidden pieces back
	int movedSectors = 0, hiddenPieces = 0;
	int checkedTic = INT_MIN;		// game tic of the last moving-sector check

	// lodonly: BSP subtrees entirely in lodonly sectors (skipped by the walk)
	bool lodOnlyMode = false;		// gl_worldlod_lodonly when built
	TArray<uint8_t> subLodOnly, nodeLodOnly;
	int lodOnlySectors = 0, lodOnlyNodes = 0;
	FModelVertexBuffer *vb = nullptr;
	FGameTexture *palette = nullptr;	// the colour model's texture (owned by the texture manager)

	~FWorldLOD() { delete vb; }

	bool Matches(FLevelLocals *Level) const
	{
		return sectorsData == Level->sectors.Data() && numSectors == Level->sectors.Size() && numSides == Level->sides.Size()
			&& chunkSize == clamp<int>(gl_worldlod_chunk, 512, 65536) && seamless == !!gl_seamless && lodOnlyMode == !!gl_worldlod_lodonly;
	}

	void Build(FLevelLocals *Level);
	void CheckMoved(FLevelLocals *Level);
	void RequestRebuild() { sectorsData = nullptr; }
	bool LodOnly(const sector_t *sec) const { return lodOnlyMode && HWSectorLodOnly(sec); }	// rebuilt by UpdateWorldLOD before the next frame is drawn
};

void DeleteWorldLOD(FWorldLOD *lod)
{
	delete lod;
}

//==========================================================================
//
// Light slots: surfaces only share a light (and so a draw) if it was equal
// when the snapshot was taken (after the level's scripts have run a couple of
// tics) and is bound the same way: same kind of surface, same fake contrast
// and light offsets (they change with fog), and for 3D-floor light the same
// sector providing the light level.
//
//==========================================================================

// Which entry of a sector's 3D-floor light list lights a point (as HWWall::SplitWall puts each piece in one entry).
static int LightBand(sector_t *sec, double x, double y, double z)
{
	auto &ll = sec->e->XFloor.lightlist;
	for (unsigned i = 0; i < ll.Size(); i++)
	{
		const double lower = i + 1 < ll.Size() ? ll[i + 1].plane.ZatPoint(x, y) : -1e30;
		if (z >= lower) return i;
	}
	return (int)ll.Size() - 1;
}

// The sector whose light level a 3D-floor light entry uses (a 3D floor's control sector, or the sector itself), or -1.
static int LightOwner(FLevelLocals *Level, const short *p_lightlevel)
{
	if (Level->sectors.Size() == 0 || p_lightlevel == nullptr) return -1;
	const ptrdiff_t off = (const uint8_t *)p_lightlevel - (const uint8_t *)&Level->sectors[0].lightlevel;
	if (off < 0 || off % (ptrdiff_t)sizeof(sector_t) != 0) return -1;
	const size_t i = off / sizeof(sector_t);
	return i < Level->sectors.Size() && &Level->sectors[i].lightlevel == p_lightlevel ? (int)i : -1;
}

//==========================================================================
//
// Build: the snapshot
//
//==========================================================================

void FWorldLOD::Build(FLevelLocals *Level)
{
	const uint64_t startTime = I_nsTime();
	delete vb;
	vb = nullptr;
	materials.Clear();
	chunks.Clear();
	texGroups.Clear();
	colorGroups.Clear();
	ranges.Clear();
	slots.Clear();
	numVertices = numColors = 0;
	outdoorSectors = fixedLights = nolodSectors = 0;
	movedSectors = hiddenPieces = 0;
	checkedTic = INT_MIN;
	planes.Clear(); watched.Clear(); depStart.Clear(); depEntries.Clear();
	entryTex.Clear(); entryColor.Clear(); entryCount.Clear(); entryTexel.Clear();
	entryDepFirst.Clear(); entryDepCount.Clear(); entryDeps.Clear();
	sectorMoved.Clear(); entryHidden.Clear(); texVerts.Clear();
	envTic = INT_MIN;
	palette = nullptr;
	lodOnlyMode = gl_worldlod_lodonly;
	lodOnlySectors = lodOnlyNodes = 0;
	subLodOnly.Clear();
	nodeLodOnly.Clear();
	sectorsData = Level->sectors.Data();
	numSectors = Level->sectors.Size();
	numSides = Level->sides.Size();
	chunkSize = clamp<int>(gl_worldlod_chunk, 512, 65536);
	seamless = gl_seamless;
	built = true;

	auto fvb = screen->mVertexData;
	if (numSectors == 0 || fvb == nullptr || fvb->ibo_data.Size() == 0) return;

	TArray<uint8_t> outdoor;
	outdoor.Resize(numSectors);
	for (unsigned i = 0; i < numSectors; i++)
	{
		const bool nolod = !!(Level->sectors[i].MoreFlags & SECMF_NOLOD);
		nolodSectors += nolod;
		outdoor[i] = !nolod && HWSectorOutdoor(&Level->sectors[i]);
		outdoorSectors += outdoor[i];
	}

	// Triangles go into temp, each piece noted with its material, light source and chunk; sorted into groups at the end.
	struct Entry { int material, chunk; FLodLightSource source; unsigned first, count; unsigned depFirst, depCount; };
	TArray<int> curDeps;			// sectors whose height the pieces being made depend on
	bool curLodOnly = false;		// the pieces being made belong to a lodonly sector or line
	auto addDep = [&](sector_t *s) { if (s && curDeps.Find(s->Index()) == curDeps.Size()) curDeps.Push(s->Index()); };
	TArray<Entry> entries;
	TArray<FModelVertex> temp;
	TMap<uint64_t, int> chunkMap;

	// Materials by a hash of their bytes (they are memset first, so padding is always zero).
	std::unordered_map<uint64_t, std::vector<int>> materialMap;
	auto findMaterial = [&](const FLodMaterial &m) -> int
	{
		uint64_t h = 14695981039346656037ull;
		const uint8_t *bytes = (const uint8_t *)&m;
		for (size_t i = 0; i < sizeof(m); i++) h = (h ^ bytes[i]) * 1099511628211ull;
		auto &list = materialMap[h];
		for (int j : list) if (materials[j] == m) return j;
		const int j = materials.Push(m);
		list.push_back(j);
		return j;
	};
	auto findChunk = [&](double x, double y) -> int
	{
		const uint64_t key = HWCellKey(x, y, chunkSize);
		if (int *found = chunkMap.CheckKey(key)) return *found;
		const int c = chunks.Reserve(1);
		chunkMap[key] = c;
		return c;
	};
	// The light source is only used if looking it up now gives what the surface was built with.
	auto checkSource = [&](FLodLightSource &src, const FLodLight &built)
	{
		FLodLight now;
		if (src.fixed || !LookupLight(Level, src, now) || !(now == built))
		{
			src.fixed = true;
			fixedLights++;
		}
	};
	// What decides how a surface's light changes later, besides its light now.
	struct SlotKey
	{
		FLodLight light;
		int kind;				// LOD_*
		int fixed;				// fixed slots never change
		int lightFlags, base, tierLight;	// walls: side light flags and offsets
		int planeLight, planeAbs;	// floors and ceilings: plane light offset
		int lightOwner;			// band walls and 3D planes: the sector whose light level the 3D-floor light uses (+1), 0 for their own
	};
	std::unordered_map<uint64_t, std::vector<int>> slotMap;
	TArray<SlotKey> slotKeys;
	auto findSlot = [&](const FLodLightSource &src, const FLodLight &light) -> int
	{
		SlotKey key;
		memset((void *)&key, 0, sizeof(key));
		key.light = light;
		key.kind = src.kind;
		key.fixed = src.fixed;
		if (src.fixed) {}	// keeps the light it was built with: shares with anything fixed at the same light
		else if (src.kind == LOD_WALL && (unsigned)src.side < Level->sides.Size())
		{
			side_t *side = &Level->sides[src.side];
			// Fake contrast isn't part of the key: walls whose contrast was equal when snapshotted share a slot. (In fog it is
			// off for all of them; a wall whose contrast comes back when the fog goes takes its slot's.)
			key.lightFlags = side->Flags & (WALLF_ABSLIGHTING | (WALLF_ABSLIGHTING_TIER << src.tier));
			key.base = side->Light;
			key.tierLight = side->TierLights[src.tier];
			if (src.band >= 0 && (unsigned)src.sector < Level->sectors.Size())
			{
				auto &ll = Level->sectors[src.sector].e->XFloor.lightlist;
				if ((unsigned)src.band < ll.Size())
				{
					const int owner = LightOwner(Level, ll[src.band].p_lightlevel);
					key.lightOwner = owner == side->sector->Index() ? 0 : 1 + owner;	// 0: its own sector's light, like any wall
				}
			}
		}
		else if ((src.kind == LOD_FLOOR || src.kind == LOD_CEILING) && (unsigned)src.sector < Level->sectors.Size())
		{
			sector_t *sec = &Level->sectors[src.sector];
			const int plane = src.kind == LOD_FLOOR ? sector_t::floor : sector_t::ceiling;
			key.planeLight = sec->GetPlaneLight(plane);
			key.planeAbs = !!(sec->GetFlags(plane) & PLANEF_ABSLIGHTING);
		}
		else if (src.kind == LOD_3D && (unsigned)src.sector < Level->sectors.Size())
		{
			sector_t *sec = &Level->sectors[src.sector];
			auto &ffloors = sec->e->XFloor.ffloors;
			if ((unsigned)src.ffloor < ffloors.Size())
			{
				F3DFloor::planeref &pr = src.bottom ? ffloors[src.ffloor]->bottom : ffloors[src.ffloor]->top;
				const int owner = LightOwner(Level, P_GetPlaneLight(sec, pr.plane, src.bottom)->p_lightlevel);
				key.lightOwner = owner == src.sector ? 0 : 1 + owner;	// 0: its own sector's light
			}
		}
		uint64_t h = 14695981039346656037ull;
		const uint8_t *bytes = (const uint8_t *)&key;
		for (size_t i = 0; i < sizeof(key); i++) h = (h ^ bytes[i]) * 1099511628211ull;
		auto &list = slotMap[h];
		for (int j : list) if (!memcmp((const void *)&slotKeys[j], (const void *)&key, sizeof(key))) return j;
		FLodSlot slot;
		slot.light = slot.current = light;
		slot.source = src;
		const int j = slots.Push(slot);
		slotKeys.Push(key);
		list.push_back(j);
		return j;
	};
	auto addPiece = [&](int material, const FLodLightSource &src, unsigned first)
	{
		if (temp.Size() <= first) return;
		// The piece goes to the chunk its middle is in; the chunk's box grows to hold all of it.
		double mx = 0, my = 0;
		for (unsigned k = first; k < temp.Size(); k++) { mx += temp[k].x; my += temp[k].z; }
		const unsigned n = temp.Size() - first;
		const int c = findChunk(mx / n, my / n);
		auto &ch = chunks[c];
		for (unsigned k = first; k < temp.Size(); k++)
		{
			const FModelVertex &mv = temp[k];
			ch.x1 = min(ch.x1, mv.x); ch.x2 = max(ch.x2, mv.x);
			ch.y1 = min(ch.y1, mv.z); ch.y2 = max(ch.y2, mv.z);	// map Y
			ch.z1 = min(ch.z1, mv.y); ch.z2 = max(ch.z2, mv.y);	// height
		}
		const unsigned depFirst = entryDeps.Size();
		for (int d : curDeps) entryDeps.Push(d);
		entries.Push({ material, c, src, first, n, depFirst, curDeps.Size() });
	};
	auto pushVertex = [&](float x, float z, float y, float u, float v, const FVector3 &n)
	{
		FModelVertex mv;
		mv.Set(x, z, y, u, v);	// renderer convention: (map X, height, map Y)
		mv.SetNormal(n.X, n.Y, n.Z);
		mv.lu = mv.lv = 0;
		mv.lindex = -1;
		mv.SetBoneSelector(0, 0, 0, 0);
		mv.SetBoneWeight(0, 0, 0, 0);
		temp.Push(mv);
	};

	// Walls: the normal wall setup in mesh mode, for every side facing an outdoor sector. Opaque and masked parts only.
	const ELightMode lightmode = getRealLightmode(Level, true);
	HWMeshHelper mh;
	TArray<FFlatVertex> fan;
	for (unsigned si = 0; si < numSides; si++)
	{
		side_t *side = &Level->sides[si];
		if (side->sector == nullptr || side->linedef == nullptr) continue;
		if (side->Flags & WALLF_POLYOBJ) continue;
		// Nothing between a nolod sector and its neighbours either (those walls only exist because of it).
		line_t *ln = side->linedef;
		if ((ln->frontsector && (ln->frontsector->MoreFlags & SECMF_NOLOD)) || (ln->backsector && (ln->backsector->MoreFlags & SECMF_NOLOD))) continue;
		// Lines touching a lodonly sector are always in (both sides): nothing else draws them.
		curLodOnly = LodOnly(ln->frontsector) || LodOnly(ln->backsector);
		if (!outdoor[side->sector->Index()] && !curLodOnly) continue;
		if (side->numsegs <= 0 || side->segs == nullptr) continue;
		seg_t *seg = side->segs[0];
		if (seg == nullptr || seg->sidedef != side || seg->Subsector == nullptr || seg->frontsector == nullptr) continue;

		sector_t *front = seg->frontsector;
		sector_t *back = seg->backsector;
		if (back && back->sectornum == front->sectornum) back = front;

		// The wall's shape depends on the heights of the sectors on both sides and of their 3D floors' control sectors.
		curDeps.Clear();
		for (sector_t *s : { ln->frontsector, ln->backsector })
		{
			if (s == nullptr) continue;
			addDep(s);
			for (auto *ff : s->e->XFloor.ffloors) addDep(ff->model);
		}

		mh.list.Clear();
		mh.translucent.Clear();
		mh.portals.Clear();
		mh.lower.Clear();
		mh.upper.Clear();
		HWWallDispatcher disp(Level, &mh, lightmode);
		HWWall wall;
		wall.sub = seg->Subsector;
		wall.Process(&disp, seg, front, back, false);
		sector_t *lsec = seg->Subsector->sector;	// whose 3D-floor light list lights the walls (as in HWWall::DoTexture)
		const bool lightList = lsec->e->XFloor.lightlist.Size() > 0;

		for (auto &w : mh.list)
		{
			if (w.type != RENDERWALL_TOP && w.type != RENDERWALL_BOTTOM && w.type != RENDERWALL_M1S && w.type != RENDERWALL_M2S && w.type != RENDERWALL_FFBLOCK) continue;
			if (w.texture == nullptr || !w.texture->isValid()) continue;
			if (w.flags & HWWall::HWF_SKYHACK) continue;
			// Two-sided mid textures and 3D-floor sides get STYLE_Translucent whenever the line has an alpha, even at 1.
			const bool opaqueTranslucent = w.RenderStyle == STYLE_Translucent && w.alpha >= 1.f && (w.type == RENDERWALL_M2S || w.type == RENDERWALL_FFBLOCK);
			if (!opaqueTranslucent && (w.RenderStyle != STYLE_Normal || w.alpha != 1.f)) continue;

			const bool split = seamless && !(w.flags & HWWall::HWF_NOSPLIT);
			fan.Resize(split ? w.CountVertices() : 4);
			FFlatVertex *ptr = fan.Data();
			const int count = w.CreateVertices(ptr, split);
			if (count < 3) continue;

			const int tier = w.type == RENDERWALL_TOP || w.type == RENDERWALL_FFBLOCK ? side_t::top : w.type == RENDERWALL_BOTTOM ? side_t::bottom : side_t::mid;
			FLodMaterial m;
			memset((void *)&m, 0, sizeof(m));
			m.texture = w.texture;
			m.kind = LOD_WALL;
			m.lodonly = curLodOnly;
			m.clamp = w.flags & 3;
			m.m2s = w.type == RENDERWALL_M2S;
			m.masked = (m.m2s || w.type == RENDERWALL_FFBLOCK) && w.texture->isMasked();
			m.light.lightlevel = w.lightlevel;
			m.light.rellight = w.rellight;
			memcpy((void *)&m.light.colormap, &w.Colormap, sizeof(m.light.colormap));
			m.light.objColor = 0xffffffff;
			FLodLightSource src;
			memset((void *)&src, 0, sizeof(src));
			src.kind = LOD_WALL;
			src.side = si;
			src.tier = tier;
			src.band = -1;
			if (side->Flags & WALLF_EXTCOLOR)
			{
				m.ext = src.ext = true;
				m.light.objColor = side->GetSpecialColor(tier, side_t::walltop, w.frontsector);
				m.light.addColor = side->GetAdditiveColor(tier, w.frontsector);
				memcpy((void *)&m.fx, &side->textures[tier].TextureFx, sizeof(m.fx));
			}
			if (lightList)
			{
				// Under 3D floors the wall is lit by the light list entry at its height. The model keeps one light per
				// wall piece: the entry at its middle.
				src.sector = lsec->Index();
				src.band = LightBand(lsec, (w.glseg.x1 + w.glseg.x2) * 0.5, (w.glseg.y1 + w.glseg.y2) * 0.5,
					(w.ztop[0] + w.ztop[1] + w.zbottom[0] + w.zbottom[1]) * 0.25);
				LookupLight(Level, src, m.light);
			}
			checkSource(src, m.light);
			m.slot = findSlot(src, m.light);
			const int material = findMaterial(m);

			const FVector3 n = w.glseg.Normal();
			const unsigned first = temp.Size();
			for (int k = 1; k + 1 < count; k++)
			{
				for (int i : { 0, k, k + 1 })
				{
					const FFlatVertex &fv = fan[i];
					pushVertex(fv.x, fv.z, fv.y, fv.u, fv.v, n);
				}
			}
			addPiece(material, src, first);
		}

		// Missing-texture fills: the normal renderer floods them with the neighbouring plane; snapshot that as a
		// plain wall with the neighbouring sector's plane texture.
		if (back && back != front && (mh.lower.Size() || mh.upper.Size()))
		{
			line_t *line = side->linedef;
			vertex_t *va = side == line->sidedef[0] ? line->v1 : line->v2;
			vertex_t *vb2 = side == line->sidedef[0] ? line->v2 : line->v1;
			const double ax = va->fX(), ay = va->fY(), bx = vb2->fX(), by = vb2->fY();
			const double len = sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
			for (int part = 0; part < 2 && len > 0; part++)
			{
				if (part == 0 && !mh.lower.Size()) continue;
				if (part == 1 && !mh.upper.Size()) continue;
				const int plane = part == 0 ? sector_t::floor : sector_t::ceiling;
				if (back->GetTexture(plane) == skyflatnum || front->GetTexture(plane) == skyflatnum) continue;
				FGameTexture *tex = TexMan.GetGameTexture(back->GetTexture(plane), true);
				if (!tex || !tex->isValid() || tex->GetDisplayWidth() <= 0 || tex->GetDisplayHeight() <= 0) continue;
				double zb[2], zt[2];
				const double xs[2] = { ax, bx }, ys[2] = { ay, by };
				for (int e = 0; e < 2; e++)
				{
					if (part == 0) { zb[e] = front->floorplane.ZatPoint(xs[e], ys[e]); zt[e] = back->floorplane.ZatPoint(xs[e], ys[e]); }
					else { zb[e] = back->ceilingplane.ZatPoint(xs[e], ys[e]); zt[e] = front->ceilingplane.ZatPoint(xs[e], ys[e]); }
					if (zt[e] < zb[e]) zt[e] = zb[e];
				}
				if (zt[0] <= zb[0] && zt[1] <= zb[1]) continue;

				FLodMaterial m;
				memset((void *)&m, 0, sizeof(m));
				m.texture = tex;
				m.kind = LOD_WALL;
				m.lodonly = curLodOnly;
				FLodLightSource src;
				memset((void *)&src, 0, sizeof(src));
				src.kind = LOD_FILL;
				src.sector = front->Index();
				LookupLight(Level, src, m.light);
				m.slot = findSlot(src, m.light);
				const int material = findMaterial(m);

				const float tw = (float)tex->GetDisplayWidth(), th = (float)tex->GetDisplayHeight();
				const FVector3 n((float)((by - ay) / len), 0.f, (float)(-(bx - ax) / len));
				const float q[4][5] = {
					{ (float)ax, (float)zb[0], (float)ay, 0.f, (float)(-zb[0] / th) },
					{ (float)ax, (float)zt[0], (float)ay, 0.f, (float)(-zt[0] / th) },
					{ (float)bx, (float)zt[1], (float)by, (float)(len / tw), (float)(-zt[1] / th) },
					{ (float)bx, (float)zb[1], (float)by, (float)(len / tw), (float)(-zb[1] / th) } };
				const unsigned first = temp.Size();
				for (int i : { 0, 1, 2, 0, 2, 3 }) pushVertex(q[i][0], q[i][1], q[i][2], q[i][3], q[i][4], n);
				addPiece(material, src, first);
			}
		}
	}

	// Flats: one plane's triangles over all of a sector's sections, wound to face up or down like the walls' front faces.
	auto bakePlane = [&](sector_t *sec, const secplane_t &sp, int iboBase, int kind, bool faceUp, sector_t *model, int modelPlane, FTextureID texid,
		FLodLightSource &src, int dontRender)
	{
		if (texid == skyflatnum) return;
		if (iboBase < 0) return;	// no vertices were made for it (3D-floor planes the flat code doesn't draw)
		FGameTexture *tex = TexMan.GetGameTexture(texid, true);
		if (!tex || !tex->isValid()) return;

		FLodMaterial m;
		memset((void *)&m, 0, sizeof(m));
		m.texture = tex;
		m.kind = kind;
		m.lodonly = curLodOnly;
		m.offsX = (float)model->GetXOffset(modelPlane);
		m.offsY = (float)model->GetYOffset(modelPlane);
		m.scaleX = (float)model->GetXScale(modelPlane);
		m.scaleY = (float)model->GetYScale(modelPlane);
		m.angle = (float)model->GetAngle(modelPlane).Degrees();
		LookupLight(Level, src, m.light);
		if (tex->isFullbright())
		{
			m.light.colormap.MakeWhite();
			m.light.lightlevel = 255;
			src.fixed = true;
		}
		memcpy((void *)&m.fx, &model->planes[modelPlane].TextureFx, sizeof(m.fx));
		checkSource(src, m.light);
		m.slot = findSlot(src, m.light);
		const int material = findMaterial(m);

		DVector3 pn = sp.Normal();
		if ((pn.Z > 0) != faceUp) pn = -pn;
		const FVector3 n((float)pn.X, (float)pn.Z, (float)pn.Y);	// renderer convention; points to the side it is seen from
		for (auto &section : Level->sections.SectionsForSector(sec))
		{
			if (section.flags & dontRender) continue;
			if (section.vertexindex < 0 || section.vertexcount <= 0) continue;
			const unsigned base = iboBase + section.vertexindex;
			if (base + section.vertexcount > fvb->ibo_data.Size()) continue;	// out of range: skip rather than read garbage
			const unsigned first = temp.Size();
			for (int k = 0; k + 2 < section.vertexcount; k += 3)
			{
				const unsigned t = temp.Size();
				bool bad = false;
				for (int i = 0; i < 3; i++) if ((unsigned)fvb->ibo_data[base + k + i] >= fvb->vbo_shadowdata.Size()) bad = true;
				if (bad) continue;
				for (int i = 0; i < 3; i++)
				{
					const FFlatVertex &fv = fvb->vbo_shadowdata[fvb->ibo_data[base + k + i]];
					pushVertex(fv.x, (float)sp.ZatPoint(fv.x, fv.y), fv.y, fv.u, fv.v, n);
				}
				// Wind it like the walls: front face where cross(v1-v0, v2-v0) points along the normal (back-face culling).
				FModelVertex *tri = &temp[t];
				const FVector3 a(tri[1].x - tri[0].x, tri[1].y - tri[0].y, tri[1].z - tri[0].z);
				const FVector3 b(tri[2].x - tri[0].x, tri[2].y - tri[0].y, tri[2].z - tri[0].z);
				if (((a ^ b) | n) < 0) std::swap(tri[1], tri[2]);
			}
			addPiece(material, src, first);	// per section, so a big sector spreads over the chunks it covers
		}
	};

	for (unsigned s = 0; s < numSectors; s++)
	{
		sector_t *sec = &Level->sectors[s];
		curLodOnly = LodOnly(sec);
		if (!outdoor[s] && !curLodOnly) continue;
		FLodLightSource src;
		memset((void *)&src, 0, sizeof(src));
		src.sector = s;
		curDeps.Clear();
		addDep(sec);
		if (sec->Portals[sector_t::floor] == 0)
		{
			src.kind = LOD_FLOOR;
			src.fixed = false;
			bakePlane(sec, sec->floorplane, sec->iboindex[sector_t::floor], LOD_FLOOR, true, sec, sector_t::floor, sec->GetTexture(sector_t::floor),
				src, FSection::DONTRENDERFLOOR);
		}
		if (sec->Portals[sector_t::ceiling] == 0)
		{
			src.kind = LOD_CEILING;
			src.fixed = false;
			bakePlane(sec, sec->ceilingplane, sec->iboindex[sector_t::ceiling], LOD_CEILING, false, sec, sector_t::ceiling, sec->GetTexture(sector_t::ceiling),
				src, FSection::DONTRENDERCEILING);
		}

		// Opaque 3D floors: the top seen from above, the bottom seen from below, within this sector's height (see HWFlat::ProcessSector).
		auto &ffloors = sec->e->XFloor.ffloors;
		const double secFloor = sec->CenterFloor(), secCeiling = sec->CenterCeiling();
		for (unsigned k = 0; k < ffloors.Size(); k++)
		{
			F3DFloor *rover = ffloors[k];
			if ((rover->flags & (FF_EXISTS | FF_RENDERPLANES)) != (FF_EXISTS | FF_RENDERPLANES)) continue;
			if ((rover->flags & (FF_TRANSLUCENT | FF_ADDITIVETRANS | FF_FOG)) || rover->alpha < 255) continue;
			for (int bottom = 0; bottom < 2; bottom++)
			{
				F3DFloor::planeref &pr = bottom ? rover->bottom : rover->top;
				if (pr.copied) continue;
				const double z = pr.plane->ZatPoint(sec->centerspot);
				if (bottom ? !(z >= secFloor && z < secCeiling) : !(z > secFloor && z <= secCeiling)) continue;
				src.kind = LOD_3D;
				src.fixed = false;
				src.ffloor = k;
				curDeps.Clear();
				addDep(sec);
				addDep(rover->model);
				addDep(pr.model);
				src.bottom = !!bottom;
				bakePlane(sec, *pr.plane, pr.vindex, LOD_3D, !bottom, pr.model, pr.isceiling, *pr.texture, src, 0);
			}
		}
	}

	// Colour model: every texture's average colour, one texel each in the colour texture.
	auto img = new FLodPaletteImage;
	TArray<int> materialColor;		// per material: its texel, -1 for none (masked)
	materialColor.Resize(materials.Size());
	{
		TMap<FGameTexture *, PalEntry> average;
		TArray<PalEntry> colors;
		for (unsigned i = 0; i < materials.Size(); i++)
		{
			FGameTexture *tex = materials[i].texture;
			if (!average.CheckKey(tex))
			{
				FBitmap bmp = tex->GetTexture()->GetBgraBitmap(nullptr);
				average[tex] = bmp.GetPixels() ? averageColor((const uint32_t *)bmp.GetPixels(), bmp.GetWidth() * bmp.GetHeight(), 0) : PalEntry(255, 128, 128, 128);
			}
		}
		// Exact colours if they fit, else 4 bits per channel (which always fits).
		for (int precision = 8; precision >= 4; precision -= 4)
		{
			colors.Clear();
			TMap<uint32_t, int> index;
			const uint32_t mask = precision == 8 ? 0xffffff : 0xf0f0f0;
			bool fits = true;
			for (unsigned i = 0; i < materials.Size() && fits; i++)
			{
				materialColor[i] = -1;
				if (materials[i].masked) continue;
				const uint32_t c = (uint32_t)average[materials[i].texture] & mask;
				if (int *found = index.CheckKey(c)) { materialColor[i] = *found; continue; }
				if (colors.Size() >= LOD_PALETTE_SIZE * LOD_PALETTE_SIZE) { fits = false; break; }
				materialColor[i] = index[c] = colors.Push(PalEntry(255, (c >> 16) & 255, (c >> 8) & 255, c & 255));
			}
			if (fits) break;
		}
		for (unsigned i = 0; i < colors.Size(); i++)
		{
			uint8_t *p = &img->bgra[i * 4];
			p[0] = colors[i].b; p[1] = colors[i].g; p[2] = colors[i].r; p[3] = 255;
		}
		numColors = colors.Size();
	}
	auto texelU = [](int i) { return ((i % LOD_PALETTE_SIZE) + 0.5f) / (float)LOD_PALETTE_SIZE; };
	auto texelV = [](int i) { return ((i / LOD_PALETTE_SIZE) + 0.5f) / (float)LOD_PALETTE_SIZE; };

	// Chunk order: by Morton key, so that chunks near each other on the map are near each other in the buffer and a
	// group's chunks in view tend to be one run.
	TArray<int> chunkRank;
	{
		TArray<std::pair<uint64_t, int>> keys;
		TMap<uint64_t, int>::Iterator it(chunkMap);
		TMap<uint64_t, int>::Pair *pair;
		while (it.NextPair(pair))
		{
			const uint32_t cx = (uint32_t)(pair->Key >> 32) ^ 0x80000000u, cy = (uint32_t)pair->Key ^ 0x80000000u;
			uint64_t morton = 0;
			for (int b = 0; b < 32; b++) morton |= (uint64_t((cx >> b) & 1) << (2 * b + 1)) | (uint64_t((cy >> b) & 1) << (2 * b));
			keys.Push({ morton, pair->Value });
		}
		std::sort(keys.begin(), keys.end());
		chunkRank.Resize(chunks.Size());
		for (unsigned i = 0; i < keys.Size(); i++) chunkRank[keys[i].second] = i;
	}

	TArray<FModelVertex> vertices;
	vertices.Resize(temp.Size() * 2);
	unsigned v = 0;
	auto addRange = [&](FLodGroup &g, int chunk, unsigned count)
	{
		if (g.numRanges == 0 || ranges.Last().chunk != chunk)
		{
			ranges.Push({ chunk, v, 0 });
			g.numRanges++;
		}
		ranges.Last().count += count;
		v += count;
	};

	entryTex.Resize(entries.Size());
	entryColor.Resize(entries.Size());
	entryCount.Resize(entries.Size());
	entryTexel.Resize(entries.Size());
	for (unsigned i = 0; i < entries.Size(); i++)
	{
		entryTex[i] = entryColor[i] = ~0u;
		entryCount[i] = entries[i].count;
		entryTexel[i] = materialColor[entries[i].material];
	}

	// Textured model: by material (opaque first, masked last), then chunk.
	{
		TArray<int> order;
		order.Resize(entries.Size());
		for (unsigned i = 0; i < order.Size(); i++) order[i] = i;
		std::sort(order.begin(), order.end(), [&](int ia, int ib)
		{
			const Entry &a = entries[ia], &b = entries[ib];
			const bool ma = materials[a.material].masked, mb = materials[b.material].masked;
			if (ma != mb) return mb;
			if (a.material != b.material) return a.material < b.material;
			return chunkRank[a.chunk] < chunkRank[b.chunk];
		});
		for (int i : order)
		{
			const Entry &e = entries[i];
			const FLodMaterial &m = materials[e.material];
			if (texGroups.Size() == 0 || texGroups.Last().material != e.material)
			{
				FLodGroup g;
				memset((void *)&g, 0, sizeof(g));
				g.material = e.material;
				g.slot = m.slot;
				g.firstRange = ranges.Size();
				g.masked = m.masked;
				g.wall = m.kind == LOD_WALL;
				g.lodonly = m.lodonly;
				texGroups.Push(g);
			}
			entryTex[i] = v;
			memcpy(&vertices[v], &temp[e.first], e.count * sizeof(FModelVertex));
			addRange(texGroups.Last(), e.chunk, e.count);
		}
	}
	const unsigned numTexVertices = v;

	// Colour model: by light as snapshotted (walls and flats alike), then chunk. Masked walls are left out.
	{
		TArray<int> lightKey;		// per entry: its light slot, -1 if not in the colour model
		lightKey.Resize(entries.Size());
		for (unsigned i = 0; i < entries.Size(); i++)
		{
			lightKey[i] = materialColor[entries[i].material] < 0 ? -1 : materials[entries[i].material].slot;
		}
		TArray<int> order;
		for (unsigned i = 0; i < entries.Size(); i++) if (lightKey[i] >= 0) order.Push(i);
		std::sort(order.begin(), order.end(), [&](int a, int b)
		{
			if (lightKey[a] != lightKey[b]) return lightKey[a] < lightKey[b];
			return chunkRank[entries[a].chunk] < chunkRank[entries[b].chunk];
		});
		int cur = -1;
		for (int i : order)
		{
			const Entry &e = entries[i];
			if (cur != lightKey[i])
			{
				cur = lightKey[i];
				FLodGroup g;
				memset((void *)&g, 0, sizeof(g));
				g.material = -1;
				g.slot = cur;
				g.firstRange = ranges.Size();
				colorGroups.Push(g);
			}
			auto &g = colorGroups.Last();
			entryColor[i] = v;
			const int texel = materialColor[e.material];
			const float u = texelU(texel), tv = texelV(texel);
			for (unsigned k = 0; k < e.count; k++)
			{
				FModelVertex mv = temp[e.first + k];
				mv.u = u;
				mv.v = tv;
				vertices[v + k] = mv;
			}
			addRange(g, e.chunk, e.count);
		}
	}
	numVertices = v;

	// Moving sectors: who depends on whom, and where everything was.
	{
		entryDepFirst.Resize(entries.Size());
		entryDepCount.Resize(entries.Size());
		depStart.Resize(numSectors + 1);
		for (auto &d : depStart) d = 0;
		for (unsigned i = 0; i < entries.Size(); i++)
		{
			entryDepFirst[i] = entries[i].depFirst;
			entryDepCount[i] = entries[i].depCount;
			for (unsigned d = 0; d < entries[i].depCount; d++) depStart[entryDeps[entries[i].depFirst + d] + 1]++;
		}
		for (unsigned s = 0; s < numSectors; s++)
		{
			if (depStart[s + 1] > 0) watched.Push(s);
			depStart[s + 1] += depStart[s];
		}
		depEntries.Resize(depStart[numSectors]);
		TArray<int> fill;
		fill.Resize(numSectors);
		for (unsigned s = 0; s < numSectors; s++) fill[s] = depStart[s];
		for (unsigned i = 0; i < entries.Size(); i++)
		{
			for (unsigned d = 0; d < entries[i].depCount; d++) depEntries[fill[entryDeps[entries[i].depFirst + d]]++] = i;
		}
		planes.Resize(numSectors * 4);
		for (unsigned s = 0; s < numSectors; s++)
		{
			const sector_t *sec = &Level->sectors[s];
			planes[s * 4 + 0] = sec->floorplane.fD();
			planes[s * 4 + 1] = sec->floorplane.Normal().Z;
			planes[s * 4 + 2] = sec->ceilingplane.fD();
			planes[s * 4 + 3] = sec->ceilingplane.Normal().Z;
		}
		sectorMoved.Resize(numSectors);
		for (auto &m : sectorMoved) m = 0;
		entryHidden.Resize(entries.Size());
		for (auto &h : entryHidden) h = 0;
		texVerts.Resize(numTexVertices);
		if (numTexVertices > 0) memcpy(texVerts.Data(), vertices.Data(), numTexVertices * sizeof(FModelVertex));
	}

	// lodonly subtrees: a subsector is lodonly if its sector (and the sector its planes are drawn from) is and it holds
	// no polyobject; a node if both its children are.
	for (unsigned s = 0; s < numSectors; s++) lodOnlySectors += LodOnly(&Level->sectors[s]);
	if (lodOnlySectors > 0 && Level->nodes.Size() > 0)
	{
		subLodOnly.Resize(Level->subsectors.Size());
		for (unsigned i = 0; i < Level->subsectors.Size(); i++)
		{
			const subsector_t &sub = Level->subsectors[i];
			subLodOnly[i] = sub.polys == nullptr && LodOnly(sub.sector) && (sub.render_sector == nullptr || LodOnly(sub.render_sector));
		}
		nodeLodOnly.Resize(Level->nodes.Size());
		for (auto &n : nodeLodOnly) n = 0;
		auto childLodOnly = [&](void *child) -> bool
		{
			if ((size_t)child & 1) return subLodOnly[((subsector_t *)((uint8_t *)child - 1))->Index()];
			return nodeLodOnly[((node_t *)child)->Index()];
		};
		// Children before parents.
		TArray<int> stack;
		TArray<uint8_t> expanded;
		expanded.Resize(Level->nodes.Size());
		for (auto &e : expanded) e = 0;
		stack.Push(Level->HeadNode()->Index());
		while (stack.Size())
		{
			const int n = stack.Last();
			node_t *node = &Level->nodes[n];
			if (!expanded[n])
			{
				expanded[n] = 1;
				for (int i = 0; i < 2; i++) if (!((size_t)node->children[i] & 1)) stack.Push(((node_t *)node->children[i])->Index());
			}
			else
			{
				stack.Pop();
				nodeLodOnly[n] = childLodOnly(node->children[0]) && childLodOnly(node->children[1]);
				lodOnlyNodes += nodeLodOnly[n];
			}
		}
	}

	if (numVertices > 0)
	{
		vb = new FModelVertexBuffer(false, true);
		FModelVertex *out = vb->LockVertexBuffer(numVertices);
		memcpy(out, vertices.Data(), numVertices * sizeof(FModelVertex));
		vb->UnlockVertexBuffer();
	}
	if (colorGroups.Size() > 0)
	{
		// A new texture per build: the texture manager owns it (a few kilobytes per level loaded).
		palette = MakeGameTexture(new FImageTexture(img), nullptr, ETextureType::Override);
		palette->SetUpscaleFlag(0, true);
		palette->SetNoMipmap(true);
		TexMan.AddGameTexture(palette, false);
	}
	else delete img;
	lodBuildMS = (I_nsTime() - startTime) / 1e6;
}

//==========================================================================
//
// Moving sectors, once per game tic: hide the pieces depending on a sector
// whose floor or ceiling has left the height it was snapshotted at, and put
// them back when it returns.
//
//==========================================================================

void FWorldLOD::CheckMoved(FLevelLocals *Level)
{
	if (vb == nullptr) return;
	IVertexBuffer *buf = vb->vertexBuffer();
	static TArray<FModelVertex> scratch;
	for (int s : watched)
	{
		const sector_t *sec = &Level->sectors[s];
		const double *p = &planes[s * 4];
		const bool moved = sec->floorplane.fD() != p[0] || sec->floorplane.Normal().Z != p[1]
			|| sec->ceilingplane.fD() != p[2] || sec->ceilingplane.Normal().Z != p[3];
		if (moved == !!sectorMoved[s]) continue;
		sectorMoved[s] = moved;
		movedSectors += moved ? 1 : -1;
		for (int k = depStart[s]; k < depStart[s + 1]; k++)
		{
			const int e = depEntries[k];
			bool hide = false;
			for (unsigned d = 0; d < entryDepCount[e] && !hide; d++) hide = !!sectorMoved[entryDeps[entryDepFirst[e] + d]];
			if (hide == !!entryHidden[e]) continue;
			entryHidden[e] = hide;
			hiddenPieces += hide ? 1 : -1;
			const unsigned n = entryCount[e];
			const size_t bytes = n * sizeof(FModelVertex);
			if (hide)
			{
				// All vertices at one point: the triangles have no area and draw nothing.
				scratch.Resize(n);
				memset((void *)scratch.Data(), 0, bytes);
				if (entryTex[e] != ~0u) buf->SetSubData(entryTex[e] * sizeof(FModelVertex), bytes, scratch.Data());
				if (entryColor[e] != ~0u) buf->SetSubData(entryColor[e] * sizeof(FModelVertex), bytes, scratch.Data());
			}
			else if (entryTex[e] != ~0u)
			{
				buf->SetSubData(entryTex[e] * sizeof(FModelVertex), bytes, &texVerts[entryTex[e]]);
				if (entryColor[e] != ~0u && entryTexel[e] >= 0)
				{
					scratch.Resize(n);
					const float u = ((entryTexel[e] % LOD_PALETTE_SIZE) + 0.5f) / (float)LOD_PALETTE_SIZE;
					const float tv = ((entryTexel[e] / LOD_PALETTE_SIZE) + 0.5f) / (float)LOD_PALETTE_SIZE;
					for (unsigned j = 0; j < n; j++)
					{
						scratch[j] = texVerts[entryTex[e] + j];
						scratch[j].u = u;
						scratch[j].v = tv;
					}
					buf->SetSubData(entryColor[e] * sizeof(FModelVertex), bytes, scratch.Data());
				}
			}
		}
	}
}

// Take a new snapshot of the whole map before the next frame (console: worldlod_rebuild, ZScript: Level.RebuildWorldLOD()).
static void RebuildWorldLOD(FLevelLocals *Level)
{
	if (Level->WorldLOD) Level->WorldLOD->RequestRebuild();
}

CCMD(worldlod_rebuild)
{
	for (auto Level : AllLevels()) RebuildWorldLOD(Level);
}

DEFINE_ACTION_FUNCTION(FLevelLocals, RebuildWorldLOD)
{
	PARAM_SELF_STRUCT_PROLOGUE(FLevelLocals);
	RebuildWorldLOD(self);
	return 0;
}

//==========================================================================
//
// Per frame
//
//==========================================================================

void UpdateWorldLOD(FLevelLocals *Level)
{
	if (!gl_worldlod || r_distance_cull_type <= 0 || Level->sectors.Size() == 0)
	{
		// Off: free it (it's snapshotted again when turned back on). Nothing is drawing from it this frame yet.
		if (Level->WorldLOD)
		{
			DeleteWorldLOD(Level->WorldLOD);
			Level->WorldLOD = nullptr;
		}
		return;
	}
	FWorldLOD *lod = Level->WorldLOD;
	if (lod == nullptr) lod = Level->WorldLOD = new FWorldLOD;
	if (!lod->Matches(Level))
	{
		// Wait for the level's scripts to have run a couple of tics, so that the snapshot has the light they set up
		// (light is grouped as it is then).
		if (Level->maptime < 2) return;
		lod->Build(Level);
	}
	if (lod->checkedTic != Level->maptime)
	{
		lod->checkedTic = Level->maptime;
		lod->CheckMoved(Level);
	}
	lod->frame++;
}

// The main view of a perspective camera, with a line cull distance: the line cull's edge shows the model.
void HWDrawInfo::SetupWorldLOD()
{
	worldLOD = nullptr;
	lodOnly = false;
	worldLODBackdrop = false;
	FWorldLOD *lod = Level->WorldLOD;
	if (gl_worldlod && r_distance_cull_type > 0 && Viewpoint.culldistsq > 0 && lod && lod->built && lod->vb && lod->Matches(Level)
		&& outer == nullptr && mCurrentPortal == nullptr && !Viewpoint.bDoOob && !Viewpoint.bDoOrtho && !isNoLineCullMode())
	{
		worldLOD = lod;
		lodOnly = lod->lodOnlyMode && lod->lodOnlySectors > 0;
		AddWorldLODBackdrop();
	}
}

// Is this BSP child (node or subsector) entirely in lodonly sectors, drawn by the world LOD model?
bool HWDrawInfo::LodOnlyChild(void *child) const
{
	if (!lodOnly) return false;
	const FWorldLOD *lod = worldLOD;
	if ((size_t)child & 1)
	{
		const unsigned i = ((subsector_t *)((uint8_t *)child - 1))->Index();
		return i < lod->subLodOnly.Size() && lod->subLodOnly[i];
	}
	const unsigned i = ((node_t *)child)->Index();
	return i < lod->nodeLodOnly.Size() && lod->nodeLodOnly[i];
}

// The sky behind the world LOD: one box of sky walls around the camera, drawn at the far plane (HWSkyPortal with
// backdrop set), with caps above and below. It only shows where nothing else is drawn, so it is safe everywhere,
// and lines the walk skips (lodonly) or culls by distance need no sky walls of their own. Not for skybox skies.
void HWDrawInfo::AddWorldLODBackdrop()
{
	sector_t *sec = Viewpoint.sector;
	if (sec == nullptr) return;
	FSectorPortal *sportal = sec->ValidatePortal(sector_t::ceiling);
	if (sportal != nullptr && !(sportal->mFlags & PORTSF_INSKYBOX) && sportal->mType == PORTS_SKYVIEWPOINT) return;

	HWSkyInfo skyinfo;
	skyinfo.init(this, sec, sector_t::ceiling, sec->skytransfer, sec->Colormap.FadeColor);
	skyinfo.backdrop = true;
	HWWallDispatcher disp(this);
	const float R = 8192.f, cx = (float)Viewpoint.Pos.X, cy = (float)Viewpoint.Pos.Y;
	const float xs[5] = { cx - R, cx + R, cx + R, cx - R, cx - R };
	const float ys[5] = { cy - R, cy - R, cy + R, cy + R, cy - R };
	for (int i = 0; i < 4; i++)
	{
		HWWall wall;
		memset((void *)&wall, 0, sizeof(wall));
		wall.glseg.x1 = xs[i]; wall.glseg.y1 = ys[i];
		wall.glseg.x2 = xs[i + 1]; wall.glseg.y2 = ys[i + 1];
		wall.glseg.fracleft = 0; wall.glseg.fracright = 1;
		wall.ztop[0] = wall.ztop[1] = 32768.f;
		wall.zbottom[0] = wall.zbottom[1] = -32768.f;
		wall.flags = HWWall::HWF_NOSPLIT;
		wall.Colormap = sec->Colormap;
		wall.sky = &skyinfo;
		wall.MakeVertices(true);
		// The first two walls also turn on the portal's caps (above and below the box).
		wall.PutPortal(&disp, PORTALTYPE_SKY, i == 0 ? sector_t::ceiling : i == 1 ? sector_t::floor : -1);
	}
	worldLODBackdrop = true;
}


// pass 0: opaque (textured model, then the colour model), 1: masked (textured model only; alpha test already set up by RenderScene).
void HWDrawInfo::DrawWorldLOD(FRenderState &state, int pass)
{
	FWorldLOD *lod = worldLOD;
	if (lod == nullptr || lod->vb == nullptr) return;
	lodDrawTime.Clock();

	const DVector2 eye = Viewpoint.Pos.XY();
	const double D2 = Viewpoint.culldistsq;
	const double colorMul = gl_worldlod_colordist;
	const bool colorModel = colorMul > 1. && lod->palette != nullptr && lod->colorGroups.Size() > 0;
	const double C2 = colorModel ? D2 * colorMul * colorMul : 0.;	// where the colour model takes over (squared)
	if (lod->checkedFrame != lod->frame)
	{
		// Chunks in view, by model: each draws its part of the chunk between its distances.
		lod->checkedFrame = lod->frame;
		lodStatTexChunks = lodStatColorChunks = lodStatTexDraws = lodStatColorDraws = 0;
		for (auto &c : lod->chunks)
		{
			const double fx = max(fabs(c.x1 - eye.X), fabs(c.x2 - eye.X)), fy = max(fabs(c.y1 - eye.Y), fabs(c.y2 - eye.Y));
			const double nx = eye.X < c.x1 ? c.x1 - eye.X : eye.X > c.x2 ? eye.X - c.x2 : 0.;
			const double ny = eye.Y < c.y1 ? c.y1 - eye.Y : eye.Y > c.y2 ? eye.Y - c.y2 : 0.;
			const double farthest = fx * fx + fy * fy, nearest = nx * nx + ny * ny;
			c.texVisible = c.colorVisible = c.nearVisible = false;
			if (!Frustum.MapBoxVisible(c.x1, c.x2, c.y1, c.y2, c.z1, c.z2)) continue;
			c.nearVisible = !colorModel || nearest < C2;
			if (farthest <= D2) continue;
			c.texVisible = c.nearVisible;
			c.colorVisible = colorModel && farthest > C2;
			lodStatTexChunks += c.texVisible;
			lodStatColorChunks += c.colorVisible;
		}
	}

	// Light, colour and fog of every group, looked up again every gl_worldlod_envupdatetic tics (0: every frame).
	const int envEvery = max<int>(0, gl_worldlod_envupdatetic);
	const int tic = Level->maptime;
	if (pass == 0 && (envEvery == 0 || lod->envTic == INT_MIN || tic - lod->envTic >= envEvery || tic < lod->envTic))
	{
		lod->envTic = tic;
		lodStatEnvUpdates++;
		const uint64_t envStart = I_nsTime();
		for (auto &slot : lod->slots)
		{
			if (slot.source.fixed || !LookupLight(Level, slot.source, slot.current)) slot.current = slot.light;
		}
		lodEnvMS = (I_nsTime() - envStart) / 1e6;
	}

	const bool fullbright = isFullbrightScene();
	const int extra = getExtraLight();
	state.SetVertexBuffer(lod->vb->vertexBuffer(), 0, 0);
	state.SetInterpolationFactor(0.f);
	state.SetBoneIndexBase(-1);
	state.SetDynLight(0, 0, 0);
	state.SetLightIndex(-1);
	state.EnableTextureMatrix(false);
	state.SetDepthBias(1.f, gl_worldlod_bias);
	// Only the front of a surface is ever drawn, as the BSP's facing tests do for walls and flats.
	state.SetCulling(portalState.isMirrored() ? Cull_CCW : Cull_CW);

	auto setLight = [&](const FLodGroup &g)
	{
		const FLodLight &L = lod->slots[g.slot].current;
		FColormap cm = L.colormap;
		if (fullbright) cm.Clear();
		SetFog(state, Level, lightmode, L.lightlevel, L.rellight + extra, fullbright, &cm, false);
		SetColor(state, Level, lightmode, L.lightlevel, L.rellight + extra, fullbright, cm, 1.f);
		state.SetObjectColor(L.objColor);
		state.SetObjectColor2(0);
		state.SetAddColor(L.addColor);
	};
	// Runs of consecutive chunks in view are one draw.
	auto drawGroup = [&](const FLodGroup &g, bool color, const std::function<void()> &setup) -> int	// color: colour model
	{
		int draws = 0;
		bool set = false;
		unsigned runFirst = 0, runCount = 0;
		auto flush = [&]()
		{
			if (runCount == 0) return;
			if (!set)
			{
				set = true;
				setup();
				setLight(g);
			}
			state.Draw(DT_Triangles, runFirst, runCount);
			vertexcount += runCount;
			draws++;
			runCount = 0;
		};
		for (unsigned r = 0; r < g.numRanges; r++)
		{
			const FLodRange &range = lod->ranges[g.firstRange + r];
			const FLodChunk &c = lod->chunks[range.chunk];
			if (!(color ? c.colorVisible : g.lodonly ? c.nearVisible : c.texVisible)) { flush(); continue; }
			if (runCount == 0) runFirst = range.first;
			runCount += range.count;	// ranges of a group are consecutive in the buffer
		}
		flush();
		return draws;
	};

	// Textured model: from the cull distance to where the colour model takes over.
	// lodonly groups have no inner limit (nothing else draws them) and no depth bias (nothing to give way to).
	int cutoffMode = -1;
	for (auto &g : lod->texGroups)
	{
		if ((g.masked ? 1 : 0) != pass) continue;
		if (g.wall ? !gl_render_walls : !gl_render_flats) continue;
		if (cutoffMode != (int)g.lodonly)
		{
			cutoffMode = g.lodonly;
			state.SetLodCutoff((float)eye.X, (float)eye.Y, g.lodonly ? 0.f : (float)D2, (float)C2);
			if (g.lodonly) state.ClearDepthBias();
			else state.SetDepthBias(1.f, gl_worldlod_bias);
		}
		FLodMaterial &m = lod->materials[g.material];
		lodStatTexDraws += drawGroup(g, false, [&]()
		{
			if (m.kind == LOD_WALL)
			{
				state.EnableTextureMatrix(false);
				state.SetMaterial(m.texture, UF_Texture, 0, m.clamp, NO_TRANSLATION, -1, nullptr);
				state.SetTextureClamp(m.m2s && (m.clamp & HWWall::HWF_CLAMPY) && !m.texture->isWarped());
				state.ApplyTextureManipulation(m.ext ? &m.fx : nullptr);
			}
			else
			{
				state.SetTextureClamp(false);
				state.SetMaterial(m.texture, UF_Texture, 0, CLAMP_NONE, NO_TRANSLATION, -1, nullptr);
				HWSectorPlane plane;
				plane.Offs = { m.offsX, m.offsY };
				plane.Scale = { m.scaleX, m.scaleY };
				plane.Angle = m.angle;
				state.EnableTextureMatrix(hw_SetPlaneTextureRotation(&plane, m.texture, state.mTextureMatrix));
				state.ApplyTextureManipulation(&m.fx);
			}
		});
	}

	// Colour model: past that, every surface in its texture's average colour.
	if (colorModel && pass == 0)
	{
		state.SetLodCutoff((float)eye.X, (float)eye.Y, (float)C2, 0.f);
		state.SetDepthBias(1.f, gl_worldlod_bias);
		bool materialSet = false;
		for (auto &g : lod->colorGroups)
		{
			lodStatColorDraws += drawGroup(g, true, [&]()
			{
				if (materialSet) return;
				materialSet = true;
				state.EnableTextureMatrix(false);
				state.SetTextureClamp(false);
				state.ApplyTextureManipulation(nullptr);
				state.SetMaterial(lod->palette, UF_None, 0, CLAMP_XY, NO_TRANSLATION, -1, nullptr);
			});
		}
	}

	state.SetLodCutoff(0, 0, 0, 0);
	state.ClearDepthBias();
	state.SetCulling(Cull_None);
	state.EnableTextureMatrix(false);
	state.SetTextureClamp(false);
	state.SetObjectColor(0xffffffff);
	state.SetObjectColor2(0);
	state.SetAddColor(0);
	state.ApplyTextureManipulation(nullptr);
	state.SetVertexBuffer(screen->mVertexData);
	lodDrawTime.Unclock();
}

void ResetWorldLODStats()
{
	lodDrawTime.Reset();
}

ADD_STAT(worldlod)
{
	FString out;
	FWorldLOD *lod = nullptr;
	for (auto Level : AllLevels()) if (Level->WorldLOD) { lod = Level->WorldLOD; break; }
	if (lod == nullptr || !lod->built)
	{
		out = gl_worldlod ? "World LOD: off (needs r_distance_cull_type > 0)" : "World LOD: off (gl_worldlod 0)";
		return out;
	}
	int liveSlots = 0;
	for (auto &slot : lod->slots) liveSlots += !slot.source.fixed;
	out.Format("World LOD: %d outdoor sectors (%d nolod, %d lodonly in %d skipped BSP nodes%s), %u vertices, %u chunks, built in %.0f ms | %d sectors moved, %d pieces hidden\n"
		"Textured: %u groups, %d draws in %d chunks | Colour: %u colours, %u groups, %d draws in %d chunks | %d surfaces with fixed light\n"
		"Drawing: %.2f ms | light, colour and fog looked up every %d tics, last lookup %.3f ms (%d since last shown); %d of %u light slots live",
		lod->outdoorSectors, lod->nolodSectors, lod->lodOnlySectors, lod->lodOnlyNodes, lod->lodOnlyMode ? "" : ", gl_worldlod_lodonly 0", lod->numVertices, lod->chunks.Size(), lodBuildMS, lod->movedSectors, lod->hiddenPieces,
		lod->texGroups.Size(), lodStatTexDraws, lodStatTexChunks, lod->numColors, lod->colorGroups.Size(), lodStatColorDraws, lodStatColorChunks,
		lod->fixedLights, lodDrawTime.TimeMS(), max<int>(0, gl_worldlod_envupdatetic), lodEnvMS, lodStatEnvUpdates,
		liveSlots, lod->slots.Size());
	lodStatEnvUpdates = 0;
	return out;
}

//==========================================================================
//
// worldlod_probe: where the biggest draw groups take their light from, and
// what that light is now. Run it at two times of day to see whether the
// world LOD follows.
//
//==========================================================================

CCMD(worldlod_probe)
{
	FWorldLOD *lod = nullptr;
	FLevelLocals *level = nullptr;
	for (auto Level : AllLevels()) if (Level->WorldLOD) { lod = Level->WorldLOD; level = Level; break; }
	if (lod == nullptr || !lod->built)
	{
		Printf("World LOD: not built\n");
		return;
	}
	static const char *const kinds[] = { "wall", "fill", "floor", "ceiling", "3D plane" };
	Printf("World LOD at tic %d (last light lookup at tic %d, every %d tics)\n", level->maptime, lod->envTic, max<int>(0, gl_worldlod_envupdatetic));
	for (int list = 0; list < 2; list++)
	{
		auto &groups = list == 0 ? lod->colorGroups : lod->texGroups;
		TArray<std::pair<unsigned, int>> bySize;
		for (unsigned i = 0; i < groups.Size(); i++)
		{
			unsigned n = 0;
			for (unsigned r = 0; r < groups[i].numRanges; r++) n += lod->ranges[groups[i].firstRange + r].count;
			bySize.Push({ n, (int)i });
		}
		std::sort(bySize.begin(), bySize.end(), [](auto &a, auto &b) { return a.first > b.first; });
		Printf("%s groups (biggest first):\n", list == 0 ? "Colour" : "Textured");
		for (unsigned k = 0; k < bySize.Size() && k < 6; k++)
		{
			const FLodGroup &g = groups[bySize[k].second];
			const FLodSlot &sl = lod->slots[g.slot];
			const FLodLightSource &src = sl.source;
			const int sector = src.kind == LOD_WALL ? ((unsigned)src.side < level->sides.Size() ? level->sides[src.side].sector->Index() : -1) : src.sector;
			sector_t *sec = (unsigned)sector < level->sectors.Size() ? &level->sectors[sector] : nullptr;
			Printf("  %u vertices, light slot %d, from %s of sector %d (tag %d)%s\n    built: light %d rel %d fade %06x density %d | used now: light %d rel %d fade %06x density %d",
				bySize[k].first, g.slot, kinds[src.kind], sector, sec && level->tagManager.CountSectorTags(sec) > 0 ? level->tagManager.GetSectorTag(sec, 0) : 0, src.fixed ? ", FIXED (snapshot)" : "",
				sl.light.lightlevel, sl.light.rellight, (uint32_t)sl.light.colormap.FadeColor & 0xffffff, sl.light.colormap.FogDensity,
				sl.current.lightlevel, sl.current.rellight, (uint32_t)sl.current.colormap.FadeColor & 0xffffff, sl.current.colormap.FogDensity);
			if (sec) Printf(" | sector now: light %d fade %06x density %d", sec->lightlevel, (uint32_t)sec->Colormap.FadeColor & 0xffffff, sec->Colormap.FogDensity);
			Printf("\n");
		}
	}
}
