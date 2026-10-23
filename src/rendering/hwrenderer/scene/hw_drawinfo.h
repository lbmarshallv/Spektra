/*
** hw_drawinfo.h
**
** Basic scene draw info management class
**
**---------------------------------------------------------------------------
**
** Copyright 2000-2018 Christoph Oelckers
** Copyright 2017-2025 GZDoom Maintainers and Contributors
** Copyright 2025-2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#pragma once

#include <atomic>
#include <functional>
#include "vectors.h"
#include "r_defs.h"
#include "r_utility.h"
#include "hw_viewpointuniforms.h"
#include "v_video.h"
#include "hw_weapon.h"
#include "hw_drawlist.h"
#include "hw_frustum.h"

struct FBspZBounds;	// hw_bsp.cpp
struct HWHorizon;	// hw_horizon.h
struct FTerrainBatches;	// hw_terrainbatch.cpp
struct FWorldLOD;	// hw_worldlod.cpp

// Outdoor: sky ceiling, or a closed sector next to one (hw_worldlod.cpp).
bool HWSectorOutdoor(const sector_t *sec);

// UDMF lodonly (and not nolod): drawn only by the world LOD model (hw_worldlod.cpp). These test the map flag only;
// the renderer checks HWDrawInfo::lodOnly (gl_worldlod_lodonly, world LOD active in this view) first.
inline bool HWSectorLodOnly(const sector_t *sec) { return sec && (sec->MoreFlags & (SECMF_LODONLY | SECMF_NOLOD)) == SECMF_LODONLY; }
inline bool HWLineLodOnly(const line_t *line) { return HWSectorLodOnly(line->frontsector) || HWSectorLodOnly(line->backsector); }

// Map cell of a point, for the terrain batches and the world LOD (cells of 'size' map units).
inline uint64_t HWCellKey(double x, double y, int size)
{
	const int cx = (int)floor(x / size), cy = (int)floor(y / size);
	return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy);
}

enum EDrawMode
{
	DM_MAINVIEW,
	DM_OFFSCREEN,
	DM_PORTAL,
	DM_SKYPORTAL
};

struct FSectorPortalGroup;
struct FLinePortalSpan;
struct FFlatVertex;
class HWWall;
class HWFlat;
class HWSprite;
struct HWDecal;
class IShadowMap;
struct particle_t;
struct FDynLightData;
struct HUDSprite;
class ACorona;
class Clipper;
class HWPortal;
class FFlatVertexBuffer;
class IRenderQueue;
class HWScenePortalBase;
class FRenderState;

//==========================================================================
//
// these are used to link faked planes due to missing textures to a sector
//
//==========================================================================
struct gl_subsectorrendernode
{
	gl_subsectorrendernode *	next;
	subsector_t *				sub;
	int							lightindex;
	int							vertexindex;
};

struct gl_floodrendernode
{
	gl_floodrendernode * next;
	seg_t *seg;
	int vertexindex;
	// This will use the light list of the originating sector.
};

enum area_t : int;

enum SectorRenderFlags
{
	// This is used to merge several subsectors into a single draw item
	SSRF_RENDERFLOOR = 1,
	SSRF_RENDERCEILING = 2,
	SSRF_RENDER3DPLANES = 4,
	SSRF_RENDERALL = 7,
	SSRF_PROCESSED = 8,
	SSRF_SEEN = 16,
	SSRF_PLANEHACK = 32,
	SSRF_FLOODHACK = 64
};

enum EPortalClip
{
	PClip_InFront,
	PClip_Inside,
	PClip_Behind,
};

enum DrawListType
{
	GLDL_PLAINWALLS,
	GLDL_PLAINFLATS,
	GLDL_MASKEDWALLS,
	GLDL_MASKEDFLATS,
	GLDL_MASKEDWALLSOFS,
	GLDL_MODELS,

	GLDL_TRANSLUCENT,
	GLDL_TRANSLUCENTBORDER,

	GLDL_TYPES,
};


// A plant close enough to the viewer to be drawn blended in the translucent pass (hw_foliage.cpp).
struct FFoliageTranslucentItem
{
	uint32_t instance;		// index into FFoliageSystem::Instances
	int lightIndex;			// dynamic light list, -1 = none
	float distSq;			// for back-to-front sorting
	uint8_t level;			// level of detail
	uint8_t fade;			// 0 = fully translucent ... FOLIAGE_FADE_STEPS = fully opaque
};

constexpr int FOLIAGE_FADE_STEPS = 15;

struct HWDrawInfo
{
	struct wallseg
	{
		float x1, y1, z1, x2, y2, z2;
	};

	struct MissingTextureInfo
	{
		seg_t * seg;
		subsector_t * sub;
		float Planez;
		float Planezfront;
	};

	struct MissingSegInfo
	{
		seg_t * seg;
		int MTI_Index;    // tells us which MissingTextureInfo represents this seg.
	};

	struct SubsectorHackInfo
	{
		subsector_t * sub;
		uint8_t flags;
	};

	enum EFullbrightFlags
	{
		Fullbright = 1,
		Nightvision = 2,
		StealthVision = 4
	};

	bool isFullbrightScene() const { return !!(FullbrightFlags & Fullbright); }
	bool isNightvision() const { return !!(FullbrightFlags & Nightvision); }
	bool isStealthVision() const { return !!(FullbrightFlags & StealthVision); }

	HWDrawList drawlists[GLDL_TYPES];
	int vpIndex;
	ELineCullMode linecullmode;
	ELightMode lightmode;

	FLevelLocals *Level;
	HWDrawInfo * outer = nullptr;
	int FullbrightFlags;
	std::atomic<int> spriteindex;
	HWPortal *mClipPortal;
	HWPortal *mCurrentPortal;
	//FRotator mAngles;
	Clipper *mClipper;
	Clipper *vClipper; // Vertical clipper
	Clipper *rClipper; // Radar clipper
	FRenderViewpoint Viewpoint;
	HWViewpointUniforms VPUniforms;	// per-viewpoint uniform state
	VSMatrix ProjectionMatrix2;
	HWFrustum Frustum;	// set by SetupView
	FBspZBounds *zBounds = nullptr;	// set by RenderBSP when BSP frustum culling is active for this view
	HWHorizon *horizon = nullptr;	// set by RenderBSP when horizon culling is active (main view only)
	bool clipOnly = false;	// AddLine only feeds the clipper (subsector hidden behind the horizon)
	FTerrainBatches *terrainBatches = nullptr;	// set by RenderBSP when this view draws batched outdoor floors
	FWorldLOD *worldLOD = nullptr;	// set by RenderBSP when the world LOD is drawn past the line cull distance (main view)
	bool lodOnly = false;			// this view: lodonly sectors are drawn by the world LOD, and the BSP walk skips them
	bool worldLODBackdrop = false;	// this view: one sky backdrop around the camera replaces the culled lines' own
	bool thingsOnly = false;
	int horizonHiddenCount = 0;	// subsectors hidden by the horizon in this view (decides whether it keeps running)	// walking a subtree hidden behind the horizon: actors, particles and foliage only
	float bspZPad = 0;
	bool frustum3D = false;	// test BSP subtrees and subsectors against the 3D frustum in this view
	TArray<HWPortal *> Portals;
	TArray<HWDecal *> Decals[2];	// the second slot is for mirrors which get rendered in a separate pass.
	TArray<HUDSprite> hudsprites;	// These may just be stored by value.
	TArray<ACorona*> Coronas;
	TArray<int> FoliageSubsectors;	// visible render subsectors that contain foliage (filled by the BSP walk)
	TArray<FFoliageTranslucentItem> FoliageTranslucent;	// near translucent plants, drawn in RenderTranslucent
	uint64_t LastFrameTime = 0;

	TArray<int> culledSectors;

	TArray<MissingTextureInfo> MissingUpperTextures;
	TArray<MissingTextureInfo> MissingLowerTextures;

	TArray<MissingSegInfo> MissingUpperSegs;
	TArray<MissingSegInfo> MissingLowerSegs;

	TArray<SubsectorHackInfo> SubsectorHacks;

	TMap<int, gl_subsectorrendernode*> otherFloorPlanes;
	TMap<int, gl_subsectorrendernode*> otherCeilingPlanes;
	TMap<int, gl_floodrendernode*> floodFloorSegs;
	TMap<int, gl_floodrendernode*> floodCeilingSegs;

	//TArray<sector_t *> CeilingStacks;
	//TArray<sector_t *> FloorStacks;

	TArray<subsector_t *> HandledSubsectors;

	TArray<uint8_t> section_renderflags;
	TArray<uint8_t> ss_renderflags;
	TArray<uint8_t> no_renderflags;

	// This is needed by the BSP traverser.
	BitArray CurrentMapSections;	// this cannot be a single number, because a group of portals with the same displacement may link different sections.
	area_t	in_area;
	fixed_t viewx, viewy;	// since the nodes are still fixed point, keeping the view position  also fixed point for node traversal is faster.
	bool multithread;

private:
	// For ProcessLowerMiniseg
	bool inview;
	subsector_t * viewsubsector;
	TArray<seg_t *> lowersegs;

	subsector_t *currentsubsector;	// used by the line processing code.
	sector_t *currentsector;

	void WorkerThread();

	void UnclipSubsector(subsector_t *sub);

	bool IsDistanceCulled(seg_t *line);
	void AddLine(seg_t *seg, bool portalclip);
	void AddCulledLine(seg_t* seg);
	void PolySubsector(subsector_t * sub);
	void RenderPolyBSPNode(void *node);
	void AddPolyobjs(subsector_t *sub);
	void AddLines(subsector_t * sub, sector_t * sector);
	void AddSpecialPortalLines(subsector_t * sub, sector_t * sector, linebase_t *line);
	public:
	void RenderThings(subsector_t * sub, sector_t * sector);
	void RenderParticles(subsector_t *sub, sector_t *front);
	void DoSubsector(subsector_t * sub);
	int SetupLightsForOtherPlane(subsector_t * sub, FDynLightData &lightdata, const secplane_t *plane);
	int CreateOtherPlaneVertices(subsector_t *sub, const secplane_t *plane);
	void DrawPSprite(HUDSprite *huds, FRenderState &state);
	WeaponLighting GetWeaponLighting(sector_t *viewsector, const DVector3 &pos, int cm, area_t in_area, const DVector3 &playerpos, bool weaponPureLightLevel);

	void PreparePlayerSprites2D(sector_t * viewsector, area_t in_area);
	void PreparePlayerSprites3D(sector_t * viewsector, area_t in_area);
public:

	void SetCameraPos(const DVector3 &pos)
	{
		VPUniforms.mCameraPos = { (float)pos.X, (float)pos.Z, (float)pos.Y, 0.f };
	}

	void SetClipHeight(float h, float d)
	{
		VPUniforms.mClipHeight = h;
		VPUniforms.mClipHeightDirection = d;
		VPUniforms.mClipLine.X = -1000001.f;
	}

	void SetClipLine(line_t *line)
	{
		VPUniforms.mClipLine = { (float)line->v1->fX(), (float)line->v1->fY(), (float)line->Delta().X, (float)line->Delta().Y };
		VPUniforms.mClipHeight = 0;
		VPUniforms.mClipHeightDirection = 0.f;
	}

	HWPortal * FindPortal(const void * src);
	void RenderBSPNode(void *node);
	bool ChildInFrustum(void *child, const float *bbox);
	void SetupTerrainBatches();
	void DrawTerrainBatches(FRenderState &state);
	bool FloorBatched(const sector_t *sec) const;
	bool SegBatched(const seg_t *seg) const;
	void SetupWorldLOD();
	bool LodOnlyChild(void *child) const;	// BSP child entirely in lodonly sectors (skipped by the walk)
	void AddWorldLODBackdrop();
	void DrawWorldLOD(FRenderState &state, int pass);	// 0 opaque, 1 masked
	bool HorizonHides(void *child, const float *bbox);
	bool HorizonChildHidden(void *child, const float *bbox);
	void HorizonAdd(subsector_t *sub);
	void RenderOrthoNoFog();
	void RenderBSP(void *node, bool drawpsprites);

	static HWDrawInfo *StartDrawInfo(FLevelLocals *lev, HWDrawInfo *parent, FRenderViewpoint &parentvp, HWViewpointUniforms *uniforms);
	void StartScene(FRenderViewpoint &parentvp, HWViewpointUniforms *uniforms);
	void ClearBuffers();
	HWDrawInfo *EndDrawInfo();
	void SetViewArea();
	int SetFullbrightFlags(player_t *player);

	void DrawScene(int drawmode);
	void CreateScene(bool drawpsprites);
	void RenderScene(FRenderState &state);
	void DrawFoliage(FRenderState &state);
	void DrawFoliageTranslucent(FRenderState &state);
	void RenderTranslucent(FRenderState &state);
	void RenderPortal(HWPortal *p, FRenderState &state, bool usestencil);
	void EndDrawScene(sector_t * viewsector, FRenderState &state);
	void DrawEndScene2D(sector_t * viewsector, FRenderState &state);
	void Set3DViewport(FRenderState &state);
	void ProcessScene(bool toscreen);

	bool DoOneSectorUpper(subsector_t * subsec, float planez, area_t in_area);
	bool DoOneSectorLower(subsector_t * subsec, float planez, area_t in_area);
	bool DoFakeBridge(subsector_t * subsec, float planez, area_t in_area);
	bool DoFakeCeilingBridge(subsector_t * subsec, float planez, area_t in_area);

	bool CheckAnchorFloor(subsector_t * sub);
	bool CollectSubsectorsFloor(subsector_t * sub, sector_t * anchor);
	bool CheckAnchorCeiling(subsector_t * sub);
	bool CollectSubsectorsCeiling(subsector_t * sub, sector_t * anchor);
	void CollectSectorStacksCeiling(subsector_t * sub, sector_t * anchor, area_t in_area);
	void CollectSectorStacksFloor(subsector_t * sub, sector_t * anchor, area_t in_area);

	void DispatchRenderHacks();
	void AddUpperMissingTexture(side_t * side, subsector_t *sub, float backheight);
	void AddLowerMissingTexture(side_t * side, subsector_t *sub, float backheight);
	void HandleMissingTextures(area_t in_area);
	void PrepareUnhandledMissingTextures();
	void PrepareUpperGap(seg_t * seg);
	void PrepareLowerGap(seg_t * seg);
	void CreateFloodPoly(wallseg * ws, FFlatVertex *vertices, float planez, sector_t * sec, bool ceiling);
	void CreateFloodStencilPoly(wallseg * ws, FFlatVertex *vertices);

	void AddHackedSubsector(subsector_t * sub);
	void HandleHackedSubsectors();
	void AddFloorStack(sector_t * sec);
	void AddCeilingStack(sector_t * sec);
	void ProcessSectorStacks(area_t in_area);

	void ProcessActorsInPortal(FLinePortalSpan *glport, area_t in_area);

	void AddOtherFloorPlane(int sector, gl_subsectorrendernode * node);
	void AddOtherCeilingPlane(int sector, gl_subsectorrendernode * node);

	void GetDynSpriteLight(AActor *self, float x, float y, float z, FSection *sec, int portalgroup, float *out);
	void GetDynSpriteLight(AActor *thing, particle_t *particle, float *out);

	void PreparePlayerSprites(sector_t * viewsector, area_t in_area);
	void PrepareTargeterSprites(double ticfrac);

	void UpdateCurrentMapSection();
	void SetViewMatrix(const FRotator &angles, float vx, float vy, float vz, bool mirror, bool planemirror);
	void SetupView(FRenderState &state, float vx, float vy, float vz, bool mirror, bool planemirror);
	angle_t FrustumAngle();

	void DrawDecals(FRenderState &state, TArray<HWDecal *> &decals);
	void DrawPlayerSprites(bool hudModelStep, FRenderState &state);
	void DrawCoronas(FRenderState& state);
	void DrawCorona(FRenderState& state, ACorona* corona, double dist);

	void SetDitherTransFlags(AActor* actor);

	void ProcessLowerMinisegs(TArray<seg_t *> &lowersegs);
	void AddSubsectorToPortal(FSectorPortalGroup *portal, subsector_t *sub);

	void AddWall(HWWall *w);
	void AddMirrorSurface(HWWall *w);
	void AddFlat(HWFlat *flat, bool fog);
	void AddSprite(HWSprite *sprite, bool translucent);


	HWDecal *AddDecal(bool onmirror);

	void SetFallbackLightMode()
	{
		lightmode = ELightMode::Doom;
	}

	bool isNoLineCullMode() const
	{
		return linecullmode == ELineCullMode::NoCulling;
	}

	bool isOutDoorLineCullMode() const
	{
		return linecullmode == ELineCullMode::OutDoorCulling;
	}

	bool isGlobalLineCullMode() const
	{
		return linecullmode == ELineCullMode::OutDoorCulling;
	}


};

void CleanSWDrawer();
sector_t* RenderViewpoint(FRenderViewpoint& mainvp, AActor* camera, IntRect* bounds, float fov, float ratio, float fovratio, bool mainview, bool toscreen);
void WriteSavePic(player_t* player, FileWriter* file, int width, int height);
sector_t* RenderView(player_t* player);


inline bool isSoftwareLighting(ELightMode lightmode)
{
	return lightmode == ELightMode::ZDoomSoftware || lightmode == ELightMode::DoomSoftware || lightmode == ELightMode::Build;
}

inline bool isBuildSoftwareLighting(ELightMode lightmode)
{
	return lightmode == ELightMode::Build;
}

inline bool isDoomSoftwareLighting(ELightMode lightmode)
{
	return lightmode == ELightMode::ZDoomSoftware || lightmode == ELightMode::DoomSoftware;
}

inline bool isDarkLightMode(ELightMode lightmode)
{
	return lightmode == ELightMode::Doom || lightmode == ELightMode::DoomDark;
}

int CalcLightLevel(ELightMode lightmode, int lightlevel, int rellight, bool weapon, int blendfactor, bool weaponPureLightLevel = false);
PalEntry CalcLightColor(ELightMode lightmode, int light, PalEntry pe, int blendfactor);
float GetFogDensity(FLevelLocals* Level, ELightMode lightmode, int lightlevel, PalEntry fogcolor, int sectorfogdensity, int blendfactor);
bool CheckFog(FLevelLocals* Level, sector_t* frontsector, sector_t* backsector, ELightMode lightmode);
void SetColor(FRenderState& state, FLevelLocals* Level, ELightMode lightmode, int sectorlightlevel, int rellight, bool fullbright, const FColormap& cm, float alpha, bool weapon = false);
void SetShaderLight(FRenderState& state, FLevelLocals* Level, float level, float olight);
void SetFog(FRenderState& state, FLevelLocals* Level, ELightMode lightmode, int lightlevel, int rellight, bool fullbright, const FColormap* cmap, bool isadditive);
void ResetFoliageStats();

// Per-frame counts for the 'stat frustum' display (gl_frustum_exact / gl_frustum_cull).
struct FFrustumCullStats
{
	int nodes;			// BSP subtrees skipped because they are outside the view frustum
	int subsectors;		// single subsectors skipped for the same reason
	int horizonSubsectors;	// subsectors whose walls and flats are hidden behind hills / under ceilings
	int seamlessLines;	// two-sided lines skipped because both sides meet exactly (no wall, no clipping)
	int horizonNodes;		// whole subtrees hidden the same way (walked for actors only)
	bool horizonResting;	// skipped this frame: it wasn't hiding enough lately
	int models;			// actor models outside the view frustum
	int sprites;		// actor sprites outside the view frustum
	int plants;			// foliage outside the view frustum
	int unbounded;		// models that could not be tested (skinned, or not built yet)
	double yawHalfAngle;	// main view: half-width of the yaw range the clipper keeps, in degrees (360 = everything)
};
extern FFrustumCullStats frustumStats;
void ResetFrustumStats();
void UpdateBspZBounds(FLevelLocals *Level);
void UpdateTerrainBatches(FLevelLocals *Level);
void ResetTerrainStats();

// Per-frame counts for 'stat terrain' (hw_terrainbatch.cpp).
struct FTerrainStats
{
	int chunks;		// terrain chunks drawn as batches (main view)
	int draws;		// draw calls for them
	int segs;		// seamless lines between batched floors skipped by the walk
};
extern FTerrainStats terrainStats;


void UpdateWorldLOD(FLevelLocals *Level);	// hw_worldlod.cpp
void ResetWorldLODStats();
