/*
** hw_foliage.cpp
**
** Draws static foliage clusters (see p_foliage.cpp). The BSP walk records
** which visible subsectors contain plants; DrawFoliage then picks a level of
** detail per plant, groups plants that share a model, LOD level, sector
** lighting and dynamic light list, and draws each group with one instanced
** draw per model surface.
**
** Close to the viewer, plants also get per-pixel dynamic lights
** (FoliageLightRange, fading out towards its end), and translucent plants are blended and depth sorted
** in the translucent pass, fading smoothly to opaque across their
** FoliageTranslucentRange. Farther away they are drawn without dynamic
** lights and opaque, which is what keeps them cheap. r_foliagerangescale
** scales every class's ranges.
**
**---------------------------------------------------------------------------
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#include <algorithm>
#include "p_foliage.h"
#include "actor.h"
#include "info.h"
#include "g_levellocals.h"
#include "r_defs.h"
#include "a_dynlight.h"
#include "models.h"
#include "model.h"
#include "modelrenderer.h"
#include "texturemanager.h"
#include "v_video.h"
#include "c_cvars.h"
#include "c_dispatch.h"
#include "printf.h"
#include "stats.h"
#include "hw_material.h"
#include "hw_renderstate.h"
#include "hw_bonebuffer.h"
#include "hw_lightbuffer.h"
#include "hw_dynlightdata.h"
#include "hw_modelvertexbuffer.h"
#include "hw_cvars.h"
#include "hw_lighting.h"
#include "hwrenderer/scene/hw_drawinfo.h"
#include "hwrenderer/scene/hw_drawstructs.h"
#include "hwrenderer/scene/hw_portal.h"
#include "flatvertices.h"

EXTERN_CVAR(Float, r_lod)
EXTERN_CVAR(Float, r_lod_hysteresis)
EXTERN_CVAR(Int, r_distance_cull_type)
EXTERN_CVAR(Bool, r_foliageclusters)
EXTERN_CVAR(Float, gl_mask_sprite_threshold)
EXTERN_CVAR(Float, gl_mask_threshold)
CVARD(Bool, r_drawfoliage, true, 0, "Draw foliage clusters (debugging aid)")
CVARD(Float, r_foliagerangescale, 1.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Scales every foliage class's FoliageLightRange and FoliageTranslucentRange. 0 = no dynamic lights or translucency on foliage.")
CVARD(Float, r_foliagealphathreshold, 0.1f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Near translucent foliage: pixels with less alpha than this are skipped entirely, so they don't hide things drawn behind them later.")

static const unsigned MAX_INSTANCES_PER_DRAW = 4096;
static const int MAX_FOLIAGE_LODS = 16;
static const int MAX_LIGHTS_PER_PLANT = 32;
static const int LIGHT_FADE_STEPS = 16;	// dynamic light strength steps across the light fade band (shared light lists need a few distinct values)

//==========================================================================
//
// Per-frame statistics for 'stat foliage' and 'dumpfoliage'
//
//==========================================================================

static struct FFoliageStats
{
	int visibleSubsectors;
	int considered;			// plants in visible subsectors
	int drawn;
	int hiddenByLOD;		// LODModel "None" (or "Sprite", which foliage doesn't support yet)
	int culledByDistance;	// DistanceCheck / r_distance_cull_type
	int lit;				// plants drawn with dynamic lights
	int lightLists;			// distinct light lists uploaded
	int translucent;		// plants drawn blended in the translucent pass
	int groups;
	int instancedDraws;
	int singleDraws;		// groups of one, renderer without instancing, or bone buffer full
	bool instancing;
	cycle_t time;
	TArray<int> drawnPerTypeLevel;	// [type * (MAX_FOLIAGE_LODS + 1) + level], last frame
} foliageStats;

void ResetFoliageStats()
{
	foliageStats.visibleSubsectors = foliageStats.considered = foliageStats.drawn = foliageStats.hiddenByLOD = foliageStats.culledByDistance = 0;
	foliageStats.lit = foliageStats.lightLists = foliageStats.translucent = 0;
	foliageStats.groups = foliageStats.instancedDraws = foliageStats.singleDraws = 0;
	foliageStats.time.Reset();
	for (auto &c : foliageStats.drawnPerTypeLevel) c = 0;
}

//==========================================================================
//
// Model renderer for foliage: issues instanced draws and takes the actor
// class and light list for materials from its owner instead of an actor.
//
//==========================================================================

class FHWFoliageModelRenderer : public FModelRenderer
{
	HWDrawInfo *di;
	FRenderState &state;
	PClass *cls;
	int instances;
	int lightIndex;
	bool culled = false;

public:
	FHWFoliageModelRenderer(HWDrawInfo *d, FRenderState &st, PClass *c, int n, int light) : di(d), state(st), cls(c), instances(n), lightIndex(light) {}

	ModelRendererType GetType() const override { return GLModelRendererType; }

	void BeginDrawModel(FRenderStyle style, int smf_flags, const VSMatrix &objectToWorldMatrix, bool mirrored) override
	{
		state.SetDepthFunc(DF_LEqual);
		state.EnableTexture(true);
		// Same rule as FHWModelRenderer: translucent models cull back faces unless MODELDEF says otherwise.
		culled = (smf_flags & MDL_FORCECULLBACKFACES) || (!(style == DefaultRenderStyle()) && !(smf_flags & MDL_DONTCULLBACKFACES));
		if (culled) state.SetCulling(portalState.isMirrored() ? Cull_CCW : Cull_CW);
		state.mModelMatrix = objectToWorldMatrix;
		state.EnableModelMatrix(true);
	}

	void EndDrawModel(FRenderStyle style, int smf_flags) override
	{
		state.SetBoneIndexBase(-1);
		state.EnableModelMatrix(false);
		state.SetDepthFunc(DF_Less);
		if (culled) state.SetCulling(Cull_None);
	}

	IModelVertexBuffer *CreateVertexBuffer(bool needindex, bool singleframe) override
	{
		return new FModelVertexBuffer(needindex, singleframe);
	}

	VSMatrix GetViewToWorldMatrix() override
	{
		VSMatrix objectToWorldMatrix;
		di->VPUniforms.ViewMatrix.inverseMatrix(objectToWorldMatrix);
		return objectToWorldMatrix;
	}

	void BeginDrawHUDModel(FRenderStyle style, const VSMatrix &objectToWorldMatrix, bool mirrored, int smf_flags) override {}
	void EndDrawHUDModel(FRenderStyle style, int smf_flags) override {}

	void SetInterpolation(double inter) override
	{
		state.SetInterpolationFactor((float)inter);
	}

	void SetMaterial(FGameTexture *skin, bool clampNoFilter, FTranslationID translation, AActor *act) override
	{
		state.SetMaterial(skin, UF_Skin, 0, clampNoFilter ? CLAMP_NOFILTER : CLAMP_NONE, translation, -1, cls);
		state.SetLightIndex(lightIndex);
	}

	void DrawArrays(int start, int count) override
	{
		if (instances > 1) state.DrawInstanced(DT_Triangles, start, count, instances);
		else state.Draw(DT_Triangles, start, count);
	}

	void DrawElements(int numIndices, size_t offset) override
	{
		const int first = int(offset / sizeof(unsigned int));
		if (instances > 1) state.DrawIndexedInstanced(DT_Triangles, first, numIndices, instances);
		else state.DrawIndexed(DT_Triangles, first, numIndices);
	}

	void SetupFrame(FModel *model, unsigned int frame1, unsigned int frame2, unsigned int size, int boneStartIndex) override
	{
		auto mdbuff = static_cast<FModelVertexBuffer*>(model->GetVertexBuffer(GetType()));
		state.SetBoneIndexBase(boneStartIndex);
		if (mdbuff)
		{
			state.SetVertexBuffer(mdbuff->vertexBuffer(), frame1, frame2);
			if (mdbuff->indexBuffer()) state.SetIndexBuffer(mdbuff->indexBuffer());
		}
	}
};

//==========================================================================
//
// Lighting groups: plants in sectors with identical sprite lighting share
// draws. Compared bytewise, so always zero before filling in.
//
//==========================================================================

struct FFoliageLightKey
{
	FColormap colormap;
	uint32_t objectColor;
	uint32_t addColor;
	int lightlevel;
	int foglevel;
};

struct FFoliageGroup
{
	int type;
	int level;
	int lodModelID;			// replacement for model index 0, -1 = the MODELDEF model
	int lightGroup;
	int lightIndex;			// dynamic light list, -1 = none
	TArray<VSMatrix> matrices;	// pairs of (model matrix, normal matrix)
};

struct FFoliageTypeFrameInfo
{
	const LODmodel *levels[MAX_FOLIAGE_LODS];
	int numLevels;
	double maxDistSq;	// from the DistanceCheck property, -1 = none
	bool noHysteresis;
	double lightDistSq;	// FoliageLightRange (scaled), -1 = no dynamic lights
	double lightFadeStart;	// lights are at full strength closer than this
	double lightRange;
	double fadeStart;	// FoliageTranslucentRange (scaled)
	double fadeEnd;
	double fadeEndSq;	// -1 = never translucent
};

// A distinct set of dynamic lights touching one or more plants, uploaded once per frame and view.
struct FFoliageLightSet
{
	uint32_t hash;
	int group;
	int strength;			// 1..LIGHT_FADE_STEPS
	unsigned first, count;	// range in lightSetLights
	int bufferIndex;
};

// Scratch data reused from frame to frame. Only the main render thread draws.
static TArray<FFoliageLightKey> lightGroups;
static TArray<int> sectorLightGroup;
static TArray<uint32_t> sectorLightStamp;
static uint32_t lightStamp;
static TArray<FFoliageGroup> groups;
static unsigned numGroups;
static TMap<uint64_t, unsigned> groupMap;
static TArray<FFoliageTypeFrameInfo> typeInfo;
static TArray<VSMatrix> uploadChunk;
static TArray<VSMatrix> translucentRun;
static TArray<FDynamicLight *> candidateLights;
static TArray<uint32_t> plantStamps;		// per plant: the last DrawFoliage call that considered it
static uint32_t plantStampCounter;
static TArray<FDynamicLight *> lightSetLights;
static TArray<FFoliageLightSet> lightSets;
static FDynLightData plantLightData;

static void ResetLightGroups(FLevelLocals *Level)
{
	if (sectorLightStamp.Size() != Level->sectors.Size())
	{
		sectorLightStamp.Resize(Level->sectors.Size());
		sectorLightGroup.Resize(Level->sectors.Size());
		for (auto &s : sectorLightStamp) s = 0;
	}
	if (++lightStamp == 0) // wrapped: invalidate everything
	{
		for (auto &s : sectorLightStamp) s = 0;
		lightStamp = 1;
	}
	lightGroups.Clear();
}

static int GetLightGroup(HWDrawInfo *di, sector_t *sec)
{
	const unsigned index = sec->Index();
	if (sectorLightStamp[index] == lightStamp) return sectorLightGroup[index];

	FFoliageLightKey key;
	memset(&key, 0, sizeof(key));
	key.lightlevel = RescaleLightLevel(sec->GetSpriteLight());
	key.foglevel = clamp<int>(sec->lightlevel, 0, 255);
	if (di->isFullbrightScene())
	{
		key.colormap.Clear();
	}
	else
	{
		key.colormap = sec->Colormap;
		if (di->Level->flags3 & LEVEL3_NOCOLOREDSPRITELIGHTING) key.colormap.Decolorize();
	}
	PalEntry white = 0xffffffff;
	key.objectColor = white.Modulate(sec->SpecialColors[sector_t::sprites]).d;
	key.addColor = sec->AdditiveColors[sector_t::sprites].d | 0xff000000;

	int group = -1;
	for (unsigned i = 0; i < lightGroups.Size(); i++)
	{
		if (!memcmp(&lightGroups[i], &key, sizeof(key)))
		{
			group = i;
			break;
		}
	}
	if (group < 0) group = lightGroups.Push(key);

	sectorLightStamp[index] = lightStamp;
	sectorLightGroup[index] = group;
	return group;
}

static void ApplyLightGroup(HWDrawInfo *di, FRenderState &state, const FFoliageLightKey &key, float alpha, bool additivefog)
{
	const int rel = getExtraLight();
	const bool fullbright = di->isFullbrightScene();
	state.SetObjectColor(key.objectColor);
	state.SetAddColor(key.addColor);
	state.SetDynLight(0, 0, 0);
	SetColor(state, di->Level, di->lightmode, key.lightlevel, rel, fullbright, key.colormap, alpha);
	const int foglevel = key.colormap.FadeColor.isBlack() ? key.lightlevel : key.foglevel;
	SetFog(state, di->Level, di->lightmode, foglevel, rel, fullbright, &key.colormap, additivefog);
}

//==========================================================================
//
// Dynamic lights for plants near the viewer. Candidates come from the
// lights touching the plant's subsector section; a plant uses those whose
// radius reaches its bounding sphere. Plants touched by the same lights
// share one uploaded light list, so they can still be drawn together.
//
//==========================================================================

static void GatherCandidateLights(FLevelLocals *Level, subsector_t *sub)
{
	candidateLights.Clear();
	auto flatLightList = Level->lightlists.flat_dlist.CheckKey(sub->section);
	if (flatLightList == nullptr) return;

	TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Iterator it(*flatLightList);
	TMap<FDynamicLight *, std::unique_ptr<FLightNode>>::Pair *pair;
	while (it.NextPair(pair))
	{
		auto node = pair->Value.get();
		if (node == nullptr) continue;
		FDynamicLight *light = node->lightsource;
		if (light->ShouldLightActor(nullptr)) candidateLights.Push(light);	// skips DONTLIGHTACTORS / DONTLIGHTOTHERS lights
	}
}

static int GetPlantLightIndex(const FFoliageInstance &inst, const FFoliageType &type, int strength)
{
	FDynamicLight *touching[MAX_LIGHTS_PER_PLANT];
	int n = 0;
	const int group = inst.sector->PortalGroup;
	const double cx = inst.pos.X, cy = inst.pos.Y, cz = inst.pos.Z + type.centerHeight;
	for (FDynamicLight *light : candidateLights)
	{
		const DVector3 lpos = light->PosRelative(group);
		const double radius = light->GetRadius() + type.lightRadius;
		const double dx = lpos.X - cx, dy = lpos.Y - cy, dz = lpos.Z - cz;
		if (dx * dx + dy * dy + dz * dz < radius * radius && n < MAX_LIGHTS_PER_PLANT) touching[n++] = light;
	}
	if (n == 0) return -1;

	std::sort(touching, touching + n);
	uint32_t hash = (2166136261u ^ (uint32_t)group) * 16777619u ^ (uint32_t)strength;
	for (int i = 0; i < n; i++)
	{
		hash ^= (uint32_t)(uintptr_t)touching[i];
		hash *= 16777619u;
	}

	for (auto &set : lightSets)
	{
		if (set.hash == hash && set.group == group && set.strength == strength && set.count == (unsigned)n &&
			!memcmp(&lightSetLights[set.first], touching, n * sizeof(FDynamicLight *)))
		{
			return set.bufferIndex;
		}
	}

	plantLightData.Clear();
	for (int i = 0; i < n; i++) AddLightToList(plantLightData, group, touching[i], true);
	if (strength < LIGHT_FADE_STEPS)
	{
		// Scale every light's color (16 floats per light, color at 4..6) so the plant blends into its unlit look.
		const float f = strength / float(LIGHT_FADE_STEPS);
		for (auto &arr : plantLightData.arrays)
		{
			for (unsigned j = 0; j + 16 <= arr.Size(); j += 16)
			{
				arr[j + 4] *= f;
				arr[j + 5] *= f;
				arr[j + 6] *= f;
			}
		}
	}
	FFoliageLightSet set;
	set.hash = hash;
	set.group = group;
	set.strength = strength;
	set.first = lightSetLights.Size();
	set.count = n;
	for (int i = 0; i < n; i++) lightSetLights.Push(touching[i]);
	set.bufferIndex = screen->mLights->UploadLights(plantLightData);	// -1 if the light buffer is full: drawn unlit
	lightSets.Push(set);
	foliageStats.lightLists++;
	return set.bufferIndex;
}

//==========================================================================
//
//
//
//==========================================================================

static FFoliageGroup &FindGroup(int type, int level, int lightGroup, int lightIndex, int lodModelID)
{
	// type: 16 bits, level: 5 bits, light group: 16 bits, light index + 1: 27 bits
	const uint64_t key = (uint64_t(type) << 48) | (uint64_t(level) << 43) | (uint64_t(lightGroup & 0xffff) << 27) | uint64_t((lightIndex + 1) & 0x7ffffff);
	unsigned *found = groupMap.CheckKey(key);
	if (found) return groups[*found];

	if (numGroups == groups.Size()) groups.Push(FFoliageGroup());
	FFoliageGroup &g = groups[numGroups];
	g.type = type;
	g.level = level;
	g.lodModelID = lodModelID;
	g.lightGroup = lightGroup;
	g.lightIndex = lightIndex;
	g.matrices.Clear();
	groupMap[key] = numGroups++;
	return g;
}

//==========================================================================
//
// Draws one foliage type with the given LOD model. With instanceBase >= 0
// every draw is instanced; otherwise 'matrix' places a single plant.
//
//==========================================================================

static void DrawFoliageModel(HWDrawInfo *di, FRenderState &state, const FFoliageType &type, FRenderStyle style, int lodModelID, int lightIndex, const VSMatrix &matrix, int instanceBase, int instances)
{
	FHWFoliageModelRenderer renderer(di, state, type.cls, instances, lightIndex);
	FSpriteModelFrame *smf = type.smf;
	const FTranslationID translation = (type.smfFlags & MDL_IGNORETRANSLATION) ? NO_TRANSLATION : GetDefaultByType(type.cls)->Translation;

	renderer.BeginDrawModel(style, type.smfFlags, matrix, false);
	state.SetInstanceBase(instanceBase);
	for (unsigned i = 0; i < smf->modelsAmount; i++)
	{
		int modelid = smf->modelIDs[i];
		if (i == 0 && lodModelID >= 0) modelid = lodModelID;
		const int frame = smf->modelframes[i];
		if (modelid < 0 || modelid >= (int)Models.Size() || frame < 0) continue;

		FModel *mdl = Models[modelid];
		FGameTexture *tex = smf->skinIDs[i].isValid() ? TexMan.GetGameTexture(smf->skinIDs[i], true) : nullptr;
		const FTextureID *ssid = (i * MD3_MAX_SURFACES) < smf->surfaceskinIDs.Size() ? &smf->surfaceskinIDs[i * MD3_MAX_SURFACES] : nullptr;
		mdl->BuildVertexBuffer(&renderer);
		mdl->RenderFrame(&renderer, tex, frame, frame, -1.f, translation, ssid, -1, nullptr);
	}
	state.SetInstanceBase(-1);
	renderer.EndDrawModel(style, type.smfFlags);
}

// Draws 'count' plants whose matrix pairs are in 'matrices' starting at 'first', instanced if possible.
static void DrawFoliageBatch(HWDrawInfo *di, FRenderState &state, const FFoliageType &type, FRenderStyle style, int lodModelID, int lightIndex,
	const TArray<VSMatrix> &matrices, unsigned first, unsigned count, bool instancing)
{
	if (!instancing || count == 1)
	{
		for (unsigned i = 0; i < count; i++)
		{
			DrawFoliageModel(di, state, type, style, lodModelID, lightIndex, matrices[(first + i) * 2], -1, 1);
		}
		foliageStats.singleDraws += count;
		return;
	}

	const VSMatrix identity(0);
	for (unsigned start = 0; start < count; start += MAX_INSTANCES_PER_DRAW)
	{
		const unsigned num = min(MAX_INSTANCES_PER_DRAW, count - start);
		int base;
		if (first == 0 && start == 0 && num * 2 == matrices.Size())
		{
			base = screen->mBones->UploadBones(matrices);
		}
		else
		{
			uploadChunk.Resize(num * 2);
			memcpy(uploadChunk.Data(), &matrices[(first + start) * 2], num * 2 * sizeof(VSMatrix));
			base = screen->mBones->UploadBones(uploadChunk);
		}

		if (base >= 0)
		{
			DrawFoliageModel(di, state, type, style, lodModelID, lightIndex, identity, base, num);
			foliageStats.instancedDraws++;
		}
		else
		{
			// The bone buffer is full for this frame: draw these plants one at a time.
			for (unsigned i = 0; i < num; i++)
			{
				DrawFoliageModel(di, state, type, style, lodModelID, lightIndex, matrices[(first + start + i) * 2], -1, 1);
			}
			foliageStats.singleDraws += num;
		}
	}
}

//==========================================================================
//
// Picks the level of detail for one plant, with the same rules and
// hysteresis as AActor::SelectLOD.
//
//==========================================================================

static inline int PickFoliageLevel(const FFoliageTypeFrameInfo &ti, double distance, double hysteresis, uint8_t current)
{
	const double up = 1.0 + hysteresis * 0.5;
	const double down = 1.0 - hysteresis * 0.5;
	int level = min<int>(current, ti.numLevels);
	while (level < ti.numLevels && distance > ti.levels[level]->distance * up) level++;
	while (level > 0 && distance < ti.levels[level - 1]->distance * down) level--;
	return level;
}

static int LODModelForLevel(int type, int level)
{
	return level > 0 ? typeInfo[type].levels[level - 1]->modelID : -1;	// -1 (not found) falls back to the MODELDEF model
}

//==========================================================================
//
// Opaque pass: gathers every visible plant, sends near translucent plants
// to the translucent pass, and draws the rest grouped and instanced.
//
//==========================================================================

void HWDrawInfo::DrawFoliage(FRenderState &state)
{
	FFoliageSystem *foliage = Level->Foliage;
	if (foliage == nullptr || FoliageSubsectors.Size() == 0 || !r_drawfoliage) return;

	foliageStats.time.Clock();
	foliageStats.visibleSubsectors += FoliageSubsectors.Size();
	foliageStats.instancing = state.SupportsModelInstancing();
	const unsigned statStride = MAX_FOLIAGE_LODS + 1;
	if (foliageStats.drawnPerTypeLevel.Size() < foliage->Types.Size() * statStride)
	{
		const unsigned old = foliageStats.drawnPerTypeLevel.Size();
		foliageStats.drawnPerTypeLevel.Resize(foliage->Types.Size() * statStride);
		for (unsigned i = old; i < foliageStats.drawnPerTypeLevel.Size(); i++) foliageStats.drawnPerTypeLevel[i] = 0;
	}

	// per-frame setup
	ResetLightGroups(Level);
	numGroups = 0;
	groupMap.Clear();
	lightSets.Clear();
	lightSetLights.Clear();

	const double rangescale = clamp<double>(r_foliagerangescale, 0., 100.);
	const bool lightsEnabled = Level->HasDynamicLights && gl_light_sprites && !isFullbrightScene();

	typeInfo.Resize(foliage->Types.Size());
	for (unsigned t = 0; t < foliage->Types.Size(); t++)
	{
		auto &ti = typeInfo[t];
		auto cls = foliage->Types[t].cls;
		ti.numLevels = 0;
		for (const LODmodel *m = cls->ActorInfo()->LODmodels; m != nullptr && ti.numLevels < MAX_FOLIAGE_LODS; m = m->Next)
		{
			ti.levels[ti.numLevels++] = m;
		}
		FIntCVar *dc = cls->ActorInfo()->distancecheck;
		ti.maxDistSq = (dc != nullptr && **dc >= 0) ? double(**dc) * double(**dc) : -1.;
		ti.noHysteresis = !!(GetDefaultByType(cls)->perfFlags & PERF_NOLODHYSTERESIS);

		const FActorInfo *ai = cls->ActorInfo();
		const double lightRange = ai->FoliageLightRange * rangescale;
		ti.lightDistSq = (lightsEnabled && lightRange > 0) ? lightRange * lightRange : -1.;
		ti.lightRange = lightRange;
		ti.lightFadeStart = ai->FoliageLightFadeStart < 0 ? lightRange * 0.75 : min<double>(ai->FoliageLightFadeStart * rangescale, lightRange);
		ti.fadeStart = ai->FoliageFadeStart * rangescale;
		ti.fadeEnd = ai->FoliageFadeEnd * rangescale;
		ti.fadeEndSq = (foliage->Types[t].translucent && ti.fadeEnd > 0) ? ti.fadeEnd * ti.fadeEnd : -1.;
	}

	const DVector3 &viewpos = Viewpoint.bDoOob ? Viewpoint.OffPos : Viewpoint.Pos;
	const double lodscale = clamp<double>(r_lod, 0., 1000.);
	const double hysteresis = clamp<double>(r_lod_hysteresis, 0., 1.);
	const double culldistsq = (r_distance_cull_type > 0) ? Viewpoint.culldistsq : 0.;

	// A plant whose RenderRadius reaches several visible subsectors must only be drawn once per view.
	if (plantStamps.Size() < foliage->Instances.Size())
	{
		const unsigned old = plantStamps.Size();
		plantStamps.Resize(foliage->Instances.Size());
		for (unsigned i = old; i < plantStamps.Size(); i++) plantStamps[i] = 0;
	}
	if (++plantStampCounter == 0)
	{
		for (auto &s : plantStamps) s = 0;
		plantStampCounter = 1;
	}
	const uint32_t stamp = plantStampCounter;

	screen->mLights->Map();

	// Gather: one pass over the plants of every visible subsector.
	int candidateSub = -1;	// subsector whose dynamic lights are in candidateLights
	for (int sub : FoliageSubsectors)
	{
		const unsigned start = foliage->RefStart[sub];
		const unsigned end = foliage->RefStart[sub + 1];
		for (unsigned r = start; r < end; r++)
		{
			const unsigned i = foliage->Refs[r].instance;
			if (plantStamps[i] == stamp) continue;	// already reached through another subsector
			plantStamps[i] = stamp;

			FFoliageInstance &inst = foliage->Instances[i];
			const FFoliageType &type = foliage->Types[inst.type];
			foliageStats.considered++;

			const double dx = inst.pos.X - viewpos.X, dy = inst.pos.Y - viewpos.Y, dz = inst.pos.Z - viewpos.Z;
			const double distsq = dx * dx + dy * dy + dz * dz;
			const FFoliageTypeFrameInfo &ti = typeInfo[inst.type];
			if ((culldistsq > 0 && distsq >= culldistsq) || (ti.maxDistSq >= 0 && distsq >= ti.maxDistSq))
			{
				foliageStats.culledByDistance++;
				continue;
			}

			int level = 0;
			if (ti.numLevels > 0 && lodscale > 0)
			{
				level = PickFoliageLevel(ti, sqrt(distsq) * lodscale, ti.noHysteresis ? 0. : hysteresis, inst.lodLevel);
				inst.lodLevel = (uint8_t)level;
			}
			if (level > 0 && ti.levels[level - 1]->type != LOD_Model)
			{
				foliageStats.hiddenByLOD++;
				continue;
			}

			int lightIndex = -1;
			if (type.allowLights && distsq < ti.lightDistSq)
			{
				// Lights are found from the plant's own subsector, even when it was reached through a neighbouring one.
				if (candidateSub != (int)inst.subsector)
				{
					GatherCandidateLights(Level, &Level->subsectors[inst.subsector]);
					candidateSub = (int)inst.subsector;
				}
				// Light strength fades out across [lightFadeStart, lightRange], so plants don't pop between lit and unlit.
				int strength = LIGHT_FADE_STEPS;
				if (ti.lightRange > ti.lightFadeStart)
				{
					const double f = (ti.lightRange - sqrt(distsq)) / (ti.lightRange - ti.lightFadeStart);
					strength = (int)ceil(clamp(f, 0., 1.) * LIGHT_FADE_STEPS);
				}
				if (strength > 0 && candidateLights.Size() > 0) lightIndex = GetPlantLightIndex(inst, type, strength);
				if (lightIndex >= 0) foliageStats.lit++;
			}

			foliageStats.drawnPerTypeLevel[inst.type * statStride + level]++;
			foliageStats.drawn++;

			if (distsq < ti.fadeEndSq)
			{
				// 0 = fully translucent (closer than the range start) ... FOLIAGE_FADE_STEPS = fully opaque (range end).
				// Additive plants can't fade into the opaque version, so they switch at the end of the range.
				int fade = 0;
				const bool additive = type.style == LegacyRenderStyles[STYLE_Add];
				if (!additive && ti.fadeEnd > ti.fadeStart)
				{
					const double f = (sqrt(distsq) - ti.fadeStart) / (ti.fadeEnd - ti.fadeStart);
					fade = (int)round(clamp(f, 0., 1.) * FOLIAGE_FADE_STEPS);
				}
				if (fade < FOLIAGE_FADE_STEPS)
				{
					FFoliageTranslucentItem item;
					item.instance = i;
					item.lightIndex = lightIndex;
					item.distSq = (float)distsq;
					item.level = (uint8_t)level;
					item.fade = (uint8_t)fade;
					FoliageTranslucent.Push(item);
					continue;
				}
			}

			FFoliageGroup &g = FindGroup(inst.type, level, GetLightGroup(this, inst.sector), lightIndex, LODModelForLevel(inst.type, level));
			g.matrices.Push(inst.matrix);
			g.matrices.Push(inst.normalMatrix);
		}
	}

	screen->mLights->Unmap();

	// Draw: one group at a time, instanced where possible. Translucent plants beyond their FoliageTranslucentRange are drawn opaque here.
	int currentLight = -1;
	for (unsigned gi = 0; gi < numGroups; gi++)
	{
		FFoliageGroup &g = groups[gi];
		const unsigned count = g.matrices.Size() / 2;
		if (count == 0) continue;
		foliageStats.groups++;

		if (g.lightGroup != currentLight)
		{
			ApplyLightGroup(this, state, lightGroups[g.lightGroup], 1.f, false);
			currentLight = g.lightGroup;
		}
		DrawFoliageBatch(this, state, foliage->Types[g.type], DefaultRenderStyle(), g.lodModelID, g.lightIndex, g.matrices, 0, count, foliageStats.instancing);
	}

	// Leave the state the way the sprite drawer does.
	state.SetObjectColor(0xffffffff);
	state.SetAddColor(0);
	state.SetDynLight(0, 0, 0);
	state.SetLightIndex(-1);
	state.EnableTexture(true);
	state.SetVertexBuffer(screen->mVertexData);
	foliageStats.time.Unclock();
}

//==========================================================================
//
// Translucent pass: near translucent plants, back to front. Consecutive
// plants that share type, LOD, lighting and light list are drawn in one
// instanced draw; instances are rasterized in order, so blending stays
// correctly sorted. These write depth (above r_foliagealphathreshold) so
// translucent sprites drawn after them don't show through dense grass.
//
//==========================================================================

void HWDrawInfo::DrawFoliageTranslucent(FRenderState &state)
{
	FFoliageSystem *foliage = Level->Foliage;
	if (foliage == nullptr || FoliageTranslucent.Size() == 0 || !r_drawfoliage) return;

	foliageStats.time.Clock();
	foliageStats.translucent += FoliageTranslucent.Size();

	std::sort(FoliageTranslucent.Data(), FoliageTranslucent.Data() + FoliageTranslucent.Size(),
		[](const FFoliageTranslucentItem &a, const FFoliageTranslucentItem &b) { return a.distSq > b.distSq; });

	ResetLightGroups(Level);
	screen->mBones->Map();
	state.SetDepthMask(true);
	const float nearThreshold = clamp<float>(r_foliagealphathreshold, 0.f, 1.f);
	const float farThreshold = clamp<float>(gl_mask_threshold, 0.f, 1.f);

	const bool instancing = state.SupportsModelInstancing();
	TArray<VSMatrix> &runMatrices = translucentRun;
	const unsigned n = FoliageTranslucent.Size();
	unsigned i = 0;
	while (i < n)
	{
		const FFoliageTranslucentItem &first = FoliageTranslucent[i];
		const FFoliageInstance &inst0 = foliage->Instances[first.instance];
		const int lightGroup = GetLightGroup(this, inst0.sector);

		// extend the run while plants can share a draw
		runMatrices.Clear();
		unsigned j = i;
		while (j < n)
		{
			const FFoliageTranslucentItem &item = FoliageTranslucent[j];
			const FFoliageInstance &inst = foliage->Instances[item.instance];
			if (inst.type != inst0.type || item.level != first.level || item.fade != first.fade || item.lightIndex != first.lightIndex || GetLightGroup(this, inst.sector) != lightGroup) break;
			runMatrices.Push(inst.matrix);
			runMatrices.Push(inst.normalMatrix);
			j++;
		}

		const FFoliageType &type = foliage->Types[inst0.type];
		state.SetRenderStyle(type.style);
		state.SetTextureMode(type.style);
		// Fade toward the opaque look: alpha toward 1 and the alpha cutoff toward the opaque pass's.
		const float f = first.fade / float(FOLIAGE_FADE_STEPS);
		state.AlphaFunc(Alpha_GEqual, nearThreshold + (farThreshold - nearThreshold) * f);
		const bool additivefog = type.style.BlendOp == STYLEOP_Add && type.style.DestAlpha == STYLEALPHA_One;
		ApplyLightGroup(this, state, lightGroups[lightGroup], type.alpha + (1.f - type.alpha) * f, additivefog);
		foliageStats.groups++;
		DrawFoliageBatch(this, state, type, type.style, LODModelForLevel(inst0.type, first.level), first.lightIndex, runMatrices, 0, j - i, instancing);
		i = j;
	}

	// Restore what RenderTranslucent expects.
	screen->mBones->Unmap();
	state.SetDepthMask(false);
	state.SetRenderStyle(STYLE_Translucent);
	state.SetTextureMode(TM_NORMAL);
	state.AlphaFunc(Alpha_GEqual, gl_mask_sprite_threshold);
	state.SetObjectColor(0xffffffff);
	state.SetAddColor(0);
	state.SetDynLight(0, 0, 0);
	state.SetLightIndex(-1);
	state.EnableTexture(true);
	state.SetVertexBuffer(screen->mVertexData);
	foliageStats.time.Unclock();
}

//==========================================================================
//
// Diagnostics
//
//==========================================================================

ADD_STAT(foliage)
{
	FString out;
	out.Format("Foliage: %d plants in %d visible subsectors, %d drawn (%d translucent, %d lit by %d light lists), %d hidden by LOD, %d distance-culled\n"
		"%d groups, %d instanced draws, %d single draws, instancing %s, %.3f ms",
		foliageStats.considered, foliageStats.visibleSubsectors, foliageStats.drawn, foliageStats.translucent, foliageStats.lit, foliageStats.lightLists,
		foliageStats.hiddenByLOD, foliageStats.culledByDistance,
		foliageStats.groups, foliageStats.instancedDraws, foliageStats.singleDraws, foliageStats.instancing ? "on" : "off", foliageStats.time.TimeMS());
	return out;
}

CCMD(dumpfoliage)
{
	FFoliageSystem *foliage = primaryLevel ? primaryLevel->Foliage : nullptr;
	Printf(TEXTCOLOR_ORANGE "--- Foliage clusters ---\n");

	// FOLIAGE classes that exist as normal actors right now, whatever the reason: rejected at map load
	// (listed with reasons further down), spawned with Spawn() instead of LevelLocals.SpawnFoliage (ACS, A_SpawnItem...),
	// or placed in the map but never reaching the foliage check. Cluster proxies (solid foliage) are excluded.
	if (primaryLevel != nullptr)
	{
		TMap<PClassActor *, int> live;
		auto it = primaryLevel->GetThinkerIterator<AActor>();
		AActor *mo;
		while ((mo = it.Next()) != nullptr)
		{
			if ((mo->perfFlags & PERF_FOLIAGE) && !(mo->perfFlags & PERF_CLUSTERPROXY)) live[mo->GetClass()]++;
		}
		if (live.CountUsed() > 0)
		{
			Printf(TEXTCOLOR_YELLOW "FOLIAGE classes currently existing as normal actors:\n");
			TMap<PClassActor *, int>::Iterator lit(live);
			TMap<PClassActor *, int>::Pair *pair;
			while (lit.NextPair(pair))
			{
				Printf("  %s: %d\n", pair->Key->TypeName.GetChars(), pair->Value);
			}
		}
	}
	Printf("r_foliageclusters %s (applies at map load), r_drawfoliage %s, renderer instancing %s\n",
		*r_foliageclusters ? "on" : "off", *r_drawfoliage ? "on" : "off", foliageStats.instancing ? "on" : "off");
	Printf("r_foliagerangescale %g, r_foliagealphathreshold %g\n", *r_foliagerangescale, *r_foliagealphathreshold);
	if (foliage == nullptr || (foliage->Types.Size() == 0 && foliage->Rejections.Size() == 0))
	{
		Printf("No FOLIAGE things in this level.\n");
		return;
	}

	Printf("%u plants of %u types, %u subsector references (RenderRadius)", foliage->Instances.Size(), foliage->Types.Size(), foliage->Refs.Size());
	if (foliage->ScriptPlants > 0) Printf(", %u from SpawnFoliage (%u added after map load, saved with the level)", foliage->ScriptPlants, foliage->ScriptSpawns.Size());
	Printf("\n");
	const unsigned statStride = MAX_FOLIAGE_LODS + 1;
	for (unsigned t = 0; t < foliage->Types.Size(); t++)
	{
		const FFoliageType &type = foliage->Types[t];
		const int mid = type.smf->modelsAmount > 0 ? type.smf->modelIDs[0] : -1;
		Printf("  %s (%s): %u placed", type.cls->TypeName.GetChars(),
			mid >= 0 && mid < (int)Models.Size() ? Models[mid]->mFileName.GetChars() : "?", type.count);
		if (type.proxies > 0) Printf(", %u solid (collision proxy actors)", type.proxies);
		const FActorInfo *ai = type.cls->ActorInfo();
		if (type.translucent) Printf(", %s alpha %.2f fading %g-%g", type.style == LegacyRenderStyles[STYLE_Add] ? "additive" : "translucent", type.alpha, ai->FoliageFadeStart, ai->FoliageFadeEnd);
		if (!type.allowLights) Printf(", no dynamic lights");
		else Printf(", lights within %g (fading from %g)", ai->FoliageLightRange, ai->FoliageLightFadeStart < 0 ? ai->FoliageLightRange * 0.75 : ai->FoliageLightFadeStart);
		if (t * statStride < foliageStats.drawnPerTypeLevel.Size())
		{
			FString levels;
			int total = 0;
			for (unsigned l = 0; l < statStride; l++)
			{
				const int n = foliageStats.drawnPerTypeLevel[t * statStride + l];
				total += n;
				if (n > 0) levels.AppendFormat("%s LOD%u: %d", levels.IsEmpty() ? "" : ",", l, n);
			}
			Printf(", %d drawn last frame%s%s", total, levels.IsEmpty() ? "" : " -", levels.GetChars());
		}
		if (type.hasSpriteLOD) Printf(TEXTCOLOR_YELLOW " (\"Sprite\" LOD level is hidden)");
		Printf("\n");
	}

	if (foliage->Rejections.Size() > 0)
	{
		static const char *reasons[FFoliageSystem::NUM_REJECT] =
		{
			"r_foliageclusters off", "has a TID or special", "SHOOTABLE, PUSHABLE or a pickup", "unsupported render style", "no model for spawn state"
		};
		Printf(TEXTCOLOR_YELLOW "FOLIAGE things (map or SpawnFoliage) spawned as normal actors:\n");
		for (auto &r : foliage->Rejections)
		{
			FString line;
			line.Format("  %s:", r.cls->TypeName.GetChars());
			bool first = true;
			for (int i = 0; i < FFoliageSystem::NUM_REJECT; i++)
			{
				if (r.counts[i] == 0) continue;
				line.AppendFormat("%s %d %s", first ? "" : ",", r.counts[i], reasons[i]);
				first = false;
			}
			Printf("%s\n", line.GetChars());
		}
	}
}
