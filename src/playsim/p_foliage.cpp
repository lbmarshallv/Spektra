/*
** p_foliage.cpp
**
** Static foliage clusters (level side). Map things whose class has the
** FOLIAGE flag and that nothing else can depend on are turned into compact
** instance records at map load instead of being spawned as actors. They
** never tick, aren't in the blockmap and aren't saved; the map rebuilds them
** on every load. Scripts can add plants too (LevelLocals.SpawnFoliage); those
** added after map load are saved with the level as spawn records. The
** hardware renderer draws them (hw_foliage.cpp).
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
#include "doomdata.h"
#include "models.h"
#include "model.h"
#include "c_cvars.h"
#include "printf.h"
#include "serializer.h"
#include "serializer_doom.h"
#include "vm.h"
#include "m_bbox.h"

// This decides which things exist as actors, so it is game state: SERVERINFO keeps it identical for all
// players in a netgame, records it in demos, and restores it from savegames before the map is loaded.
CVARD(Bool, r_foliageclusters, true, CVAR_ARCHIVE | CVAR_SERVERINFO, "Turn non-solid FOLIAGE things into static foliage clusters instead of actors. Takes effect on the next map load.")

//==========================================================================
//
// Repeatable pseudo-random values from a plant's placement. This must
// never use the game's random number generators: which plants exist and
// how they look must not depend on (or change) the playsim's random
// sequence, so every player in a netgame and every reload of a map gets
// exactly the same result.
//
//==========================================================================

static uint32_t FoliageHash(double x, double y, double z, double angle, uint32_t salt)
{
	uint32_t h = 2166136261u ^ (salt * 0x9e3779b9u);
	auto mix = [&h](double v)
	{
		h ^= (uint32_t)(int32_t)floor(v * 16.0);
		h *= 16777619u;
		h ^= h >> 15;
	};
	mix(x);
	mix(y);
	mix(z);
	mix(angle);
	h ^= h >> 13;
	h *= 0x5bd1e995u;
	h ^= h >> 15;
	return h;
}

static double FoliageRandom(const FFoliagePlacement &p, uint32_t salt)
{
	return (FoliageHash(p.xy.X, p.xy.Y, p.z, p.yaw, salt) >> 8) * (1.0 / 16777216.0);	// [0, 1)
}

//==========================================================================
//
//
//
//==========================================================================

void FFoliageSystem::NoteRejection(PClassActor *cls, int reason)
{
	for (auto &r : Rejections)
	{
		if (r.cls == cls)
		{
			r.counts[reason]++;
			return;
		}
	}
	FRejection r = {};
	r.cls = cls;
	r.counts[reason] = 1;
	Rejections.Push(r);
}

int FFoliageSystem::FindOrAddType(PClassActor *cls, FSpriteModelFrame *smf, FRenderStyle style, float alpha)
{
	for (unsigned i = 0; i < Types.Size(); i++)
	{
		if (Types[i].cls == cls && Types[i].smf == smf && Types[i].style == style && Types[i].alpha == alpha) return i;
	}
	const AActor *info = GetDefaultByType(cls);
	FFoliageType t = {};
	t.cls = cls;
	t.smf = smf;
	t.smfFlags = smf->getFlags(nullptr);
	t.style = style;
	t.alpha = alpha;
	t.translucent = !(style == DefaultRenderStyle()) || alpha < 1.f;
	t.allowLights = !(info->renderflags2 & RF2_NODYNAMICLIGHTING) && !(t.smfFlags & MDL_NOPERPIXELLIGHTING) && !smf->isVoxel;
	t.centerHeight = (float)(info->Height * 0.5);
	t.lightRadius = (float)max(info->radius, info->Height * 0.5);
	for (const LODmodel *m = cls->ActorInfo()->LODmodels; m != nullptr; m = m->Next)
	{
		if (m->type == LOD_Sprite) t.hasSpriteLOD = true;
	}
	if (t.hasSpriteLOD && Finalized)
	{
		Printf(TEXTCOLOR_ORANGE "Foliage: %s has a \"Sprite\" LODModel level. Sprite stand-ins aren't supported for foliage yet; that level is hidden.\n", cls->TypeName.GetChars());
	}
	return Types.Push(t);
}

//==========================================================================
//
// Resolves a placement (position, angle, scale, style) the same way
// SpawnMapThing places an actor, and adds a cluster instance if the class
// qualifies. 'out' is always filled in, so a caller can spawn a matching
// actor whether or not the plant became foliage.
//
// SOLID plants (trees) are drawn by the cluster too, but still need their
// actor as a collision-only proxy: collision finds obstacles through
// blockmap actors, so the proxy keeps movement, projectiles and hitscans
// working unchanged. Nothing can move or change a proxy (interactive
// classes are rejected), so the cluster copy stays valid.
//
//==========================================================================

FFoliageSystem::EAddResult FFoliageSystem::Place(FLevelLocals *Level, PClassActor *cls, const FFoliagePlacement &p, FFoliageResolved &out, bool noteRejections)
{
	const AActor *info = GetDefaultByType(cls);
	const FActorInfo *ai = cls->ActorInfo();

	// Position: on the floor / under the ceiling like ONFLOORZ / ONCEILINGZ, or absolute.
	sector_t *sec = Level->PointInSector(p.xy);
	const bool onCeiling = !!(info->flags & MF_SPAWNCEILING);
	double z;
	if (p.absoluteZ) z = p.z;
	else if (onCeiling) z = sec->ceilingplane.ZatPoint(p.xy) - info->Height - p.z;
	else z = sec->floorplane.ZatPoint(p.xy) + p.z;
	const DVector3 up = onCeiling ? -sec->ceilingplane.Normal() : sec->floorplane.Normal();
	out.pos = DVector3(p.xy, z);

	// Angles and scale, with the class's random yaw/scale.
	out.yaw = p.yaw;
	if (p.randomize && (info->perfFlags & PERF_FOLIAGERANDOMYAW)) out.yaw += 360.0 * FoliageRandom(p, 2);
	out.pitch = (p.defaultPitchRoll && p.pitch == 0) ? info->Angles.Pitch.Degrees() : p.pitch;
	out.roll = (p.defaultPitchRoll && p.roll == 0) ? info->Angles.Roll.Degrees() : p.roll;
	if (p.absoluteScale)
	{
		out.scale = p.scale;
	}
	else
	{
		out.scale = info->Scale;
		if (p.scale.X != 0) out.scale.X *= p.scale.X;
		if (p.scale.Y != 0) out.scale.Y *= p.scale.Y;
		if (p.randomize && (ai->FoliageScaleMin != 1.f || ai->FoliageScaleMax != 1.f))
		{
			out.scale *= ai->FoliageScaleMin + (ai->FoliageScaleMax - ai->FoliageScaleMin) * FoliageRandom(p, 1);
		}
	}

	FRenderStyle style = info->RenderStyle;
	if (p.renderStyle >= 0 && p.renderStyle < STYLE_Count) style = (ERenderStyle)p.renderStyle;
	out.alpha = clamp(p.alpha >= 0 ? p.alpha : info->Alpha, 0., 1.);
	out.style = style;

	// Does it qualify?
	if (!(info->perfFlags & PERF_FOLIAGE)) return ADD_NotFoliage;
	if (!r_foliageclusters)
	{
		if (noteRejections) NoteRejection(cls, REJ_Disabled);
		return ADD_NotFoliage;
	}
	if ((info->flags & (MF_SHOOTABLE | MF_SPECIAL)) || (info->flags2 & MF2_PUSHABLE))
	{
		if (noteRejections) NoteRejection(cls, REJ_Interactive);
		return ADD_NotFoliage;
	}
	if (style == DefaultRenderStyle() && out.alpha < 1.) style = LegacyRenderStyles[STYLE_Translucent];
	if (!(style == DefaultRenderStyle() || style == LegacyRenderStyles[STYLE_Translucent] || style == LegacyRenderStyles[STYLE_Add]))
	{
		if (noteRejections) NoteRejection(cls, REJ_RenderStyle);
		return ADD_NotFoliage;
	}
	FState *state = info->SpawnState;
	FSpriteModelFrame *smf = state ? FindModelFrame(cls, state->sprite, state->GetFrame(), false) : nullptr;
	if (smf == nullptr || smf->modelsAmount == 0)
	{
		if (noteRejections) NoteRejection(cls, REJ_NoModel);
		return ADD_NotFoliage;
	}
	const bool solid = !!(info->flags & MF_SOLID);

	// Rotation, matching FSpriteModelFrame::ObjectToWorldMatrix(AActor*...) for a standing actor.
	const int typeIndex = FindOrAddType(cls, smf, style, (float)out.alpha);
	const unsigned smfFlags = Types[typeIndex].smfFlags;
	const double yaw = out.yaw + info->SpriteRotation.Degrees();
	double pitch = 0, roll = 0;
	if (smfFlags & MDL_USEACTORPITCH) pitch = (smfFlags & MDL_BADROTATION) ? out.pitch : -out.pitch;
	if (smfFlags & MDL_USEACTORROLL) roll = out.roll;

	VSMatrix local = smf->ObjectToWorldMatrix(Level, DVector3(0, 0, 0), DRotator(DAngle::fromDeg(pitch), DAngle::fromDeg(yaw), DAngle::fromDeg(roll)), out.scale, smfFlags, 0.);

	const DVector3 rpos = out.pos + info->WorldOffset;	// drawn position
	FFoliageInstance inst;
	inst.matrix.loadIdentity();
	inst.matrix.translate((float)rpos.X, (float)rpos.Z, (float)rpos.Y);	// renderer coordinates are (x, z, y)
	if (info->perfFlags & PERF_FOLIAGEALIGNSLOPE)
	{
		// Rotate the model's up axis (0,1,0) onto the surface normal: axis = up x n, angle = acos(up . n).
		const float nx = (float)up.X, ny = (float)up.Z, nz = (float)up.Y;
		const float len = sqrtf(nz * nz + nx * nx);
		if (len > 1e-5f)
		{
			const float angle = (float)(acos(clamp(ny, -1.f, 1.f)) * (180. / M_PI));
			inst.matrix.rotate(angle, nz / len, 0.f, -nx / len);
		}
	}
	inst.matrix.multMatrix(local);
	inst.normalMatrix.computeNormalMatrix(inst.matrix);
	inst.pos = FVector3((float)rpos.X, (float)rpos.Y, (float)rpos.Z);
	inst.sector = sec;
	inst.subsector = Level->PointInRenderSubsector(p.xy)->Index();
	inst.type = (uint16_t)typeIndex;
	inst.lodLevel = 0;
	inst.padding = 0;
	const uint32_t instanceIndex = Instances.Push(inst);

	// Which render subsectors make it visible: like actors, max(radius, RenderRadius) reaches into neighbouring
	// subsectors (a negative RenderRadius means only its own). Scaled up by the foliage random scale, if any.
	double renderRadius = 0;
	if (info->renderradius >= 0)
	{
		renderRadius = max(info->radius, info->renderradius);
		if (info->Scale.X != 0) renderRadius *= max(1., fabs(out.scale.X / info->Scale.X));
	}
	AddRefs(Level, instanceIndex, inst.subsector, p.xy, renderRadius);

	Types[typeIndex].count++;
	if (solid) Types[typeIndex].proxies++;
	if (Finalized) Dirty = true;	// refs appended after the index was built; the sorted part stays valid until the rebuild
	return solid ? ADD_Proxy : ADD_Absorbed;
}

//==========================================================================
//
// Adds the plant to its own subsector and to every other render subsector
// whose BSP bounding box overlaps the square of its render radius
// (conservative: a box can overlap without the subsector itself doing so,
// which only means the plant is considered a little more often).
//
//==========================================================================

void FFoliageSystem::AddRefs(FLevelLocals *Level, uint32_t instance, uint32_t home, const DVector2 &pos, double radius)
{
	Refs.Push({ home, instance });
	if (radius <= 0 || Level->nodes.Size() == 0) return;

	const float left = float(pos.X - radius), right = float(pos.X + radius);
	const float bottom = float(pos.Y - radius), top = float(pos.Y + radius);

	TArray<void *> stack;
	stack.Push(Level->HeadNode());
	while (stack.Size() > 0)
	{
		void *child;
		stack.Pop(child);
		if ((size_t)child & 1)
		{
			const uint32_t sub = ((subsector_t *)((uint8_t *)child - 1))->Index();
			if (sub != home) Refs.Push({ sub, instance });
			continue;
		}
		const node_t *node = (const node_t *)child;
		for (int side = 0; side < 2; side++)
		{
			const float *box = node->bbox[side];
			if (box[BOXLEFT] <= right && box[BOXRIGHT] >= left && box[BOXBOTTOM] <= top && box[BOXTOP] >= bottom)
			{
				stack.Push(node->children[side]);
			}
		}
	}
}

//==========================================================================
//
// Called from SpawnMapThing for classes with the FOLIAGE flag, after all
// skill/mode filtering.
//
//==========================================================================

FFoliageSystem::EAddResult FFoliageSystem::TryAdd(FLevelLocals *Level, const FMapThing *mthing, PClassActor *cls, FFoliageResolved &resolved)
{
	const AActor *info = GetDefaultByType(cls);
	if (!(info->perfFlags & PERF_FOLIAGE) || Finalized) return ADD_NotFoliage;	// map things spawned after map load always become actors

	if (mthing->thingid != 0 || mthing->special != 0)
	{
		NoteRejection(cls, REJ_TIDOrSpecial);
		return ADD_NotFoliage;
	}

	FFoliagePlacement p;
	p.xy = mthing->pos.XY();
	p.z = mthing->pos.Z;
	p.yaw = mthing->angle;
	p.pitch = mthing->pitch;
	p.roll = mthing->roll;
	p.defaultPitchRoll = true;
	p.scale = DVector2(mthing->Scale.X, mthing->Scale.Y);	// 0 = class default
	p.renderStyle = mthing->RenderStyle != STYLE_Count ? (int)mthing->RenderStyle : -1;
	p.alpha = mthing->Alpha;
	return Place(Level, cls, p, resolved, true);
}

//==========================================================================
//
// LevelLocals.SpawnFoliage. During map load (e.g. from a FoliageSpawner's
// BeginPlay) plants are part of the map and get rebuilt on every load like
// map things. After map load they are recorded and saved with the level.
//
// Solid plants and plants that don't qualify spawn an actor. That happens
// identically on every machine, and the actor is saved like any other.
//
//==========================================================================

AActor *FFoliageSystem::SpawnScripted(FLevelLocals *Level, PClassActor *cls, const FFoliagePlacement &placement)
{
	cls = cls->GetReplacement(Level);
	FFoliageResolved r;
	const EAddResult result = Place(Level, cls, placement, r, true);

	if (result != ADD_NotFoliage)
	{
		ScriptPlants++;
		if (Finalized)
		{
			FFoliageSpawnRecord rec;
			rec.cls = cls;
			rec.pos = r.pos;
			rec.yaw = r.yaw;
			rec.pitch = r.pitch;
			rec.roll = r.roll;
			rec.scale = r.scale;
			rec.style = r.style;
			rec.alpha = r.alpha;
			ScriptSpawns.Push(rec);
		}
		if (result == ADD_Absorbed) return nullptr;
	}

	AActor *mo = AActor::StaticSpawn(Level, cls, r.pos, NO_REPLACE);
	if (mo == nullptr) return nullptr;
	if (result == ADD_Proxy) mo->perfFlags |= PERF_CLUSTERPROXY;
	mo->Angles.Yaw = DAngle::fromDeg(r.yaw);
	mo->Angles.Pitch = DAngle::fromDeg(r.pitch);
	mo->Angles.Roll = DAngle::fromDeg(r.roll);
	mo->PrevAngles = mo->Angles;
	mo->Scale = r.scale;
	mo->RenderStyle = r.style;
	mo->Alpha = r.alpha;
	return mo;
}

//==========================================================================
//
// Groups the plant references by render subsector so the renderer can find
// a visible subsector's plants directly.
//
//==========================================================================

void FFoliageSystem::BuildIndex(FLevelLocals *Level)
{
	std::sort(Refs.Data(), Refs.Data() + Refs.Size(),
		[](const FFoliageRef &a, const FFoliageRef &b) { return a.subsector != b.subsector ? a.subsector < b.subsector : a.instance < b.instance; });

	const unsigned numsubs = Level->subsectors.Size();
	RefStart.Resize(numsubs + 1);
	unsigned index = 0;
	for (unsigned s = 0; s <= numsubs; s++)
	{
		while (index < Refs.Size() && Refs[index].subsector < s) index++;
		RefStart[s] = index;
	}
	Dirty = false;
}

// Called once all map things are spawned.
void FFoliageSystem::Finalize(FLevelLocals *Level)
{
	if (Finalized) return;
	Finalized = true;
	BuildIndex(Level);

	for (auto &t : Types)
	{
		if (t.hasSpriteLOD)
		{
			Printf(TEXTCOLOR_ORANGE "Foliage: %s has a \"Sprite\" LODModel level. Sprite stand-ins aren't supported for foliage yet; that level is hidden.\n", t.cls->TypeName.GetChars());
		}
	}
	if (Instances.Size() > 0)
	{
		DPrintf(DMSG_NOTIFY, "Foliage: %u plants of %u types in clusters, %u subsector references\n", Instances.Size(), Types.Size(), Refs.Size());
	}
}

// Called by the renderer before each frame (before the BSP walk reads the index).
void FFoliageSystem::RebuildIfDirty(FLevelLocals *Level)
{
	if (Dirty) BuildIndex(Level);
}

//==========================================================================
//
// Savegames and hub snapshots: only script-added plants are stored. The
// map's own plants (including FoliageSpawner results during map load) are
// rebuilt from the map before this runs.
//
//==========================================================================

FSerializer &Serialize(FSerializer &arc, const char *key, FFoliageSpawnRecord &rec, FFoliageSpawnRecord *def)
{
	if (arc.BeginObject(key))
	{
		arc("class", rec.cls)
			("pos", rec.pos)
			("yaw", rec.yaw)
			("pitch", rec.pitch)
			("roll", rec.roll)
			("scale", rec.scale)
			("style", rec.style)
			("alpha", rec.alpha);
		arc.EndObject();
	}
	return arc;
}

void FFoliageSystem::Serialize(FSerializer &arc, FLevelLocals *Level)
{
	if (arc.isWriting())
	{
		if (ScriptSpawns.Size() > 0) arc("foliagespawns", ScriptSpawns);
		return;
	}

	TArray<FFoliageSpawnRecord> records;
	arc("foliagespawns", records);
	ScriptSpawns.Clear();
	for (auto &rec : records)
	{
		if (rec.cls == nullptr) continue;	// class no longer exists
		FFoliagePlacement p;
		p.xy = rec.pos.XY();
		p.z = rec.pos.Z;
		p.absoluteZ = true;
		p.yaw = rec.yaw;
		p.pitch = rec.pitch;
		p.roll = rec.roll;
		p.defaultPitchRoll = false;
		p.scale = rec.scale;
		p.absoluteScale = true;
		p.randomize = false;
		p.alpha = rec.alpha;
		// The style is restored through the record; map it back to a legacy style index if possible.
		p.renderStyle = -1;
		for (int i = 0; i < STYLE_Count; i++)
		{
			if (LegacyRenderStyles[i] == rec.style) { p.renderStyle = i; break; }
		}
		FFoliageResolved r;
		// Proxy actors of solid plants come back from the savegame on their own; only the cluster copy is re-added here.
		if (Place(Level, rec.cls, p, r, false) != ADD_NotFoliage)
		{
			ScriptSpawns.Push(rec);
			ScriptPlants++;
		}
	}
}

//==========================================================================
//
// Script interface
//
//==========================================================================

static AActor *SpawnFoliage(FLevelLocals *self, PClassActor *cls, double x, double y, double z, double angle, double pitch, double roll,
	double scaleX, double scaleY, bool absoluteZ, bool randomize)
{
	if (cls == nullptr) return nullptr;
	if (self->Foliage == nullptr) self->Foliage = new FFoliageSystem;
	FFoliagePlacement p;
	p.xy = DVector2(x, y);
	p.z = z;
	p.absoluteZ = absoluteZ;
	p.yaw = angle;
	p.pitch = pitch;
	p.roll = roll;
	p.defaultPitchRoll = false;
	p.scale = DVector2(scaleX, scaleY);
	p.randomize = randomize;
	return self->Foliage->SpawnScripted(self, cls, p);
}

DEFINE_ACTION_FUNCTION_NATIVE(FLevelLocals, SpawnFoliage, SpawnFoliage)
{
	PARAM_SELF_STRUCT_PROLOGUE(FLevelLocals);
	PARAM_CLASS_NOT_NULL(cls, AActor);
	PARAM_FLOAT(x);
	PARAM_FLOAT(y);
	PARAM_FLOAT(z);
	PARAM_FLOAT(angle);
	PARAM_FLOAT(pitch);
	PARAM_FLOAT(roll);
	PARAM_FLOAT(scaleX);
	PARAM_FLOAT(scaleY);
	PARAM_BOOL(absoluteZ);
	PARAM_BOOL(randomize);
	ACTION_RETURN_OBJECT(SpawnFoliage(self, cls, x, y, z, angle, pitch, roll, scaleX, scaleY, absoluteZ, randomize));
}

// Repeatable random number in [0, 1) from a position and a salt, for spawners that must not use the game's RNG.
static double FoliageRandomScript(FLevelLocals *self, double x, double y, double z, int salt)
{
	return (FoliageHash(x, y, z, 0, (uint32_t)salt) >> 8) * (1.0 / 16777216.0);
}

DEFINE_ACTION_FUNCTION_NATIVE(FLevelLocals, FoliageRandom, FoliageRandomScript)
{
	PARAM_SELF_STRUCT_PROLOGUE(FLevelLocals);
	PARAM_FLOAT(x);
	PARAM_FLOAT(y);
	PARAM_FLOAT(z);
	PARAM_INT(salt);
	ACTION_RETURN_FLOAT(FoliageRandomScript(self, x, y, z, salt));
}
