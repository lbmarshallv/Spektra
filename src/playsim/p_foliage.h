/*
** p_foliage.h
**
** Static foliage clusters: non-solid, non-interactive model things that are
** absorbed at map load instead of being spawned as actors, and drawn by the
** hardware renderer in batches.
**
**---------------------------------------------------------------------------
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#pragma once

#include "tarray.h"
#include "vectors.h"
#include "matrix.h"
#include "renderstyle.h"

class PClassActor;
class AActor;
class FSerializer;
struct FSpriteModelFrame;
struct FMapThing;
struct sector_t;
struct FLevelLocals;

// Where and how to place one plant. Map things and LevelLocals.SpawnFoliage both describe their plants this way.
struct FFoliagePlacement
{
	DVector2 xy;
	double z = 0;				// above the floor (below the ceiling for SPAWNCEILING classes), or absolute with absoluteZ
	bool absoluteZ = false;
	double yaw = 0;				// degrees: the thing's / actor's angle
	double pitch = 0;			// degrees; only models with USEACTORPITCH / USEACTORROLL use these
	double roll = 0;
	bool defaultPitchRoll = true;	// map things: 0 means "use the class default"
	DVector2 scale = { 1, 1 };	// multiplier on the class scale (or the final scale with absoluteScale)
	bool absoluteScale = false;
	int renderStyle = -1;		// ERenderStyle override, -1 = class default
	double alpha = -1;			// override, < 0 = class default
	bool randomize = true;		// apply FOLIAGERANDOMYAW and FoliageRandomScale
};

// What a placement turned into, for spawning the matching actor.
struct FFoliageResolved
{
	DVector3 pos;				// actor origin
	double yaw;					// final angle, including random yaw (without SpriteRotation)
	double pitch, roll;
	DVector2 scale;				// final actor scale, including random scale
	FRenderStyle style;
	double alpha;
};

// A cluster plant added by script after the map was loaded. Map plants are rebuilt from the map,
// but these have to be saved with the level and re-added when it is loaded.
struct FFoliageSpawnRecord
{
	PClassActor *cls = nullptr;
	DVector3 pos = { 0, 0, 0 };
	double yaw = 0, pitch = 0, roll = 0;
	DVector2 scale = { 1, 1 };
	FRenderStyle style = DefaultRenderStyle();
	double alpha = 1;
};

// One kind of plant: an actor class plus the model frame of its spawn state and its render style.
struct FFoliageType
{
	PClassActor *cls;
	FSpriteModelFrame *smf;
	unsigned smfFlags;
	unsigned count;			// how many were placed in this level
	unsigned proxies;		// how many of those are SOLID and kept a collision-only proxy actor
	FRenderStyle style;		// Normal, Translucent or Add
	float alpha;
	bool translucent;		// blended near the viewer (r_foliagetranslucentdist), drawn opaque beyond it
	bool allowLights;		// can receive per-pixel dynamic lights near the viewer (r_foliagelightdist)
	bool hasSpriteLOD;		// uses a LODModel "Sprite" level without +LODSPRITE (no sprite to show; treated as hidden)
	float lightRadius;		// bounding sphere used to find touching dynamic lights
	float centerHeight;		// sphere center above the plant's origin
};

// "This plant must be considered when this render subsector is visible." Every plant has one for its own
// subsector, plus one for each other subsector its RenderRadius reaches (like actors' touching_rendersectors).
struct FFoliageRef
{
	uint32_t subsector;
	uint32_t instance;
};

// One placed plant. The matrices are fixed when it is placed.
struct FFoliageInstance
{
	VSMatrix matrix;		// object-to-world, in the renderer's coordinate convention
	VSMatrix normalMatrix;
	FVector3 pos;			// map coordinates, for distance checks
	sector_t *sector;		// lighting comes from here
	uint32_t subsector;		// render subsector containing the plant's origin (dynamic lights are found from here)
	uint16_t type;			// index into FFoliageSystem::Types
	uint8_t lodLevel;		// last picked level of detail (renderer only, for hysteresis)
	uint8_t padding;
	float yaw;				// radians, map convention (counterclockwise), SpriteRotation included: picks the LOD sprite's angle
	float spriteScaleXY;	// scale relative to the class scale, for the LOD sprite's size
	float spriteScaleZ;
};

struct FFoliageSystem
{
	enum EReject
	{
		REJ_Disabled,		// r_foliageclusters was off when the map loaded
		REJ_TIDOrSpecial,	// the placed thing has a TID or a special, so scripts may need it
		REJ_Interactive,	// SHOOTABLE, PUSHABLE or a pickup (SPECIAL): something could change or move it
		REJ_RenderStyle,	// render style other than Normal, Translucent or Add
		REJ_NoModel,		// no MODELDEF entry for the spawn state
		NUM_REJECT
	};

	struct FRejection
	{
		PClassActor *cls;
		int counts[NUM_REJECT];
	};

	TArray<FFoliageType> Types;
	TArray<FFoliageInstance> Instances;		// in placement order; indices never change
	TArray<FFoliageRef> Refs;				// sorted by subsector by BuildIndex()
	TArray<uint32_t> RefStart;				// Refs of subsector i are [RefStart[i], RefStart[i+1])
	TArray<FRejection> Rejections;
	TArray<FFoliageSpawnRecord> ScriptSpawns;	// plants added by script after map load (saved with the level)
	unsigned ScriptPlants = 0;					// all plants added through SpawnFoliage, during or after map load
	bool Finalized = false;		// the map's things have all been spawned
	bool Dirty = false;			// plants were added after Finalize; the subsector index needs rebuilding

	enum EAddResult
	{
		ADD_NotFoliage,		// spawn a normal actor
		ADD_Absorbed,		// drawn by the cluster; don't spawn anything
		ADD_Proxy,			// drawn by the cluster; spawn the actor as a collision-only proxy (SOLID things)
	};

	// Map things (from SpawnMapThing, after skill/mode filtering).
	EAddResult TryAdd(FLevelLocals *Level, const FMapThing *mthing, PClassActor *cls, FFoliageResolved &resolved);

	// LevelLocals.SpawnFoliage: adds the plant to a cluster if the class qualifies, otherwise spawns a normal actor.
	// Returns the actor that was spawned (a normal actor, or the collision proxy of a SOLID plant), or null.
	AActor *SpawnScripted(FLevelLocals *Level, PClassActor *cls, const FFoliagePlacement &placement);

	void Finalize(FLevelLocals *Level);
	void RebuildIfDirty(FLevelLocals *Level);
	void Serialize(FSerializer &arc, FLevelLocals *Level);

	bool SubsectorHasFoliage(unsigned index) const
	{
		return index + 1 < RefStart.Size() && RefStart[index] != RefStart[index + 1];
	}

private:
	EAddResult Place(FLevelLocals *Level, PClassActor *cls, const FFoliagePlacement &p, FFoliageResolved &out, bool noteRejections);
	void BuildIndex(FLevelLocals *Level);
	void AddRefs(FLevelLocals *Level, uint32_t instance, uint32_t home, const DVector2 &pos, double radius);
	void NoteRejection(PClassActor *cls, int reason);
	int FindOrAddType(PClassActor *cls, FSpriteModelFrame *smf, FRenderStyle style, float alpha);
};
