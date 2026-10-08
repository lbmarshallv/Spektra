#include <bullet/btBulletCollisionCommon.h>
#include <g_levellocals.h>
#include <p_collision.h>
#include "filesystem.h"
#include <bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include "c_cvars.h"
#include "p_effect.h"

// Shows where 3D-model collisions happen: a particle at each contact point.
CVAR(Bool, col_showhits, false, 0)

void AdvancedCollisionModel::setWorldPos(DVector3 pos)
{
	btTransform transform = collider->getWorldTransform();
	transform.setOrigin(toBulletPoint(pos));
	collider->setWorldTransform(transform);
}

void AdvancedCollisionModel::setRotation(DRotator d)
{
	btTransform transform = collider->getWorldTransform();
	auto         r         = btQuaternion(0,0,0);
	r.setEulerZYX(d.Yaw.Radians(), d.Pitch.Radians(), d.Roll.Radians());
	transform.setRotation(r);
	collider->setWorldTransform(transform);
}

void AdvancedCollisionModel::updateTransform()
{
	btTransform transform = collider->getWorldTransform();
	transform.setIdentity();
	setRotation(owner->Angles);
	setWorldPos(owner->Pos());
}

void DoomStandardCollisionModel::updateTransform()
{
	if (owner->radius != radius || owner->Height != height)
	{
		radius = owner->radius;
		height = owner->Height;
		btCollisionShape *oldShape = collider->getCollisionShape();
		collider->setCollisionShape(initShape());
		delete oldShape;
	}
	AdvancedCollisionModel::updateTransform();
}

void CapsuleCollisionModel::updateTransform()
{
	if (owner->radius != radius || owner->Height != height)
	{
		radius = owner->radius;
		height = owner->Height;
		btCollisionShape *oldShape = collider->getCollisionShape();
		collider->setCollisionShape(initShape());
		delete oldShape;
	}
	AdvancedCollisionModel::updateTransform();
}

void FLevelLocals::InitCollisionWorld()
{
	if (collisionWorld)
		delete collisionWorld;
	auto config     = new btDefaultCollisionConfiguration();
	auto dispatcher = new btCollisionDispatcher(config);
	auto broadphase = new btDbvtBroadphase();
	collisionWorld  = new btCollisionWorld(dispatcher, broadphase, config);
#ifdef _DEBUG
	Printf("Created CollisionWorld for %s\n", this->MapName);
#endif
}

//==========================================================================
//
// The collision world only holds the actors taking part in the query being
// made: they are added before it and removed after.
//
//==========================================================================

void FLevelLocals::AddCollisionObject(btCollisionObject *obj)
{
	if (collisionWorld)
		collisionWorld->addCollisionObject(obj);
	else
		Printf(TEXTCOLOR_RED "AddCollisionObject was called without a CollisionWorld\n");
}

void FLevelLocals::AddCollisionObject(AActor *thing)
{
	AddCollisionObject(thing->collisionModel);
}

void FLevelLocals::AddCollisionObject(AdvancedCollisionModel *model)
{
	if (collisionModels.Find(model) == collisionModels.Size())
	{
		AddCollisionObject(model->collider);
		collisionModels.Push(model);
	}
}

void FLevelLocals::RemoveCollisionObject(btCollisionObject *obj)
{
	if (collisionWorld)
		collisionWorld->removeCollisionObject(obj);
	else
		Printf(TEXTCOLOR_RED "RemoveCollisionObject was called without a CollisionWorld\n");
}

void FLevelLocals::RemoveCollisionObject(AActor *thing)
{
	for (unsigned i = 0; i < collisionModels.Size(); i++)
	{
		auto c = collisionModels[i];
		if (c && c->owner == thing)
		{
			collisionModels.Delete(i);
			RemoveCollisionObject(c->collider);
			return;
		}
	}
}

void FLevelLocals::UpdateCollisionModel(AActor *thing)
{
	auto col = thing->collisionModel;
	if (!col)
	{
		thing->initCollisions();
		col = thing->collisionModel;
	}
	col->collider->setUserPointer(col);
	col->updateTransform();
}

btCollisionWorld::AllHitsRayResultCallback FLevelLocals::TraceCollisionHit(DVector3 &from, DVector3 &to,
                                                                           TArray<AActor *>  things)
{
	btVector3                                  bFrom(from.X, from.Y, from.Z);
	btVector3                                  bTo(to.X, to.Y, to.Z);
	btCollisionWorld::AllHitsRayResultCallback allResults(bFrom, bTo);
	if (from == to)
		return allResults;

	for (AActor *thing : things)
	{
		UpdateCollisionModel(thing);
		AddCollisionObject(thing);
	}
	collisionWorld->rayTest(bFrom, bTo, allResults);
	for (AActor *thing : things)
		RemoveCollisionObject(thing);

	return allResults;
}

//==========================================================================
//
// GetCollisionPoints
//
// Where tmthing, moving to pos, would run into 'thing'. A move is swept
// (the first contact along the way); otherwise the contacts at pos are
// returned, except those at the actor's feet.
//
//==========================================================================

TArray<DVector3> FLevelLocals::GetCollisionPoints(AActor *tmthing, AActor *thing, DVector3 &pos, bool moving, bool asOrigin)
{
	TArray<DVector3> ret;
	if (tmthing->Level != thing->Level)
	{
		Printf(TEXTCOLOR_RED "Two objects compared for GetCollisionPoints belong to different levels\n");
		return ret;
	}
	if (!collisionWorld)
	{
		Printf(TEXTCOLOR_RED "GetCollisionPoints was called without a CollisionWorld\n");
		return ret;
	}
	if (tmthing == thing)
		return ret;

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);

	btCollisionObject *a     = tmthing->collisionModel->collider;
	btCollisionObject *b     = thing->collisionModel->collider;
	btCollisionShape  *shape = a->getCollisionShape();
	DVector3           diff  = pos - tmthing->Pos();

	if (moving && shape->isConvex() && diff.LengthSquared() > 1e-6)
	{
		btTransform fromTrans = a->getWorldTransform();
		btTransform toTrans   = fromTrans;
		toTrans.setOrigin(tmthing->collisionModel->toBulletPoint(pos));
		btVector3 fromPos = fromTrans.getOrigin();
		btVector3 toPos   = toTrans.getOrigin();

		MovingConvexResultCallback closestConvex(fromPos, toPos);
		// Mesh triangles entirely under the mover's feet (e.g. tree roots below the floor) can't block it.
		closestConvex.ignoreBelowZ = min(tmthing->Z(), pos.Z) + COLLISION_SKIN;
		AddCollisionObject(thing);
		collisionWorld->convexSweepTest(static_cast<btConvexShape *>(shape), fromTrans, toTrans, closestConvex);
		RemoveCollisionObject(thing);

		if (closestConvex.hasHit())
		{
			const btVector3 &hp = closestConvex.m_hitPointWorld;
			if (col_showhits)
				P_SpawnParticle(this, DVector3(hp.x(), hp.y(), hp.z()), DVector3(0, 0, 0), DVector3(0, 0, 0),
				                PalEntry(255, 0, 255), 255.0, 350, 4., 0, 0, SPF_FULLBRIGHT);
			if (asOrigin)
			{
				btVector3 hitOrigin = fromPos.lerp(toPos, closestConvex.m_closestHitFraction);
				ret.Push(DVector3(hitOrigin.x(), hitOrigin.y(), hp.z()));
			}
			else
				ret.Push(DVector3(hp.x(), hp.y(), hp.z()));
		}
	}
	else
	{
		tmthing->collisionModel->setWorldPos(pos);
		ContactPointsCallback cb;
		AddCollisionObject(tmthing);
		AddCollisionObject(thing);
		collisionWorld->contactPairTest(a, b, cb);
		RemoveCollisionObject(tmthing);
		RemoveCollisionObject(thing);

		for (auto &point : cb.points)
		{
			DVector3 worldPoint(point.onB.x(), point.onB.y(), point.onB.z());
			if (worldPoint.Z > pos.Z + COLLISION_SKIN && worldPoint.Z < tmthing->Top())
			{
				if (col_showhits)
					P_SpawnParticle(this, worldPoint, DVector3(0, 0, 0), DVector3(0, 0, 0), PalEntry(255, 255, 255),
					                255.0, 350, 4., 0, 0, SPF_FULLBRIGHT);
				ret.Push(worldPoint);
			}
		}
	}
	return ret;
}

AllHitsConvexResultCallback FLevelLocals::ActorMovementTest(AActor *tmthing, TArray<AActor *> things,
                                                                DVector3 &newPos, bool asOrigin)
{
	AllHitsConvexResultCallback allConvex;

	UpdateCollisionModel(tmthing);
	btCollisionShape *shape = tmthing->collisionModel->getCollisionShape();
	if (!shape->isConvex())
	{
		Printf("Only convex shapes can run ActorMovementTest!");
		return allConvex;
	}
	btTransform fromTrans = tmthing->collisionModel->collider->getWorldTransform();
	btTransform toTrans   = fromTrans;
	toTrans.setOrigin(tmthing->collisionModel->toBulletPoint(newPos));

	for (AActor *thing : things)
	{
		UpdateCollisionModel(thing);
		AddCollisionObject(thing);
	}
	collisionWorld->convexSweepTest(static_cast<btConvexShape *>(shape), fromTrans, toTrans, allConvex);
	for (AActor *thing : things)
		RemoveCollisionObject(thing);

	return allConvex;
}

//==========================================================================
//
// CanCollide
//
// Whether tmthing (at pos, or where it is) touches 'thing'.
//
//==========================================================================

bool FLevelLocals::CanCollide(AActor *tmthing, AActor *thing)
{
	DVector3 pos = tmthing->Pos();
	return CanCollide(tmthing, thing, pos);
}

bool FLevelLocals::CanCollide(AActor *tmthing, AActor *thing, DVector3 &pos)
{
	if (tmthing->Level != thing->Level)
	{
		Printf(TEXTCOLOR_RED "Two objects compared for CanCollide belong to different levels\n");
		return false;
	}
	if (!collisionWorld)
	{
		Printf(TEXTCOLOR_RED "CanCollide was called without a CollisionWorld\n");
		return false;
	}

	UpdateCollisionModel(tmthing);
	tmthing->collisionModel->setWorldPos(pos);
	UpdateCollisionModel(thing);

	ContactPointsCallback cb;
	AddCollisionObject(thing);
	AddCollisionObject(tmthing);
	collisionWorld->contactPairTest(tmthing->collisionModel->collider, thing->collisionModel->collider, cb);
	RemoveCollisionObject(tmthing);
	RemoveCollisionObject(thing);
	return cb.points.Size() > 0;
}

//==========================================================================
//
// GetTop
//
// The height tmthing (at pos) would stand at on 'thing', or -INFINITY if it
// isn't standing on it. Uses the same search as the actor's own ground
// check in AActor::Tick, so that one is answered from the cache.
//
//==========================================================================

double FLevelLocals::GetTop(AActor *tmthing, AActor *thing)
{
	DVector3 pos = tmthing->Pos();
	return GetTop(tmthing, thing, pos);
}

double FLevelLocals::GetTop(AActor *tmthing, AActor *thing, DVector3 &pos)
{
	if (!collisionWorld || !thing || tmthing == thing)
		return -INFINITY;

	UpdateCollisionModel(tmthing);
	if (thing->collisionModelName == NAME_None || tmthing->collisionModel->getCollisionShape()->isConvex())
	{
		FGroundHit g = GetSupport(tmthing, thing, pos, std::max(0., -tmthing->Vel.Z) + tmthing->GetGravity());
		return g.hit ? g.z : -INFINITY;
	}

	// Actors whose own shape isn't convex can't be swept; fall back to a ray straight down.
	UpdateCollisionModel(thing);
	btVector3 from = tmthing->collisionModel->toBulletPoint(
		DVector3(pos.X, pos.Y, pos.Z + tmthing->MaxStepHeight + COLLISION_SKIN));
	btVector3 to = tmthing->collisionModel->toBulletPoint(DVector3(pos.X, pos.Y, pos.Z - tmthing->MaxStepHeight));
	AddCollisionObject(thing);
	WalkableRayResultCallback closestResults(from, to, (btScalar)tmthing->MaxSlopeSteepness);
	collisionWorld->rayTest(from, to, closestResults);
	RemoveCollisionObject(thing);

	return closestResults.hasHit() ? (double)closestResults.m_hitPointWorld.z() : -INFINITY;
}

//==========================================================================
//
// Ray helpers for the step/footing checks
//
//==========================================================================

// Ray straight along Z against 'target' only, ignoring back faces.
static bool RayZ(btCollisionWorld *world, const btCollisionObject *target, const DVector2 &xy, double fromZ,
                 double toZ, double *hitZ = nullptr, btVector3 *hitNormal = nullptr)
{
	btVector3 from(xy.X, xy.Y, fromZ), to(xy.X, xy.Y, toZ);
	btCollisionWorld::ClosestRayResultCallback cb(from, to);
	cb.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;
	world->rayTest(from, to, cb);
	if (!cb.hasHit() || cb.m_collisionObject != target)
		return false;
	if (hitZ)
		*hitZ = cb.m_hitPointWorld.z();
	if (hitNormal)
		*hitNormal = cb.m_hitNormalWorld;
	return true;
}

// Rise or fall per unit of horizontal distance along a surface with this normal.
static double SurfaceGrade(const DVector3 &normal)
{
	double nz = normal.Z < 0.05 ? 0.05 : (normal.Z > 1. ? 1. : normal.Z);
	return sqrt(1. - nz * nz) / nz;
}

static const DVector2 RayLines[] = {{1, 0}, {0.7071, 0.7071}, {0, 1}, {-0.7071, 0.7071}};

// A sweep reports its contact on the collision margin, which can sit just off the edge
// of the face itself. Find a point on the face within a unit of the contact.
static bool AnchorOnSurface(btCollisionWorld *world, const btCollisionObject *target, const DVector3 &contact,
                            double grade, DVector3 &anchor)
{
	double z;
	if (RayZ(world, target, contact.XY(), contact.Z + 1., contact.Z - 1., &z))
	{
		anchor = DVector3(contact.XY(), z);
		return true;
	}
	for (double nudge = 0.5; nudge <= 1.; nudge += 0.5)
	{
		double band = 1. + nudge * (grade + 0.25);
		for (const DVector2 &line : RayLines)
		{
			for (double side = 1.; side >= -1.; side -= 2.)
			{
				DVector2 xy = contact.XY() + line * (side * nudge);
				if (RayZ(world, target, xy, contact.Z + band, contact.Z - band, &z))
				{
					anchor = DVector3(xy, z);
					return true;
				}
			}
		}
	}
	return false;
}

//==========================================================================
//
// SweepActor
//
// Sweeps tmthing's shape from 'from' to 'to' (actor positions, i.e. what
// AActor::Pos() returns) against 'thing' and reports the first contact.
// Contacts the shape is moving away from or sliding along are ignored, as
// are mesh faces whose normal Z is below minFaceZ, mesh triangles lying
// entirely below ignoreBelowZ (if given), and contacts whose normal Z is below
// minContactZ on a face turned further up than that (an edge touched by the
// side of the shape).
//
//==========================================================================

FSweepHit FLevelLocals::SweepActor(AActor *tmthing, AActor *thing, const DVector3 &from, const DVector3 &to,
                                   double minFaceZ, double ignoreBelowZ, double minContactZ)
{
	FSweepHit result;
	if (!collisionWorld || !thing || tmthing == thing || from == to)
		return result;

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);

	AdvancedCollisionModel *model = tmthing->collisionModel;
	btCollisionShape       *shape = model->getCollisionShape();
	if (!shape->isConvex())
		return result;

	btTransform transFrom = model->collider->getWorldTransform();
	btTransform transTo   = transFrom;
	transFrom.setOrigin(model->toBulletPoint(from));
	transTo.setOrigin(model->toBulletPoint(to));

	AddCollisionObject(thing);
	MovingConvexResultCallback cb(transFrom.getOrigin(), transTo.getOrigin());
	cb.oneSidedVertical = fabs(to.Z - from.Z) > (to - from).XY().Length();
	cb.minFaceZ         = (float)minFaceZ;
	cb.ignoreBelowZ     = ignoreBelowZ;
	cb.minContactZ      = (float)minContactZ;
	collisionWorld->convexSweepTest(static_cast<btConvexShape *>(shape), transFrom, transTo, cb);
	RemoveCollisionObject(thing);

	if (cb.hasHit())
	{
		btVector3 n     = cb.m_hitNormalWorld;
		if (n.length2() > SIMD_EPSILON)
			n.normalize();
		result.hit      = true;
		result.fraction = cb.m_closestHitFraction;
		result.origin   = from + (to - from) * result.fraction;
		result.point    = DVector3(cb.m_hitPointWorld.x(), cb.m_hitPointWorld.y(), cb.m_hitPointWorld.z());
		result.normal   = DVector3(n.x(), n.y(), n.z());
		if (cb.hasFaceNormal)
		{
			result.hasFaceNormal = true;
			result.faceNormal    = DVector3(cb.faceNormal.x(), cb.faceNormal.y(), cb.faceNormal.z());
		}
	}
	return result;
}

//==========================================================================
//
// SweepFootprint
//
// Like SweepActor, but sweeps a flat disc the width of tmthing (its
// footprint) instead of its shape. Doom stands an actor on the highest
// floor anywhere under its footprint, flat up to the very edge; a rounded
// shape (a capsule) touching an edge would instead sink onto it and roll
// off. 'from' and 'to' are positions of the actor's feet. 'inset' narrows
// the disc.
//
//==========================================================================

FSweepHit FLevelLocals::SweepFootprint(AActor *tmthing, AActor *thing, const DVector3 &from, const DVector3 &to,
                                       double minFaceZ, double ignoreBelowZ, double inset)
{
	FSweepHit result;
	if (!collisionWorld || !thing || tmthing == thing || from == to)
		return result;

	UpdateCollisionModel(thing);

	const double     halfHeight = 0.5;
	const double     radius     = max(tmthing->radius - inset, 1.);
	btCylinderShapeZ disc(btVector3(radius, radius, halfHeight));
	btTransform      transFrom, transTo;
	transFrom.setIdentity();
	transTo.setIdentity();
	transFrom.setOrigin(btVector3(from.X, from.Y, from.Z + halfHeight));
	transTo.setOrigin(btVector3(to.X, to.Y, to.Z + halfHeight));

	AddCollisionObject(thing);
	MovingConvexResultCallback cb(transFrom.getOrigin(), transTo.getOrigin());
	cb.oneSidedVertical = true;
	cb.minFaceZ         = (float)minFaceZ;
	cb.ignoreBelowZ     = ignoreBelowZ;
	collisionWorld->convexSweepTest(&disc, transFrom, transTo, cb);
	RemoveCollisionObject(thing);

	if (cb.hasHit())
	{
		btVector3 n = cb.m_hitNormalWorld;
		if (n.length2() > SIMD_EPSILON)
			n.normalize();
		result.hit      = true;
		result.fraction = cb.m_closestHitFraction;
		result.origin   = from + (to - from) * result.fraction;
		result.point    = DVector3(cb.m_hitPointWorld.x(), cb.m_hitPointWorld.y(), cb.m_hitPointWorld.z());
		result.normal   = DVector3(n.x(), n.y(), n.z());
		if (cb.hasFaceNormal)
		{
			result.hasFaceNormal = true;
			result.faceNormal    = DVector3(cb.faceNormal.x(), cb.faceNormal.y(), cb.faceNormal.z());
		}
	}
	return result;
}

//==========================================================================
//
// SurfaceNormalAt
//
// For shapes without per-face normals (convex hulls): a swept shape's
// contact normal is blended at edges and corners, so cast a short ray
// straight down next to the contact (nudged away from the actor's center,
// onto the surface being stepped onto) to get the surface's own normal.
//
//==========================================================================

DVector3 FLevelLocals::SurfaceNormalAt(AActor *thing, const DVector3 &point, const DVector2 &center,
                                       const DVector3 &fallback)
{
	if (!collisionWorld || !thing || !thing->collisionModel)
		return fallback;

	DVector2 out   = point.XY() - center;
	double   len   = out.Length();
	DVector2 nudge = len > 0.01 ? out * (0.5 / len) : DVector2(0, 0);

	btVector3 from(point.X + nudge.X, point.Y + nudge.Y, point.Z + 2.);
	btVector3 to(point.X + nudge.X, point.Y + nudge.Y, point.Z - 2.);

	AddCollisionObject(thing);
	btCollisionWorld::ClosestRayResultCallback cb(from, to);
	cb.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;
	collisionWorld->rayTest(from, to, cb);
	RemoveCollisionObject(thing);

	if (cb.hasHit() && cb.m_collisionObject == thing->collisionModel->collider && cb.m_hitNormalWorld.z() > 0.)
	{
		btVector3 n = cb.m_hitNormalWorld.normalized();
		return DVector3(n.x(), n.y(), n.z());
	}
	return fallback;
}

//==========================================================================
//
// ProbeGround
//
// What tmthing would stand on at pos.XY, searching from pos.Z + up down to
// pos.Z - down, the way Doom finds an actor's floor:
//
// - Walkable surfaces count anywhere under the actor's footprint, right up
//   to their edges, however narrow they are: a thin rim or a handle holds
//   an actor up just like a one-unit-wide sector would. For a capsule this
//   is found with a flat disc (SweepFootprint), so it stands level on an
//   edge instead of rolling off it; on a slope it rests where its rounded
//   base touches the surface.
// - A steep face only counts when it is what's under the actor (it is
//   higher than any walkable ground there, and not just a wall beside it).
//   A steep face no taller than a step (see IsStepSizedFace) counts as
//   walkable.
//
//==========================================================================

FGroundHit FLevelLocals::ProbeGround(AActor *tmthing, AActor *thing, const DVector3 &pos, double up, double down)
{
	// Start a little above the step height so a surface exactly MaxStepHeight up isn't
	// already touching (and missed) at the start of the sweep.
	DVector3 from(pos.X, pos.Y, pos.Z + up + COLLISION_SKIN);
	DVector3 to(pos.X, pos.Y, pos.Z - down);
	// Mesh triangles lying entirely below the actor's floor (roots, the base of a model sunk into
	// the ground) are hidden by the floor and can't hold the actor up.
	const double belowFloor = tmthing->floorz + COLLISION_SKIN;
	const double minWalkZ   = tmthing->MaxSlopeSteepness;
	btCollisionShape *shape   = tmthing->collisionModel ? tmthing->collisionModel->getCollisionShape() : nullptr;
	const bool        capsule = shape && shape->getShapeType() == CAPSULE_SHAPE_PROXYTYPE;

	const double maxContactZ = pos.Z + up + COLLISION_SKIN; // a rounded shape touching an edge above this would creep up it

	FGroundHit ground;
	// 'where' is the point holding the actor up. One near the rim of its footprint can't lift it
	// (see FOOTPRINT_INSET); this also keeps a capsule's rounded base from riding up an edge it's
	// only standing beside.
	auto       addGround = [&](double z, const FSweepHit &h, const DVector3 &n, const DVector2 &where)
	{
		if (z > pos.Z + COLLISION_SKIN && (where - pos.XY()).Length() > tmthing->radius - FOOTPRINT_INSET)
			return;
		if (!ground.hit || z > ground.z)
		{
			ground.hit      = true;
			ground.walkable = true;
			ground.z        = z;
			ground.normal   = n;
			ground.contactZ = h.point.Z;
		}
	};
	// A capsule's rounded base rests on a surface at a point offset from its center by
	// radius * sin(slope). Touching it further out means it's on an edge, where it would sink and
	// roll off; the footprint disc (below) decides what holds it up there.
	auto restsOn = [&](const FSweepHit &h, const DVector3 &n)
	{
		return !capsule || (h.point.XY() - pos.XY()).Length() <= tmthing->radius * sqrt(max(0., 1. - n.Z * n.Z)) + 1.;
	};
	auto faceNormal = [&](const FSweepHit &h)
	{ return h.hasFaceNormal ? h.faceNormal : SurfaceNormalAt(thing, h.point, pos.XY(), h.normal); };

	// What the actor's own shape comes down on: where it rests on walkable ground (exact on
	// curved or uneven surfaces), or a steep face.
	FGroundHit steep;
	FSweepHit  edgeHit; // where the capsule rests on an edge, used if that edge is a crease (see below)
	double     edgeZ = 0;
	auto       rest = [&](double z, const FSweepHit &h, const DVector3 &n)
	{
		if (restsOn(h, n))
			addGround(z, h, n, h.point.XY());
		else if (!edgeHit.hit)
		{
			edgeHit = h;
			edgeZ   = z;
		}
	};
	FSweepHit any = SweepActor(tmthing, thing, from, to, -2., belowFloor);
	if (any.hit && any.point.Z <= maxContactZ)
	{
		FGroundHit g = ClassifyGround(tmthing, thing, pos, up, any);
		if (g.normal.Z < minWalkZ)
			steep = g;
		else
			rest(g.z, any, g.normal);
	}
	if (!ground.hit)
	{
		FSweepHit w = SweepActor(tmthing, thing, from, to, minWalkZ, belowFloor);
		if (w.hit && w.point.Z <= maxContactZ)
		{
			DVector3 n = faceNormal(w);
			if (n.Z >= minWalkZ)
				rest(w.origin.Z, w, n);
		}
	}

	// A capsule's rounded base sinks onto an edge and would roll off it. Doom stands an actor on the
	// highest floor anywhere under its footprint, level right up to the edge, so also look with a
	// flat disc the width of the actor.
	if (capsule)
	{
		// Only something higher than the ground already found matters.
		DVector3  discTo(to.X, to.Y, ground.hit ? max(to.Z, ground.z) : to.Z);
		FSweepHit flat = SweepFootprint(tmthing, thing, from, discTo, minWalkZ, belowFloor);
		if (flat.hit)
		{
			DVector3 n = faceNormal(flat);
			if (n.Z >= minWalkZ)
			{
				// The disc comes down on the highest point under the footprint. Like Doom, which
				// stands an actor on a sloped floor at its center but on the higher side of any line
				// under it:
				// - The uphill rim of a slope that carries on past it is no line: the capsule's
				//   rounded base rests on the slope instead.
				// - An edge with a drop behind it (a ledge, a stair, a rim) holds the actor at its
				//   height until the footprint is off it. Then it falls, as off a ledge in Doom.
				// - A crease in a continuous surface (where a flat top tips over into a ramp, a
				//   ridge) has no drop: the capsule rolls over it, which is found by its own sweep.
				auto planeAt = [](const DVector3 &pt, const DVector3 &nn, const DVector2 &xy)
				{ return pt.Z - (nn.X * (xy.X - pt.X) + nn.Y * (xy.Y - pt.Y)) / nn.Z; };
				const btCollisionObject *target = thing->collisionModel->collider;
				double   z      = flat.origin.Z;
				bool     hold   = true;
				DVector2 where  = flat.point.XY();
				DVector2 out    = flat.point.XY() - pos.XY();
				double   len    = out.Length();
				AddCollisionObject(thing);
				if (len >= 0.01)
				{
					double    hz;
					btVector3 hn;
					bool      continues = false;
					if (n.Z < 0.999)
					{
						// The slope carries on past the rim, and is also what's under the actor's center.
						// (Checking only past the rim, the far side of a ridge would pass for a slope the
						// actor is on, depending on which of the ridge's two faces the disc reported, and
						// the actor would bob between the two heights while standing still.)
						auto onPlane = [&](const DVector2 &xy)
						{
							double expect = planeAt(flat.point, n, xy);
							return RayZ(collisionWorld, target, xy, expect + 1., expect - 1., &hz, &hn) &&
							       hn.length2() > SIMD_EPSILON && hn.normalized().dot(btVector3(n.X, n.Y, n.Z)) > 0.999 &&
							       fabs(hz - expect) < 0.25;
						};
						continues = onPlane(flat.point.XY() + out * (1. / len)) && onPlane(pos.XY());
					}
					if (continues)
					{
						double rest = planeAt(flat.point, n, pos.XY()) + tmthing->radius * (1. / n.Z - 1.);
						if (rest < z)
							z = rest;
						where = pos.XY(); // the slope is under the actor's center
					}
					else if (RayZ(collisionWorld, target, pos.XY(), flat.point.Z + 1., flat.point.Z - 2. * tmthing->radius - 2., &hz, &hn) &&
					         hn.length2() > SIMD_EPSILON)
					{
						hn.normalize();
						DVector3 cn(hn.x(), hn.y(), hn.z());
						// A crease: the surface under the center, carried on to the contact, comes up to its
						// height (or past it: the contact may lie well beyond the crease, on the far face).
						if (cn.Z >= minWalkZ && planeAt(DVector3(pos.XY(), hz), cn, flat.point.XY()) > flat.point.Z - 1.)
						{
							hold = false;
							// The capsule rolls over the crease, resting on its edge. Report the slope under
							// the actor, so following it down (GetSupport) allows for its grade.
							if (edgeHit.hit)
								addGround(edgeZ, edgeHit, cn, pos.XY());
						}
					}
				}
				RemoveCollisionObject(thing);
				if (hold && z > pos.Z + COLLISION_SKIN && where != pos.XY())
				{
					// This would lift the actor onto an edge: it must be over it by more than
					// FOOTPRINT_INSET (see there). Check with a narrower disc.
					FSweepHit inner = SweepFootprint(tmthing, thing, from, DVector3(to.X, to.Y, z - 0.5), minWalkZ,
					                                 belowFloor, FOOTPRINT_INSET);
					hold = inner.hit;
				}
				if (hold)
					addGround(z, flat, n, pos.XY());
			}
		}
	}

	if (steep.hit)
	{
		// Walkable ground at the actor's feet wins over a steep face: that's just something it's
		// standing next to (a tree trunk that leans back slightly).
		bool groundAtFeet = ground.hit && ground.z >= pos.Z - (1. + COLLISION_SKIN);
		if (!groundAtFeet && (!ground.hit || steep.z > ground.z + COLLISION_SKIN))
			return steep;
	}
	return ground;
}

//==========================================================================
//
// ClassifyGround
//
// Turns the result of a downward sweep of tmthing's shape into ground: the
// surface's normal, and whether the actor can stand on it.
//
//==========================================================================

FGroundHit FLevelLocals::ClassifyGround(AActor *tmthing, AActor *thing, const DVector3 &pos, double up, const FSweepHit &hit)
{
	FGroundHit result;
	result.hit      = true;
	result.z        = hit.origin.Z;
	result.contactZ = hit.point.Z;
	result.normal   = hit.hasFaceNormal ? hit.faceNormal : SurfaceNormalAt(thing, hit.point, pos.XY(), hit.normal);
	result.walkable = result.normal.Z >= tmthing->MaxSlopeSteepness ||
	                  IsStepSizedFace(tmthing, thing, hit.point, result.normal);
	return result;
}

//==========================================================================
//
// IsStepSizedFace
//
// A surface too steep to walk on still behaves like a step when the steep
// face as a whole is no taller than MaxStepHeight: a curb, a low ridge, the
// lip of a model. Actors can climb onto it and stand on it without sliding.
//
// The face is measured from its foot to its crest, independent of where the
// actor touches it, so a tall slope is never climbable or standable, not
// even near its top. Starting on the face at the contact, short rays are
// cast straight down every unit: downhill until the face ends, levels out
// or turns back up (the foot), then uphill until it levels out into
// walkable ground or tops out and drops away (the crest).
//
//==========================================================================

bool FLevelLocals::IsStepSizedFace(AActor *tmthing, AActor *thing, const DVector3 &contact, const DVector3 &normal)
{
	if (!collisionWorld || !thing || tmthing == thing || !thing->collisionModel)
		return false;

	DVector2 downhill(normal.X, normal.Y);
	double   len = downhill.Length();
	if (len < 0.001)
		return false;
	downhill = downhill * (1. / len);

	const double limit = tmthing->MaxStepHeight + COLLISION_SKIN;

	// The furthest a face that is only just too steep can run horizontally while rising 'limit'.
	// Any steeper face covers that height in less distance.
	double s = tmthing->MaxSlopeSteepness;
	s        = s < 0.05 ? 0.05 : (s > 0.995 ? 0.995 : s);
	const double maxRun = limit * s / sqrt(1. - s * s) + 2.;

	const btCollisionObject *target = thing->collisionModel->collider;
	const double             minNZ  = tmthing->MaxSlopeSteepness;
	auto castRay = [&](const DVector2 &xy, double fromZ, double toZ, double &hitZ, bool &walkable)
	{
		btVector3 n;
		if (!RayZ(collisionWorld, target, xy, fromZ, toZ, &hitZ, &n))
			return false;
		walkable = n.length2() > SIMD_EPSILON && n.normalized().z() >= minNZ;
		return true;
	};

	AddCollisionObject(thing);

	bool     result = false;
	DVector3 start;
	// Without a point on the face itself (e.g. the contact is just past the end of the model)
	// the rays below would miss and look like a crest, so don't call it a step.
	if (AnchorOnSurface(collisionWorld, target, contact, SurfaceGrade(normal), start))
	{
		do
		{
			// Foot: follow the face downhill.
			double foot = start.Z;
			for (double d = 1.; d <= maxRun; d += 1.)
			{
				double z;
				bool   walkable;
				if (!castRay(start.XY() + downhill * d, foot + 1., foot - limit - 1., z, walkable))
					break; // the face ends here (edge of the model)
				if (walkable || z > foot + 0.5)
				{
					foot = min(foot, z); // levels out, or turns back up
					break;
				}
				foot = z;
				if (start.Z - foot > limit)
					break;
			}
			const double topLimit = foot + limit;
			if (start.Z > topLimit)
				break; // already more than a step above the foot: a tall face

			// Crest: follow the face uphill.
			double lastZ = start.Z;
			for (double d = 1.; d <= maxRun; d += 1.)
			{
				DVector2 xy = start.XY() - downhill * d;
				double   z;
				bool     walkable;
				// Start a full step above the limit: a face that crosses the limit between samples is then
				// still hit from above, instead of the ray starting inside the model and seeing its underside.
				bool hit = castRay(xy, topLimit + limit, foot - 1., z, walkable);

				if (hit && z > topLimit)
					break; // rises more than a step above the foot

				if (!hit || z < lastZ - 0.5)
				{
					// The surface dropped away: either we crossed the crest, or the face rises above
					// where this ray started and we're looking underneath it. A ray up tells them apart.
					double upZ;
					bool   upWalkable;
					result = !castRay(xy, lastZ, topLimit + limit, upZ, upWalkable);
					break;
				}
				if (walkable)
				{
					result = true; // levels out into walkable ground within step height
					break;
				}
				lastZ = z;
			}
			// Running out of distance while still steep and rising means the face is taller than a step.
		} while (false);
	}

	RemoveCollisionObject(thing);
	return result;
}

//==========================================================================
//
// GetSupport
//
// What tmthing at pos is standing on, with the step, slope and snap rules
// applied. hit == false means 'thing' doesn't support it (it should fall).
//
// - Walkable surfaces up to MaxStepHeight above are stepped onto.
// - Steep surfaces never lift the actor; it is held at its current height
//   with walkable == false so the caller can make it slide.
// - Surfaces below are followed only if the actor was on the ground (or
//   sliding) and not moving up, and only as far as walking along that
//   surface could have taken it this tic. A real ledge still means falling.
//
// Actors without a collision model are boxes, as in Doom: their top holds
// up anything whose box overlaps theirs.
//
// extraDrop widens the downward search, e.g. by the actor's fall speed.
//
// The same question is usually asked twice in a tic (P_CheckOnmobj, then
// AActor::Tick), so the last answer is kept.
//
//==========================================================================

FGroundHit FLevelLocals::GetSupport(AActor *tmthing, AActor *thing, const DVector3 &pos, double extraDrop)
{
	static struct
	{
		FLevelLocals *level = nullptr;
		AActor       *tmthing = nullptr, *thing = nullptr;
		int           time = -1;
		DVector3      pos, vel, thingPos;
		DRotator      thingAngles;
		double        extraDrop = 0, height = 0, floorz = 0;
		int           flags2 = 0, colFlags = 0;
		FGroundHit    result;
	} cache;
	const int flags2   = tmthing->flags2 & MF2_ONMOBJ;
	const int colFlags = tmthing->collisionFlags & COLFLAG_SLIDING;
	if (cache.level == this && cache.tmthing == tmthing && cache.thing == thing && cache.time == maptime &&
	    cache.pos == pos && cache.extraDrop == extraDrop && cache.vel == tmthing->Vel && cache.height == tmthing->Height &&
	    cache.floorz == tmthing->floorz && cache.flags2 == flags2 && cache.colFlags == colFlags &&
	    cache.thingPos == thing->Pos() && cache.thingAngles == thing->Angles)
		return cache.result;

	bool grounded = ((tmthing->flags2 & MF2_ONMOBJ) || (tmthing->collisionFlags & COLFLAG_SLIDING)) &&
	                tmthing->Vel.Z <= 0 && !(tmthing->flags & MF_NOGRAVITY);
	double speed      = tmthing->Vel.XY().Length();
	double restingGap = 1. + COLLISION_SKIN + extraDrop;

	FGroundHit g;
	if (thing->collisionModelName == NAME_None)
	{
		double blockdist = thing->radius + tmthing->radius;
		double top       = thing->Top();
		if (fabs(thing->X() - pos.X) < blockdist && fabs(thing->Y() - pos.Y) < blockdist &&
		    top <= pos.Z + tmthing->MaxStepHeight + COLLISION_SKIN && top >= pos.Z - restingGap)
		{
			g.hit      = true;
			g.z        = top;
			g.contactZ = top;
		}
	}
	else
	{
		// Look as far down as walking down a walkable slope could have taken the actor this tic.
		// Only if nothing is there, look further, for a steeper surface to slide down.
		double s         = clamp(tmthing->MaxSlopeSteepness, 0.05, 1.);
		double walkGrade = sqrt(1. - s * s) / s;
		double down      = restingGap + (grounded ? speed * walkGrade * 1.5 : 0.);
		g                = ProbeGround(tmthing, thing, pos, tmthing->MaxStepHeight, down);
		if (!g.hit && grounded && speed > 0 && walkGrade < MAX_SNAP_SLOPE)
			g = ProbeGround(tmthing, thing, pos, -down, restingGap + speed * MAX_SNAP_SLOPE * 1.5);
	}

	if (g.hit)
	{
		double rise = g.z - pos.Z;
		if (g.z < tmthing->floorz - COLLISION_SKIN)
		{
			// Nothing below the actor's floor can hold it up: the floor does. (Otherwise a steep face
			// beside an actor standing on the floor, found further down, would make it slide.)
			g = FGroundHit();
		}
		else if (rise > COLLISION_SKIN)
		{
			if (!g.walkable)
				g.z = pos.Z; // pressed against a steep face: never lift onto it
			else if (thing->collisionModelName != NAME_None && Overlaps(tmthing, thing, DVector3(pos.XY(), g.z + COLLISION_SKIN), COLLISION_SKIN))
				g = FGroundHit(); // a step with no room above it (the next plank of a stack) can't be stepped onto
		}
		else if (-rise > restingGap)
		{
			double nz    = clamp(g.normal.Z, 0.01, 1.);
			double slope = min(sqrt(1. - nz * nz) / nz, MAX_SNAP_SLOPE);
			// Follow a slope down, but drop off a ledge (fall), as Doom does.
			if (!grounded || -rise > restingGap + speed * slope * 1.5)
				g = FGroundHit();
		}
	}

	cache.level       = this;
	cache.tmthing     = tmthing;
	cache.thing       = thing;
	cache.time        = maptime;
	cache.pos         = pos;
	cache.extraDrop   = extraDrop;
	cache.vel         = tmthing->Vel;
	cache.height      = tmthing->Height;
	cache.floorz      = tmthing->floorz;
	cache.flags2      = flags2;
	cache.colFlags    = colFlags;
	cache.thingPos    = thing->Pos();
	cache.thingAngles = thing->Angles;
	cache.result      = g;
	return g;
}

//==========================================================================
//
// Overlaps
//
// True if tmthing placed at pos would penetrate 'thing' by more than
// 'tolerance'. Surfaces that are only touching don't count, nor do contacts
// below minZ.
//
//==========================================================================

bool FLevelLocals::Overlaps(AActor *tmthing, AActor *thing, const DVector3 &pos, double tolerance, double minZ)
{
	if (!collisionWorld || !thing || tmthing == thing)
		return false;

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);
	tmthing->collisionModel->setWorldPos(pos);

	ContactPointsCallback cb;

	AddCollisionObject(thing);
	AddCollisionObject(tmthing);
	collisionWorld->contactPairTest(tmthing->collisionModel->collider, thing->collisionModel->collider, cb);
	RemoveCollisionObject(tmthing);
	RemoveCollisionObject(thing);

	tmthing->collisionModel->setWorldPos(tmthing->Pos());

	for (auto &point : cb.points)
	{
		double z = (point.onA.z() + point.onB.z()) / 2.;
		if (point.distance < -tolerance && z >= minZ)
			return true;
	}
	return false;
}

//==========================================================================
//
// CanGrowTo
//
// Whether tmthing can grow in place from fromHeight to toHeight (standing
// up from a crouch) without its head entering 'thing'. Three tests, because
// each one alone misses something:
//
// 1. Sweep the grown shape up into place, starting with its top level with
//    the current top (sweeping the current shape would trace the corners of a
//    flat-topped crouched cylinder through space the standing capsule's
//    rounded top never fills).
// 2. The grown shape must not overlap 'thing'. A sweep doesn't see geometry
//    it starts inside of (e.g. when a crouched capsule turns back from a
//    cylinder into a capsule partway through standing up).
// 3. Thin geometry (single-sided cards, twigs) can slip past both, so cast
//    rays straight up through the space the top of the actor grows into,
//    counting either side of a face.
//
// Only contacts in the upper half of the body count: that's the part that
// moves into new space. Something brushing the actor's side or feet is
// already there while crouched and doesn't stop it standing.
//
//==========================================================================

bool FLevelLocals::CanGrowTo(AActor *tmthing, AActor *thing, double fromHeight, double toHeight)
{
	const double rise = toHeight - fromHeight;
	if (rise <= 0 || !collisionWorld || !thing || tmthing == thing || !thing->collisionModel)
		return true;

	const double   savedHeight = tmthing->Height;
	const DVector3 pos         = tmthing->Pos();
	const double   headZ       = pos.Z + fromHeight * 0.5;
	bool           fits        = true;

	tmthing->Height = toHeight;
	FSweepHit hit   = SweepActor(tmthing, thing, pos - DVector3(0, 0, rise), pos);
	if (hit.hit && hit.point.Z >= headZ && hit.fraction * rise < rise - COLLISION_SKIN)
		fits = false;

	if (fits && Overlaps(tmthing, thing, pos, COLLISION_SKIN, headZ))
		fits = false;
	tmthing->Height = savedHeight;

	if (fits)
	{
		const bool   capsule = tmthing->collisionModelShape == FName("capsule");
		const double r       = tmthing->radius;
		// How far the top of the shape curves down at horizontal distance d from its center.
		// (Flat while the actor is shorter than its diameter: see CapsuleCollisionModel::initShape.)
		auto dome = [&](double height, double d)
		{
			if (!capsule || height < 2. * r)
				return 0.;
			return d >= r ? r : r - sqrt(r * r - d * d);
		};

		UpdateCollisionModel(thing);
		const btCollisionObject *target = thing->collisionModel->collider;
		static const double      rings[] = {0., 0.5, 0.85};

		AddCollisionObject(thing);
		for (double ring : rings)
		{
			int count = ring == 0. ? 1 : 8;
			for (int i = 0; i < count && fits; i++)
			{
				double   ang   = i * 0.7853981633974483;
				double   d     = ring * r;
				DVector2 xy    = pos.XY() + DVector2(cos(ang) * d, sin(ang) * d);
				double   fromZ = pos.Z + fromHeight - dome(fromHeight, d) - 0.5;
				double   toZ   = pos.Z + toHeight - dome(toHeight, d);
				if (toZ <= fromZ)
					continue;
				btVector3 rayFrom(xy.X, xy.Y, fromZ), rayTo(xy.X, xy.Y, toZ);
				btCollisionWorld::ClosestRayResultCallback cb(rayFrom, rayTo);
				collisionWorld->rayTest(rayFrom, rayTo, cb);
				if (cb.hasHit() && cb.m_collisionObject == target)
					fits = false;
			}
		}
		RemoveCollisionObject(thing);
	}
	return fits;
}

//==========================================================================
//
// GetCollisionBox
//
// The space a thing's collision shape can occupy. A 3D model is placed
// around the actor's origin and turned by its angles, so it can reach below
// the origin, above Height and past the radius; the classic bounding-box
// shortcuts (is the mover entirely above / below / beside it?) must use this
// instead, or they skip the real collision test.
//
//==========================================================================

void FLevelLocals::GetCollisionBox(AActor *thing, DVector3 &mins, DVector3 &maxs)
{
	mins = DVector3(thing->X() - thing->radius, thing->Y() - thing->radius, thing->Z());
	maxs = DVector3(thing->X() + thing->radius, thing->Y() + thing->radius, thing->Top());
	if (thing->collisionModelName == NAME_None || !collisionWorld)
		return;

	UpdateCollisionModel(thing);
	btVector3 mn, mx;
	thing->collisionModel->getCollisionShape()->getAabb(thing->collisionModel->collider->getWorldTransform(), mn, mx);
	mins = DVector3(min(mins.X, (double)mn.x()), min(mins.Y, (double)mn.y()), min(mins.Z, (double)mn.z()));
	maxs = DVector3(max(maxs.X, (double)mx.x()), max(maxs.Y, (double)mx.y()), max(maxs.Z, (double)mx.z()));
}

btTriangleMesh *MeshCollisionModel::fromOBJModel(FName filename, ObjModelInfo &info, TArray<btVector3> *faceNormals,
                                                 TArray<btVector3> *triVerts)
{
	btTriangleMesh *trimesh = new btTriangleMesh();
	int             lastLump = 0;
	int lump = fileSystem.FindLumpFullName(filename.GetChars(), &lastLump, false);


	FScanner sc(lump);

	TArray<btVector3> verts;
	while (sc.GetString())
	{
		if (strlen(sc.String) == 1 && sc.String[0] == 'v')
		{
			sc.MustGetFloat();
			double x = sc.Float;
			sc.MustGetFloat();
			double y = sc.Float;
			sc.MustGetFloat();
			double z = sc.Float;
			verts.Push(btVector3(x,-z,y));
			info.maxHeight = max(y, info.maxHeight);
			info.maxRadius = max(info.maxRadius, abs(x));
			info.maxRadius = max(info.maxRadius, abs(z));
		}
		else if (strlen(sc.String) == 1 && sc.String[0] == 'f')
		{
			// Faces can have any number of vertices (Blender exports quads by default).
			// Read the whole line and triangulate it as a fan.
			TArray<int> ind;
			while (sc.GetString())
			{
				if (sc.Crossed)
				{
					sc.UnGet();
					break;
				}
				FString         s  = FString(sc.String);
				TArray<FString> ss = s.Split("/");
				int             i  = ss.Size() ? (int)ss[0].ToLong() : 0;
				if (i < 0)
					i += verts.Size() + 1; // negative indices count back from the last vertex
				ind.Push(i);
			}

			for (unsigned i = 2; i < ind.Size(); i++)
			{
				int a = ind[0], b = ind[i - 1], c = ind[i];
				if (a < 1 || b < 1 || c < 1 || a > (int)verts.Size() || b > (int)verts.Size() || c > (int)verts.Size())
				{
					Printf(TEXTCOLOR_RED "%s: face references a missing vertex\n", filename.GetChars());
					continue;
				}
				trimesh->addTriangle(verts[a - 1], verts[b - 1], verts[c - 1], true);
				if (triVerts)
				{
					triVerts->Push(verts[a - 1]);
					triVerts->Push(verts[b - 1]);
					triVerts->Push(verts[c - 1]);
				}
				if (faceNormals)
				{
					// Counter-clockwise winding faces out, as exported by Blender and most tools.
					btVector3 n = (verts[b - 1] - verts[a - 1]).cross(verts[c - 1] - verts[a - 1]);
					faceNormals->Push(n.length2() > SIMD_EPSILON ? n.normalized() : btVector3(0, 0, 0));
				}
			}
		}
	}
#ifdef _DEBUG
	Printf("%s: Pushed %d verts and %d tris to trimesh\n", filename.GetChars(), verts.size(), trimesh->getNumTriangles());
#endif
	return trimesh;
}

btConvexHullShape *ConvexHullCollisionModel::fromOBJModel(FName filename, ObjModelInfo &info)
{
	btConvexHullShape *hullshape = new btConvexHullShape();
	int             lastLump = 0;
	int             lump     = fileSystem.FindLumpFullName(filename.GetChars(), &lastLump, false);

	FScanner sc(lump);

	while (sc.GetString())
	{
		if (strlen(sc.String) == 1 && sc.String[0] == 'v')
		{
			sc.MustGetFloat();
			double x = sc.Float;
			sc.MustGetFloat();
			double y = sc.Float;
			sc.MustGetFloat();
			double z = sc.Float;
			hullshape->addPoint(btVector3(x, -z, y), false); // bounds computed once below
			info.maxHeight = max(y, info.maxHeight);
			info.maxRadius = max(info.maxRadius, abs(x));
			info.maxRadius = max(info.maxRadius, abs(z));
		}
	}
	hullshape->recalcLocalAabb();
#ifdef _DEBUG
	Printf("%s: Pushed %d verts to convex hull\n", filename.GetChars(), hullshape->getNumPoints());
#endif
	return hullshape;
}

//==========================================================================
//
// Shared collision models
//
// Every actor that uses the same OBJ shares one parsed mesh and one Bullet
// shape; only the collision object (position and rotation) is per actor.
// Loading and building the model for each actor separately made levels with
// thousands of such actors very slow to load and very large in memory.
//
//==========================================================================

const FMeshCollisionData &MeshCollisionModel::GetShared(FName filename)
{
	static TMap<FName, FMeshCollisionData *> cache;
	if (FMeshCollisionData **found = cache.CheckKey(filename))
		return **found;

	auto data     = new FMeshCollisionData;
	data->trimesh = fromOBJModel(filename, data->info, &data->faceNormals, &data->triVerts);
	data->shape   = new btBvhTriangleMeshShape(data->trimesh, true);
	data->shape->setMargin(0.1f);
	cache[filename] = data;
	return *data;
}

btConvexHullShape *ConvexHullCollisionModel::GetShared(FName filename, ObjModelInfo &info)
{
	struct Entry
	{
		btConvexHullShape *shape = nullptr;
		ObjModelInfo       info;
	};
	static TMap<FName, Entry> cache;
	if (Entry *found = cache.CheckKey(filename))
	{
		info = found->info;
		return found->shape;
	}

	Entry entry;
	entry.shape = fromOBJModel(filename, entry.info);
	entry.shape->setMargin(0.05f);
	cache[filename] = entry;
	info            = entry.info;
	return entry.shape;
}

void AActor::initCollisions()
{
	if (collisionModelName != NULL && collisionModelName != "")
	{
		ObjModelInfo info;
		if (collisionModelShape == "hull")
		{
			collisionModel = new ConvexHullCollisionModel(collisionModelName, info);
		}
		else
		{
			collisionModel = new MeshCollisionModel(collisionModelName, info);
		}
		if (collisionModelAutoRadius)
		{
			radius = ceil(max(info.maxRadius, info.maxHeight/2.) * 1.05);
			Height = ceil(max(info.maxRadius, info.maxHeight) * 1.05);
			
#ifdef _DEBUG
			Printf("automatic bounds set to %f by %f\n", radius, Height);
#endif
		}
	}
	else
	{
		if (collisionModelShape == "cylinder")
			collisionModel = new CylinderCollisionModel(radius, Height);
		else if (collisionModelShape == "capsule")
			collisionModel = new CapsuleCollisionModel(radius, Height);
		else
			collisionModel = new DoomStandardCollisionModel(radius, Height);
	}

	collisionModel->owner = this;
}
