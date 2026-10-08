#pragma once
#include <bullet/btBulletCollisionCommon.h>
#include <actor.h>

class AdvancedCollisionModel
{
  public:
	btCollisionObject *collider;
	AActor            *owner;

	virtual DVector3 toWorldPoint(btVector3 b)
	{
		return DVector3(b.x(), b.y(), b.z());
	}

	virtual btVector3 toBulletPoint(DVector3 w)
	{
		return btVector3(w.X,w.Y,w.Z);
	}

	virtual void setWorldPos(DVector3 pos);
	virtual void setRotation(DRotator d);

	virtual void updateTransform();

	btCollisionShape *getCollisionShape()
	{
		return collider->getCollisionShape();
	}

	// Triangle-mesh models: the world-space front-face normal of one of their triangles,
	// taken from its winding. Other shapes don't have per-face data and return false.
	virtual bool getFaceNormal(int part, int triangle, btVector3 &normal) const
	{
		return false;
	}

	// Triangle-mesh models: the world-space height of a triangle's highest corner.
	virtual bool getTriangleTopZ(int part, int triangle, double &topZ) const
	{
		return false;
	}
};

class CylinderCollisionModel : public AdvancedCollisionModel
{
  public:
	double radius, height;
	CylinderCollisionModel(double radius, double height)
	{
		this->radius = radius;
		this->height = height;
		btCylinderShape *bs = new btCylinderShape(btVector3(radius, height/2., radius ));
		bs->setMargin(0.1);
		collider     = new btCollisionObject();
		collider->setCollisionShape(bs);
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}

	void setWorldPos(DVector3 pos) override
	{
		btTransform transform = collider->getWorldTransform();
		// cylinder's height is along the Y axis where Doom's is on the Z
		transform.setRotation(btQuaternion(0, SIMD_HALF_PI, 0));
		transform.setOrigin(toBulletPoint(pos));
		collider->setWorldTransform(transform);
	}

	virtual btVector3 toBulletPoint(DVector3 w)
	{
		return btVector3(w.X, w.Y, w.Z + height / 2);
	}
};

class CapsuleCollisionModel : public AdvancedCollisionModel
{
  public:
	double radius, height;
	CapsuleCollisionModel(double radius, double height)
	{
		this->radius        = radius;
		this->height        = height;

		collider = new btCollisionObject();
		collider->setCollisionShape(initShape());
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}

	void setWorldPos(DVector3 pos) override
	{
		btTransform transform = collider->getWorldTransform();
		// cylinder's height is along the Y axis where Doom's is on the Z
		transform.setRotation(btQuaternion(0, SIMD_HALF_PI, 0));
		transform.setOrigin(toBulletPoint(pos));
		collider->setWorldTransform(transform);
	}

	virtual btVector3 toBulletPoint(DVector3 w)
	{
		return btVector3(w.X, w.Y, w.Z + height / 2.f);
	}

	virtual DVector3 toWorldPoint(btVector3 b)
	{
		return DVector3(b.x(), b.y(), b.z() - height / 2.f);
	}

	void updateTransform() override;

	btConvexShape *initShape()
	{
		// A capsule can't be shorter than its own diameter (e.g. a crouching player). Use a
		// cylinder of the same radius instead: shrinking the radius would let a crouched actor
		// squeeze closer to things than it can stand, then stand up into them.
		if (height < 2. * radius)
		{
			auto bs = new btCylinderShape(btVector3(radius, height / 2., radius));
			bs->setMargin(0.1);
			return bs;
		}
		auto bs = new btCapsuleShape(radius, height - 2. * radius);
		bs->setMargin(0.1);
		return bs;
	}
};

//regular doom collisi
// on model is a rectangular prism that can't rotate
class DoomStandardCollisionModel : public AdvancedCollisionModel
{
  public:
	double radius, height;

	// Doom's bounding box never turns or tilts with the actor (looking up or down would
	// otherwise tilt the player's collision box).
	void setRotation(DRotator d) override
	{
	}
	DoomStandardCollisionModel(double radius, double height)
	{
		this->radius = radius;
		this->height = height;
		collider     = new btCollisionObject();
		collider->setCollisionShape(initShape());
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}

	btBoxShape *initShape()
	{
		auto bs = new btBoxShape(btVector3(radius, radius, height / 2.f));
		bs->setMargin(0.05);
		return bs;
	}

	void setWorldPos(DVector3 pos) override
	{
		btTransform transform = collider->getWorldTransform();
		transform.setOrigin(toBulletPoint(pos));
		collider->setWorldTransform(transform);		
	}

	void updateTransform() override;

	virtual btVector3 toBulletPoint(DVector3 w)
	{
		return btVector3(w.X, w.Y, w.Z + height / 2.f);
	}
};

struct ObjModelInfo
{
	ObjModelInfo() : maxHeight(0.), maxRadius(0.) { }
	double maxHeight, maxRadius;
};

// A triangle-mesh collision model loaded from an OBJ. Loaded once per OBJ and shared by every
// actor that uses it (Bullet lets any number of collision objects share one shape).
struct FMeshCollisionData
{
	btTriangleMesh         *trimesh = nullptr;
	btBvhTriangleMeshShape *shape   = nullptr;
	TArray<btVector3>       faceNormals; // model-space front-face normal of each triangle, in mesh order
	TArray<btVector3>       triVerts;    // model-space corners of each triangle, three per triangle
	ObjModelInfo            info;
};

class ConvexHullCollisionModel : public AdvancedCollisionModel
{
  public:
	static btConvexHullShape *fromOBJModel(FName filename, ObjModelInfo &info);
	static btConvexHullShape *GetShared(FName filename, ObjModelInfo &info);

	ConvexHullCollisionModel(FName filename, ObjModelInfo &info)
	{
		btConvexHullShape *bs = GetShared(filename, info);
		collider = new btCollisionObject();
		collider->setCollisionShape(bs);
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}
};

class MeshCollisionModel : public AdvancedCollisionModel
{
  public:
	const FMeshCollisionData *data; // shared with every actor using the same OBJ

	static btTriangleMesh *fromOBJModel(FName filename, ObjModelInfo &info, TArray<btVector3> *faceNormals = nullptr,
	                                    TArray<btVector3> *triVerts = nullptr);
	static const FMeshCollisionData &GetShared(FName filename);

	 MeshCollisionModel(FName filename, ObjModelInfo &info)
	{
		data     = &GetShared(filename);
		info     = data->info;
		collider = new btCollisionObject();
		collider->setCollisionShape(data->shape);
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}

	bool getFaceNormal(int part, int triangle, btVector3 &normal) const override
	{
		if (part != 0 || triangle < 0 || triangle >= (int)data->faceNormals.Size())
			return false;
		normal = collider->getWorldTransform().getBasis() * data->faceNormals[triangle];
		return true;
	}

	bool getTriangleTopZ(int part, int triangle, double &topZ) const override
	{
		if (part != 0 || triangle < 0 || triangle * 3 + 2 >= (int)data->triVerts.Size())
			return false;
		const btTransform &t = collider->getWorldTransform();
		topZ                 = -1e30;
		for (int i = 0; i < 3; i++)
		{
			double z = (t * data->triVerts[triangle * 3 + i]).z();
			if (z > topZ)
				topZ = z;
		}
		return true;
	}
};



// Collects the contact points a contactPairTest finds.
struct ContactPointsCallback : public btCollisionWorld::ContactResultCallback
{
	struct Point
	{
		btVector3 onA, onB;
		btScalar  distance; // negative when penetrating
	};
	TArray<Point> points;

	btScalar addSingleResult(btManifoldPoint &cp, const btCollisionObjectWrapper *, int, int,
	                         const btCollisionObjectWrapper *, int, int) override
	{
		points.Push({cp.m_positionWorldOnA, cp.m_positionWorldOnB, cp.m_distance1});
		return 1.f;
	}
};

// Extra clearance used by the ground/step logic. Bullet's collision margins inflate
// mesh and hull shapes slightly, so exact comparisons against MaxStepHeight and
// "resting on" heights would otherwise fail by a fraction of a unit.
constexpr double COLLISION_SKIN = 0.25;

// Steepest surface (as rise over run, ~85 degrees) an actor will stay glued to while
// walking or sliding down it, instead of falling off and bouncing.
constexpr double MAX_SNAP_SLOPE = 11.43;

// Ground touched only by the outermost FOOTPRINT_INSET of an actor's footprint can hold it
// up but can't lift it: to step up onto something, the actor must be over it by more than
// that, i.e. by more than Overlaps' tolerance. Then anything right above that surface (the
// next plank of a barricade, a desk top over a drawer handle) overlaps the actor's body
// too, and the step is refused for lack of room, as Doom refuses a floor whose ceiling is
// too low. Otherwise an actor merely grazing a stack of ledges would climb it, a ledge per
// tic. P_TryMove looks for steps a little further into what it ran into, so walking into
// a step slowly still climbs it.
constexpr double FOOTPRINT_INSET = 0.75;

// Sliding down surfaces too steep to stand on: each tic the actor's horizontal speed is
// scaled by SLIDE_FRICTION before the part of gravity along the slope is added, so the
// slide settles at a steady speed instead of building up indefinitely.
constexpr double SLIDE_FRICTION = 0.9;

struct FSweepHit
{
	bool     hit      = false;
	double   fraction = 1.;
	DVector3 origin;            // actor position (same space as AActor::Pos()) at the moment of contact
	DVector3 point;             // contact point in world space
	DVector3 normal = {0, 0, 1}; // contact normal pointing away from the surface that was hit
	bool     hasFaceNormal = false; // the hit was on a mesh triangle whose own facing is known
	DVector3 faceNormal;            // that triangle's front-face normal
};

struct FGroundHit
{
	bool     hit      = false;
	double   z        = -INFINITY;   // Z an actor's origin would rest at on this surface
	DVector3 normal   = {0, 0, 1};   // the surface's own normal
	bool     walkable = true;        // flat enough to stand on, or a steep face no taller than MaxStepHeight
	double   contactZ = -INFINITY;   // height of the point the actor touches
};

// Closest-hit convex sweep that ignores contacts the shape is moving away from or
// sliding along. Without this, a shape that starts out touching a surface (for
// example resting on a slope) reports a hit at fraction 0 and can never leave it.
struct MovingConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
	btVector3 dir;
	bool      oneSidedVertical = false; // up/down sweeps pass through the back of mesh faces
	float     minFaceZ         = -2.f;  // ignore mesh faces whose normal Z is below this (walls, for a floor search)
	float     minContactZ      = -2.f;  // ignore contacts with a normal Z below this on faces turned further up (edges grazed)
	double    ignoreBelowZ     = -1e30; // ignore mesh triangles lying entirely below this height
	bool      hasFaceNormal    = false; // set for the accepted hit when it is on a known mesh triangle
	btVector3 faceNormal;

	MovingConvexResultCallback(const btVector3 &from, const btVector3 &to)
		: btCollisionWorld::ClosestConvexResultCallback(from, to), dir(to - from)
	{
	}

	btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult, bool normalInWorldSpace) override
	{
		btVector3 n = normalInWorldSpace
		                  ? convexResult.m_hitNormalLocal
		                  : convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
		btScalar len = dir.length() * n.length();
		if (len <= SIMD_EPSILON || n.dot(dir) >= -0.01f * len)
			return m_closestHitFraction;

		// Bullet treats mesh triangles as two-sided. Look up the triangle's own facing.
		btVector3 fn;
		bool      known = false;
		if (convexResult.m_localShapeInfo)
		{
			auto model = static_cast<const AdvancedCollisionModel *>(convexResult.m_hitCollisionObject->getUserPointer());
			int  part  = convexResult.m_localShapeInfo->m_shapePart;
			int  tri   = convexResult.m_localShapeInfo->m_triangleIndex;
			known      = model && model->getFaceNormal(part, tri, fn);

			// Geometry lying entirely below the given height (roots under the floor, the ground the
			// actor is already standing on) can't block a sideways move or hold the actor up.
			double topZ;
			if (model && ignoreBelowZ > -1e29 && model->getTriangleTopZ(part, tri, topZ) && topZ < ignoreBelowZ)
				return m_closestHitFraction;
		}
		// Moving down, only a face clearly turned up can stop the sweep: coming down onto the back
		// of a face means coming at it from inside or underneath, so the underside of a ledge can't
		// act as a floor, and vertical faces can't hold anything up even when an edge contact reports
		// a tilted normal. Moving up, every face counts, so heads can't pass up through thin or
		// single-sided geometry.
		if (known && oneSidedVertical && dir.z() < 0 && fn.z() < 0.05f)
			return m_closestHitFraction;
		if (known && fn.z() < minFaceZ * fn.length())
			return m_closestHitFraction;
		// A shape touching the edge of a face with its side, rather than resting on the face.
		if (known && n.z() < minContactZ * n.length() && fn.length2() > SIMD_EPSILON && fn.normalized().z() > n.z() / n.length() + 0.25f)
			return m_closestHitFraction;

		btScalar result = btCollisionWorld::ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
		hasFaceNormal   = known;
		if (known)
			faceNormal = fn.length2() > SIMD_EPSILON ? fn.normalized() : fn;
		return result;
	}
};

// Closest-hit ray that ignores surfaces too steep to stand on.
struct WalkableRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
	btScalar minNormalZ;

	WalkableRayResultCallback(const btVector3 &from, const btVector3 &to, btScalar minNormalZ)
		: btCollisionWorld::ClosestRayResultCallback(from, to), minNormalZ(minNormalZ)
	{
	}

	btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult, bool normalInWorldSpace) override
	{
		btVector3 n = normalInWorldSpace
		                  ? rayResult.m_hitNormalLocal
		                  : rayResult.m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
		if (n.z() < minNormalZ)
			return m_closestHitFraction;
		return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
	}
};

struct AllHitsConvexResultCallback : public btCollisionWorld::ConvexResultCallback
{
	struct Hit
	{
		btCollisionObject *collisionObject;
		btVector3          hitPointWorld;
		btVector3          hitNormalWorld;
		btScalar           hitFraction;
	};

	std::vector<Hit> hits;

	AllHitsConvexResultCallback()
	{
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult,
	                                 bool                                 normalInWorldSpace) override
	{
		Hit hit;

		hit.collisionObject = const_cast<btCollisionObject *>(convexResult.m_hitCollisionObject);

		hit.hitFraction = convexResult.m_hitFraction;

		// Convert normal to world space if necessary
		if (normalInWorldSpace)
		{
			hit.hitNormalWorld = convexResult.m_hitNormalLocal;
		}
		else
		{
			hit.hitNormalWorld = hit.collisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
		}

		// Compute world-space hit point
		hit.hitPointWorld = convexResult.m_hitPointLocal;

		hits.push_back(hit);

		// Returning the current closest fraction allows Bullet
		// to continue collecting additional hits.
		return m_closestHitFraction;
	}

	bool hasHits() const
	{
		return !hits.empty();
	}
};
