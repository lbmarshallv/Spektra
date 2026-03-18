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

	btCapsuleShape *initShape()
	{
		auto bs = new btCapsuleShape(radius, height-(2.f*radius));
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

class ConvexHullCollisionModel : public AdvancedCollisionModel
{
  public:
	static btConvexHullShape *fromOBJModel(FName filename, ObjModelInfo &info);

	ConvexHullCollisionModel(FName filename, ObjModelInfo &info)
	{
		btConvexHullShape *bs = fromOBJModel(filename, info);
		bs->setMargin(0.05f);
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
	btTriangleMesh *trimesh;

	static btTriangleMesh *fromOBJModel(FName filename, ObjModelInfo &info);

	 MeshCollisionModel(FName filename, ObjModelInfo &info)
	{
		trimesh                    = fromOBJModel(filename, info);
		btBvhTriangleMeshShape *bs = new btBvhTriangleMeshShape(trimesh, true);
		bs->setMargin(0.1f);
		collider = new btCollisionObject();
		collider->setCollisionShape(bs);
		collider->setUserPointer(this);

		btTransform transform;
		transform.setIdentity();
		collider->setWorldTransform(transform);
	}
};



	struct lwContactPoint
{
	DVector3 m_ptOnAWorld;
	DVector3 m_ptOnBWorld;
	DVector3 m_normalOnB;
	double    m_distance;
};

struct AdvancedCollisionContactResultCallback : public btCollisionWorld::ContactResultCallback
{
	int             m_numContacts;
	lwContactPoint *m_pointsOut;
	int             m_pointCapacity;
	TArray<btManifoldPoint> manifoldPoints;

	AdvancedCollisionContactResultCallback(lwContactPoint *pointsOut, int pointCapacity)
		: m_numContacts(0), m_pointsOut(pointsOut), m_pointCapacity(pointCapacity)
	{
	}
	virtual btScalar addSingleResult(btManifoldPoint &cp, const btCollisionObjectWrapper *colObj0Wrap, int partId0,
	                                 int index0, const btCollisionObjectWrapper *colObj1Wrap, int partId1, int index1)
	{
		if (m_numContacts < m_pointCapacity)
		{
			manifoldPoints.Push(cp);
			lwContactPoint &ptOut = m_pointsOut[m_numContacts];
			ptOut.m_distance      = cp.m_distance1;
			ptOut.m_normalOnB[0]  = cp.m_normalWorldOnB.getX();
			ptOut.m_normalOnB[1]  = cp.m_normalWorldOnB.getY();
			ptOut.m_normalOnB[2]  = cp.m_normalWorldOnB.getZ();
			ptOut.m_ptOnAWorld[0] = cp.m_positionWorldOnA[0];
			ptOut.m_ptOnAWorld[1] = cp.m_positionWorldOnA[1];
			ptOut.m_ptOnAWorld[2] = cp.m_positionWorldOnA[2];
			ptOut.m_ptOnBWorld[0] = cp.m_positionWorldOnB[0];
			ptOut.m_ptOnBWorld[1] = cp.m_positionWorldOnB[1];
			ptOut.m_ptOnBWorld[2] = cp.m_positionWorldOnB[2];
			m_numContacts++;
		}

		return 1.f;
	}
};
