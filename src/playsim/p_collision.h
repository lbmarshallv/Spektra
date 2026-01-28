#pragma once
#include <bullet/btBulletCollisionCommon.h>
#include <actor.h>

class AdvancedCollisionModel
{
  public:
	btCollisionObject *collider;
	AActor            *owner;

	virtual void setWorldPos(DVector3 pos)
	{
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(pos.X, pos.Y, pos.Z));
		collider->setWorldTransform(transform);
	}

	btCollisionShape *getCollisionShape()
	{
		return collider->getCollisionShape();
	}
};

class CylinderCollisionModel : public AdvancedCollisionModel
{
  public:
	int radius, height;
	CylinderCollisionModel(int radius, int height)
	{
		this->radius = radius;
		this->height = height;
		btCylinderShape *bs = new btCylinderShape(btVector3(radius, height/2, radius ));
		bs->setMargin(0);
		collider     = new btCollisionObject();
		collider->setCollisionShape(bs);
		collider->setUserPointer(this);
	}

	void setWorldPos(DVector3 pos) override
	{
		btTransform transform;
		transform.setIdentity();
		// cylinder's height is along the Y axis where Doom's is on the Z
		transform.setRotation(btQuaternion(0, SIMD_HALF_PI, 0));
		transform.setOrigin(btVector3(pos.X, pos.Y, pos.Z + height / 2));
		collider->setWorldTransform(transform);
	}
};

//regular doom collisi
// on model is a rectangular prism that can't rotate
class DoomStandardCollisionModel : public AdvancedCollisionModel
{
  public:
	int radius, height;
	DoomStandardCollisionModel(int radius, int height)
	{
		this->radius = radius;
		this->height = height;
		auto bs = new btBoxShape(btVector3(radius, radius, height/2));
		bs->setMargin(0);
		collider     = new btCollisionObject();
		collider->setCollisionShape(bs);
		collider->setUserPointer(this);
	}

	
	void setWorldPos(DVector3 pos) override
	{
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(pos.X, pos.Y, pos.Z+height/2));
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
