#include <bullet/btBulletCollisionCommon.h>
#include <g_levellocals.h>
#include <p_collision.h>
#include "filesystem.h"

void AdvancedCollisionModel::setWorldPos(DVector3 pos)
{
	btTransform transform = collider->getWorldTransform();
	transform.setOrigin(toBulletPoint(pos));
	collider->setWorldTransform(transform);
}

void AdvancedCollisionModel::setRotation(DRotator d)
{
	btTransform transform = collider->getWorldTransform();
	transform.setRotation(btQuaternion(d.Pitch.Radians(), d.Roll.Radians(), d.Yaw.Radians()));
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
		collider->setCollisionShape(initShape());
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
	//AddAllCollisionObjects();
}

void FLevelLocals::AddCollisionObject(btCollisionObject *obj)
{
	if (this->collisionWorld)
	{
		this->collisionWorld->addCollisionObject(obj);
	}
	else
	{
		Printf(TEXTCOLOR_RED "AddCollisionObject was called without a CollisionWorld\n");
	}
}

void FLevelLocals::AddCollisionObject(AActor *thing)
{
	this->AddCollisionObject(thing->collisionModel);
}

void FLevelLocals::AddCollisionObject(AdvancedCollisionModel *model)
{
	if (collisionModels.Find(model) == collisionModels.size())
	{
		AddCollisionObject(model->collider);
		collisionModels.AddUnique(model);
	}
}

void FLevelLocals::RemoveCollisionObject(btCollisionObject *obj)
{
	if (this->collisionWorld)
	{
		this->collisionWorld->removeCollisionObject(obj);
	}
	else
	{
		Printf(TEXTCOLOR_RED "RemoveCollisionObject was called without a CollisionWorld\n");
	}
}

void FLevelLocals::RemoveCollisionObject(AActor *thing)
{
	for (int i = 0; i < collisionModels.size(); i++)
	{
		auto c = collisionModels[i];
		if (c && c->owner == thing)
		{
			collisionModels.Delete(i);
			RemoveCollisionObject(c->collider);
		}
	}
}

void FLevelLocals::ClearAllCollisionObjects()
{
	for (auto c : collisionModels)
	{
		if (c->collider)
			RemoveCollisionObject(c->collider);
	}
	collisionModels.clear();
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

void FLevelLocals::AddAllCollisionObjects()
{
	for (int i = 0; i < sectors.size(); i++)
	{
		sector_t s = sectors[i];
		AActor   *thing = sectors[i].thinglist;
		while (thing)
		{
			if (thing->flags & (MF_SOLID | MF_MISSILE))
			{
				AddCollisionObject(thing);
			}
			thing = thing->snext;
		}
	}
	Printf("Added %d collisionObjects to CollisionWorld\n", collisionModels.size());
}

bool FLevelLocals::TraceCollisionHit(DVector3 &from, DVector3 &to, AActor *thing)
{
	if (from == to)
		return false;
	btVector3                                  bFrom(from.X, from.Y, from.Z); 
	btVector3                                  bTo(to.X, to.Y, to.Z);
	UpdateCollisionModel(thing);
	AddCollisionObject(thing);

	btCollisionWorld::ClosestRayResultCallback closestResults(bFrom, bTo);
	collisionWorld->rayTest(bFrom, bTo, closestResults);
	RemoveCollisionObject(thing);



#ifdef _DEBUG
	if (closestResults.hasHit())
	{
		DVector3 hitPoint(closestResults.m_hitPointWorld.x(), closestResults.m_hitPointWorld.y(),
		                  closestResults.m_hitPointWorld.z());
		P_SpawnParticle(this, hitPoint, DVector3(0, 0, 0), DVector3(0, 0, 0), PalEntry(255, 0, 255),
		                255.0, 350, 4., 0, 0, SPF_FULLBRIGHT);
	}
#endif

	return closestResults.hasHit();
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
	{
		RemoveCollisionObject(thing);
	}

	return allResults;
}

TArray<DVector3> FLevelLocals::GetCollisionPoints(AActor *tmthing, AActor *thing)
{
	TArray<DVector3> ret;
	if (tmthing->Level != thing->Level)
	{
		Printf(TEXTCOLOR_RED "Two objects compared for CanCollide belong to different levels\n");
		return ret;
	}

	if (!collisionWorld)
	{
		Printf(TEXTCOLOR_RED "CanCollide was called without a CollisionWorld\n");
		return ret;
	}

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);
	lwContactPoint pointsOut[1000];

	btCollisionObject                     *a = tmthing->collisionModel->collider;
	btCollisionObject                     *b = thing->collisionModel->collider;
	AdvancedCollisionContactResultCallback cb(pointsOut, 1000);

	AddCollisionObject(thing);
	AddCollisionObject(tmthing);

	collisionWorld->contactPairTest(a, b, cb);

	for (auto point : cb.manifoldPoints)
	{
		auto btv = point.getPositionWorldOnA();
		ret.Push(tmthing->collisionModel->toWorldPoint(btv));
	}
	
	RemoveCollisionObject(thing);
	RemoveCollisionObject(tmthing);
	return ret;
}

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
	{
		return ret;
	}

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);

	btCollisionObject *a = tmthing->collisionModel->collider;
	btCollisionObject *b = thing->collisionModel->collider;
	btCollisionShape  *shape = a->getCollisionShape();


	DVector3 diff  = pos - tmthing->Pos();
	double steps = std::max(1.f,std::ceil(diff.length()));

	if (moving && shape->isConvex())
	{
		AddCollisionObject(thing);
		btTransform fromTrans = tmthing->collisionModel->collider->getWorldTransform();
		btTransform toTrans = tmthing->collisionModel->collider->getWorldTransform();
		toTrans.setOrigin(tmthing->collisionModel->toBulletPoint(pos));
		btVector3      fromPos = fromTrans.getOrigin();
		btVector3      toPos   = toTrans.getOrigin();
		btConvexShape *cs = static_cast<btConvexShape *>(tmthing->collisionModel->getCollisionShape());
		btCollisionWorld::ClosestConvexResultCallback closestConvex(fromPos, toPos);
		collisionWorld->convexSweepTest(cs, fromTrans, toTrans, closestConvex);

		RemoveCollisionObject(thing);
		if (closestConvex.hasHit())
		{
			if (asOrigin)
			{
				btScalar t = closestConvex.m_closestHitFraction;
				btVector3 hitOrigin = fromPos.lerp(toPos, t);
				ret.Push(DVector3(hitOrigin.x(), hitOrigin.y(), closestConvex.m_hitPointWorld.z()));
			}
			else
				ret.Push(DVector3(closestConvex.m_hitPointWorld.x(), closestConvex.m_hitPointWorld.y(),
			                  closestConvex.m_hitPointWorld.z()));
		}
	}
	else
	{
		for (double step = steps; step <= steps; step++)
		{
			DVector3 ptry(0, 0, 0);
			ptry.X = tmthing->Pos().X + ((step / steps) * diff.X);
			ptry.Y = tmthing->Pos().Y + ((step / steps) * diff.Y);
			ptry.Z = tmthing->Pos().Z + ((step / steps) * diff.Z);

			tmthing->collisionModel->setWorldPos(ptry);
			AddCollisionObject(tmthing);
			AddCollisionObject(thing);
			lwContactPoint pointsOut[1000];

			AdvancedCollisionContactResultCallback cb(pointsOut, 1000);

			collisionWorld->contactPairTest(a, b, cb);

			for (auto &point : cb.manifoldPoints)
			{
				auto    &btv        = point.getPositionWorldOnB();
				DVector3 worldPoint = tmthing->collisionModel->toWorldPoint(btv);
				ret.Push(worldPoint);
#ifdef _DEBUG
				if (tmthing->player && moving)
				{
					P_SpawnParticle(tmthing->Level, worldPoint, DVector3(0, 0, 0), DVector3(0, 0, 0),
					                PalEntry(255, 255, 255), 255.0, 350, 4., 0, 0, SPF_FULLBRIGHT);
				}
#endif
			}

			RemoveCollisionObject(tmthing);
			RemoveCollisionObject(thing);
		}
	}


	return ret;
}

bool FLevelLocals::CanCollide(AActor *tmthing, AActor *thing)
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
	UpdateCollisionModel(thing);
	lwContactPoint pointsOut[1000];

	btCollisionObject *a = tmthing->collisionModel->collider;
	btCollisionObject *b = thing->collisionModel->collider;
	AdvancedCollisionContactResultCallback cb(pointsOut, 1000);

	AddCollisionObject(thing);
	AddCollisionObject(tmthing);
	
	collisionWorld->contactPairTest(a, b, cb);
	int numContacts = cb.m_numContacts;

	RemoveCollisionObject(tmthing);
	RemoveCollisionObject(thing);
	return cb.manifoldPoints.size() > 0;
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
	lwContactPoint pointsOut[1000];

	btCollisionObject *a = tmthing->collisionModel->collider;
	btCollisionObject *b = thing->collisionModel->collider;
	AdvancedCollisionContactResultCallback cb(pointsOut, 1000);

	AddCollisionObject(thing);
	AddCollisionObject(tmthing);
	
	collisionWorld->contactPairTest(a, b, cb);
	int numContacts = cb.m_numContacts;

	RemoveCollisionObject(tmthing);
	RemoveCollisionObject(thing);
	
	return numContacts > 0;
}

struct MaxZConvexResultCallback : public btCollisionWorld::ConvexResultCallback
{
	btScalar maxZ;

	MaxZConvexResultCallback() : maxZ(-std::numeric_limits<btScalar>::infinity())
	{
	}

	btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult, bool normalInWorldSpace) override
	{
		// Hit point in world space
		btVector3 hitPointWorld;

		if (normalInWorldSpace)
		{
			// hitPointLocal is already in world space in this case
			hitPointWorld = convexResult.m_hitPointLocal;
		}
		else
		{
			// Transform from local space of the hit object to world space
			hitPointWorld = convexResult.m_hitCollisionObject->getWorldTransform() * convexResult.m_hitPointLocal;
		}

		maxZ = btMax(maxZ, hitPointWorld.getZ());

		m_closestHitFraction = btMin(m_closestHitFraction, convexResult.m_hitFraction);

		return convexResult.m_hitFraction;
	}
};

DVector3 FLevelLocals::SweepTest(AActor *tmthing, AActor *thing, DVector3 pos)
{
	double t = tmthing->Top();

	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);



	btCollisionShape  *shape = tmthing->collisionModel->getCollisionShape();
	btCollisionObject *b     = thing->collisionModel->collider;

	AddCollisionObject(thing);

	btTransform transFrom = tmthing->collisionModel->collider->getWorldTransform();
	btTransform transTo = tmthing->collisionModel->collider->getWorldTransform();
	transTo.setOrigin(tmthing->collisionModel->toBulletPoint(pos));
	btConvexShape *cs = static_cast<btConvexShape *>(tmthing->collisionModel->getCollisionShape());
	btCollisionWorld::ClosestConvexResultCallback closestConvex(transFrom.getOrigin(), transTo.getOrigin());
	collisionWorld->convexSweepTest(cs, transFrom, transTo, closestConvex);
	RemoveCollisionObject(thing);
	if (closestConvex.hasHit())
	{
		auto p = closestConvex.m_hitPointWorld;
		return DVector3(p.x(),p.y(),p.z());
	}
}

double FLevelLocals::GetTop(AActor *tmthing, AActor *thing)
{
	double t = tmthing->Top();
	
	UpdateCollisionModel(tmthing);
	UpdateCollisionModel(thing);

	btCollisionShape  *shape = tmthing->collisionModel->getCollisionShape();
	btCollisionObject *b = thing->collisionModel->collider;

	AddCollisionObject(thing);

	if (shape->isConvex())
	{
		btTransform transFrom = tmthing->collisionModel->collider->getWorldTransform();
		//transFrom.setOrigin(
		//	tmthing->collisionModel->toBulletPoint(DVector3(tmthing->Pos().X, tmthing->Pos().Y, tmthing->Pos().Z)));
		btTransform transTo   = tmthing->collisionModel->collider->getWorldTransform();
		transTo.setOrigin(tmthing->collisionModel->toBulletPoint(DVector3(tmthing->Pos().X + tmthing->Vel.X,
		                                                                  tmthing->Pos().Y + tmthing->Vel.Y,
		                                                                  thing->Pos().Z - tmthing->MaxStepHeight)));
		btConvexShape *cs = static_cast<btConvexShape *>(tmthing->collisionModel->getCollisionShape());
		btCollisionWorld::ClosestConvexResultCallback closestConvex(transFrom.getOrigin(), transTo.getOrigin());
		collisionWorld->convexSweepTest(cs, transFrom, transTo, closestConvex);


		RemoveCollisionObject(thing);
		if (closestConvex.hasHit())
		{
			return closestConvex.m_hitPointWorld.z();
		}
	}
	else
	{
		btVector3 from(tmthing->Pos().X + tmthing->Vel.X, tmthing->Pos().Y + tmthing->Vel.Y, t);
		btVector3 to(tmthing->Pos().X + tmthing->Vel.X, tmthing->Pos().Y + tmthing->Vel.Y, thing->Pos().Z);
		btCollisionWorld::ClosestRayResultCallback closestResults(from, to);
		collisionWorld->rayTest(from, to, closestResults);
		RemoveCollisionObject(thing);

		if (closestResults.hasHit())
		{
#ifdef _DEBUG
			DVector3 worldPoint(closestResults.m_hitPointWorld.x(), closestResults.m_hitPointWorld.y(),
			                    closestResults.m_hitPointWorld.z());
			if (tmthing->player)
			{
				P_SpawnParticle(tmthing->Level, worldPoint, DVector3(0, 0, 0), DVector3(0, 0, 0), PalEntry(255, 0, 255),
				                255.0, 350, 4., 0, 0, SPF_FULLBRIGHT);
			}
#endif
			return closestResults.m_hitPointWorld.z();
		}
	}

	return -INFINITY;
}

btTriangleMesh *MeshCollisionModel::fromOBJModel(FName filename)
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
		}
		else if (strlen(sc.String) == 1 && sc.String[0] == 'f')
		{
			int ind[3];
			for (int i = 0; i < 3; i++)
			{
				sc.MustGetString();
				FString s = FString(sc.String);
				TArray<FString> ss = s.Split("/");
				ind[i]             = ss.begin()->ToLong();
			}
			
			btVector3 v1 = verts[ind[0] - 1];
			btVector3 v2 = verts[ind[1] - 1];
			btVector3 v3 = verts[ind[2] - 1];
			
			trimesh->addTriangle(v1,v2,v3,true);
		}
	}
#ifdef _DEBUG
	Printf("%s: Pushed %d verts and %d tris to trimesh\n", filename.GetChars(), verts.size(), trimesh->getNumTriangles());
#endif
	return trimesh;
}

btConvexHullShape *ConvexHullCollisionModel::fromOBJModel(FName filename)
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
			hullshape->addPoint(btVector3(x, -z, y));
		}
	}
#ifdef _DEBUG
	Printf("%s: Pushed %d verts to convex hull\n", filename.GetChars(), hullshape->getNumPoints());
#endif
	return hullshape;
}

void AActor::initCollisions()
{
	if (collisionModelName != NULL && collisionModelName != "")
	{
		if (collisionModelShape == "hull")
		{
			collisionModel = new ConvexHullCollisionModel(collisionModelName);
		}
		else
		{
			collisionModel = new MeshCollisionModel(collisionModelName);
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
