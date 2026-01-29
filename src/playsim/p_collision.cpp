#include <bullet/btBulletCollisionCommon.h>
#include <g_levellocals.h>
#include <p_collision.h>
#include "filesystem.h"

void FLevelLocals::InitCollisionWorld()
{
	if (collisionWorld)
		delete collisionWorld;
	auto config     = new btDefaultCollisionConfiguration();
	auto dispatcher = new btCollisionDispatcher(config);
	auto broadphase = new btDbvtBroadphase();
	collisionWorld  = new btCollisionWorld(dispatcher, broadphase, config);
	Printf("Created CollisionWorld for %s\n", this->MapName);
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
	AddCollisionObject(thing->collisionModel);
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
	col->collider->setUserPointer(col);
	col->setWorldPos(thing->Pos());
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

TArray<AActor *> FLevelLocals::CollidingActors(AActor *thing)
{
	TArray<AActor *> colliders;
	int manifoldCount = collisionWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < manifoldCount; ++i)
	{
		btPersistentManifold *manifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);

		const btCollisionObject *objA = static_cast<const btCollisionObject *>(manifold->getBody0());
		const btCollisionObject *objB = static_cast<const btCollisionObject *>(manifold->getBody1());

		int contactCount = manifold->getNumContacts();

		bool isColliding = false;

		for (int j = 0; j < contactCount; ++j)
		{
			const btManifoldPoint &pt = manifold->getContactPoint(j);

			if (pt.getDistance() < 0.0f)
			{
				isColliding = true;
				break;
			}
		}

		if (isColliding)
		{
			auto *advcola = static_cast<AdvancedCollisionModel *>(objA->getUserPointer());
			auto *advcolb = static_cast<AdvancedCollisionModel *>(objB->getUserPointer());
			auto  actorA  = advcola->owner;
			auto  actorB  = advcolb->owner;
			if (actorA == thing)
			{
				colliders.AddUnique(actorB);
			}
			else if (actorB == thing)
			{
				colliders.AddUnique(actorA);
			}
		}
	}
	return colliders;
}

void FLevelLocals::PerformCollisions()
{
	if (collisionWorld)
	{
		for (auto c : collisionModels)
		{
			UpdateCollisionModel(c->owner);
		}

		collisionWorld->performDiscreteCollisionDetection();
		for (int pi = 0; pi < 64;pi++)
		{
			if (!PlayerInGame(pi))
				continue;
			auto a = players[pi].mo;
			if (a)
			{
				auto colliders = CollidingActors(a);
				for (auto col : colliders)
				{
					Printf("Colliding with: %s\n",col->GetTag());
				}
			}
		}
	}
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
	int numContacts = cb.m_numContacts;

	for (auto point : cb.manifoldPoints)
	{
		auto btv = point.getPositionWorldOnA();
		ret.Push(DVector3(btv.x(), btv.y(), btv.z()));
	}
	
	RemoveCollisionObject(thing);
	RemoveCollisionObject(tmthing);
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
	return numContacts > 0;
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

TArray<DVector3> MeshCollisionModel::getPoints()
{
	TArray<DVector3> ret;
	btBvhTriangleMeshShape *shape = static_cast<btBvhTriangleMeshShape *>(collider->getCollisionShape());

	WorldVertexCollector callback(collider->getWorldTransform());

	btVector3 aabbMin(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	btVector3 aabbMax(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);

	shape->processAllTriangles(&callback, aabbMin, aabbMax);

	return callback.WorldPoints;
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
			

			trimesh->addTriangle(verts[ind[0]-1], verts[ind[1]-1], verts[ind[2]-1]);
		}
	}
	Printf("%s: Pushed %d verts and %d tris\n", filename.GetChars(), verts.size(), trimesh->getNumTriangles());

	
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

	Printf("%s: Pushed %d verts\n", filename.GetChars(), hullshape->getNumPoints());
	return hullshape;
}
