#include <bullet/btBulletCollisionCommon.h>
#include <g_levellocals.h>
#include <p_collision.h>

void FLevelLocals::InitCollisionWorld()
{
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
	
	ClearAllCollisionObjects();
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

	/*
	collisionWorld->performDiscreteCollisionDetection();

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();

	bool colliding = false;

	for (int i = 0; i < numManifolds; ++i)
	{
		btPersistentManifold *manifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);

		const btCollisionObject *objA = static_cast<const btCollisionObject *>(manifold->getBody0());
		const btCollisionObject *objB = static_cast<const btCollisionObject *>(manifold->getBody1());

		if ((objA == a && objB == b) || (objA == b && objB == a))
		{
			if (manifold->getNumContacts() > 0)
			{
				colliding = true;
				break;
			}
		}
	}
	RemoveCollisionObject(a);
	RemoveCollisionObject(b);
	if (colliding)
		Printf(" %s touch %s", tmthing->GetTag(), thing->GetTag());
	return colliding;
	*/
}
