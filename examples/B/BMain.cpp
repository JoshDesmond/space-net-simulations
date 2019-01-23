/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BMain.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#define SIM_HERTZ 300. // 300 hertz for higher accuracy

struct BExample : public CommonRigidBodyBase
{
	BExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BExample() {}
    virtual void stepSimulation(float deltaTime);
    virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

/**
 * Overrides stepSimulation implemented in CommonRigidBodyBase in order to increase time step
 * @param deltaTime
 */
void BExample::stepSimulation(float deltaTime)
{
    if (m_dynamicsWorld)
    {

        btScalar timeStep = deltaTime;
        int maxSubSteps=1;
        btScalar fixedTimeStep=btScalar(1.)/btScalar(SIM_HERTZ);
        // It is necessary that timeStep < maxSubSteps * fixedTimeStep

        m_dynamicsWorld->stepSimulation(timeStep,maxSubSteps,fixedTimeStep);
    }
}

void BExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create target rigid body
	{
		btBoxShape* colShape = createBoxShape(btVector3(1., 1., 1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Object
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(10.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(0),
			btScalar(0),
			btScalar(0)));

		btRigidBody* body = createRigidBody(mass, startTransform, colShape);

		// Add rotation and set properties
		body->setAngularVelocity(btVector3(.1,.3,.2));
		body->setDamping(btScalar(0),btScalar(0)); // linear damping, angular damping
		// body->setContactStiffnessAndDamping(btScalar(0), btScalar(0));
		body->setSleepingThresholds(btScalar(0), btScalar(0));
	}

	/// Create second test rigid body
	{
		btSphereShape* colShape = new btSphereShape(.5);
		m_collisionShapes.push_back(colShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		btVector3 localInertia(0,0,0);
		colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(-2,0,0));

		btRigidBody* body = createRigidBody(mass, startTransform, colShape);

		body->setDamping(btScalar(0), btScalar(0));
		body->setSleepingThresholds(btScalar(0), btScalar(0));

		body->applyCentralForce(btVector3(1000,0,0));
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BExampleCreateFunc(CommonExampleOptions& options)
{
	return new BExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BExampleCreateFunc)
