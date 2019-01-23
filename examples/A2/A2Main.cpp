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

#include "A2Main.h"

#include "btBulletDynamicsCommon.h"


#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#define SIM_HERTZ 300. // 300 hertz for higher accuracy

static btScalar gRopeResolution = 0;  // default rope resolution (number of links as in a chain)


struct A2Example : public CommonRigidBodyBase
{
	A2Example(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper)
	{
	}
	virtual ~A2Example() {}
    virtual void stepSimulation(float deltaTime);
    virtual void initPhysics();
	virtual void renderScene();
	void createEmptyDynamicsWorld()
	{ // copied from NewtonsRopeCradle.cpp
		m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		m_solver = new btSequentialImpulseConstraintSolver;

		m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

		softBodyWorldInfo.m_broadphase = m_broadphase;
		softBodyWorldInfo.m_dispatcher = m_dispatcher;
		softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
		softBodyWorldInfo.m_sparsesdf.Initialize();

        m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

        if (m_dynamicsWorld->getDebugDrawer())
            m_dynamicsWorld->getDebugDrawer()->setDebugMode(
                    btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints /** + btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits */);

    }
	virtual void connectWithMassSpringConstraints(btRigidBody *body1, btRigidBody *body2);
	virtual void connectWithRope(btRigidBody* body1, btRigidBody* body2);
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
	/**
	 * Returns m_dynamicsWorld casted to btSoftRigidDynamicsWorld.
	 * This is used before calling a method like ->addSoftBody() to the world
	 */
	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
	{
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	btSoftBodyWorldInfo softBodyWorldInfo;

	void createNet();

	btRigidBody *createNode(int x, int y, int z, btSphereShape* shape);
};

/**
 * Overrides stepSimulation implemented in CommonRigidBodyBase in order to increase time step
 * @param deltaTime
 */
void A2Example::stepSimulation(float deltaTime)
{
    if (m_dynamicsWorld)
    {
        btScalar timeStep = deltaTime;
        int maxSubSteps=1;
        btScalar fixedTimeStep=btScalar(1.)/btScalar(SIM_HERTZ);
        // It is recommended/necessary that timeStep < maxSubSteps * fixedTimeStep

        m_dynamicsWorld->stepSimulation(timeStep,maxSubSteps,fixedTimeStep);
    }
}

void A2Example::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

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

		body->applyCentralForce(btVector3(100,0,0));
	}

	createNet();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void A2Example::renderScene()
{
	CommonRigidBodyBase::renderScene();
	btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

	for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
		//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
			softWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);
			btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
		}
	}
}

void A2Example::createNet() {
	btSphereShape* nodeShape = new btSphereShape(.1);
	m_collisionShapes.push_back(nodeShape);

	btAlignedObjectArray<btRigidBody*> nodes;


	int i, j;
	int length = 10;
	int width = 10;

	for(i = 0; i < length; i++) {
		for(j = 0; j < width; j++){
			btRigidBody* node = createNode(i,3,j, nodeShape);
			nodes.push_back(node);
		}
	}

	printf("%i", nodes.size());

	for(i = 0; i < length; i++) {
		for(j = 0; j < width; j++){
			if (j != width - 1) {
				// connectWithMassSpringConstraints([i, j], [i, j+1]);
				//connectWithMassSpringConstraints(nodes[i*width + j], nodes[i*width+j + 1]);
				connectWithRope(nodes[i*width + j], nodes[i*width+j + 1]);
				printf("connecting width (%i,%i)\n", i, j);
				printf("points are (%i), and (%i)\n", i*width + j, i*width+j + 1);
			}
			if (i != length - 1) {
				// connectWithMassSpringConstraints([i, j],[i + 1, j]);
				//connectWithMassSpringConstraints(nodes[i*width + j], nodes[i*width+j + width]);
				connectWithRope(nodes[i*width + j], nodes[i*width+j + width]);
				printf("connecting length (%i,%i)\n", i, j);
				printf("points are (%i), and (%i)\n", i*width + j, i*width+j + width);
			}
		}
	}

	// Identify corners
	btAlignedObjectArray<btRigidBody*> corners;
	corners.push_back(nodes[0]);
	corners.push_back(nodes[length - 1]);
	corners.push_back(nodes[length*width - length]);
	corners.push_back(nodes[length*width - 1]);

	// Make a larger node for corners
    btScalar mass(0.1f);
    btSphereShape* largeNodeShape = new btSphereShape(.25);
    btVector3 localInertia(0,0,0);
    largeNodeShape->calculateLocalInertia(mass, localInertia);

    for (i = 0; i < 4; i++) {
        // apply changes to corners
        corners[i]->setMassProps(1.f,localInertia);
	    corners[i]->applyCentralForce(btVector3(0,-10000,0));
	    corners[i]->setCollisionShape(largeNodeShape);
	}
}

void A2Example::connectWithMassSpringConstraints(btRigidBody *body1, btRigidBody *body2) {
	btTransform localA;
	btTransform localB;
	localA.setIdentity();
	localA.getOrigin() = btVector3(.25, 0, 0);
	localB.setIdentity();
	localB.setOrigin(btVector3(-.25, 0, 0));
	btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*body1, *body2, localA, localB, true);
	m_dynamicsWorld->addConstraint(spring);

}


void A2Example::connectWithRope(btRigidBody* body1, btRigidBody* body2)
{ // Copied from NewtonsRopeCradle.cpp
	btSoftBody* softBodyRope0 = btSoftBodyHelpers::CreateRope(softBodyWorldInfo, body1->getWorldTransform().getOrigin(), body2->getWorldTransform().getOrigin(), gRopeResolution, 0);
	softBodyRope0->setTotalMass(0.01f);

	softBodyRope0->appendAnchor(0, body1);
	softBodyRope0->appendAnchor(softBodyRope0->m_nodes.size() - 1, body2);

	softBodyRope0->m_cfg.piterations = 5;
	softBodyRope0->m_cfg.kDP = 0.005f;
	softBodyRope0->m_cfg.kSHR = 1;
	softBodyRope0->m_cfg.kCHR = 1;
	softBodyRope0->m_cfg.kKHR = 1;

	getSoftDynamicsWorld()->addSoftBody(softBodyRope0);
}


btRigidBody* A2Example::createNode(int x, int y, int z, btSphereShape* shape) {
	btSphereShape* nodeShape = shape;

	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.1f);

	btVector3 localInertia(0,0,0);
	nodeShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(.5 * x ,y, .5 * z));

	btRigidBody* body = createRigidBody(mass, startTransform, nodeShape);

	body->setDamping(btScalar(0), btScalar(0));
	body->setSleepingThresholds(btScalar(0), btScalar(0));

	return body;
}



CommonExampleInterface* A2ExampleCreateFunc(CommonExampleOptions& options)
{
	return new A2Example(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(A2ExampleCreateFunc)
