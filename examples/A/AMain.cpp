//
// Created by jdesmond on 1/12/19.
//

#include <stdio.h>
#include "btBulletDynamicsCommon.h"

int main(int argc, char **argv) {
    printf("Hello");

    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface *overlappingPairCache = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver,
                                                                         collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, 0, 0));

    btCollisionShape *boxShape = new btBoxShape(btVector3(1, 1, 1));
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);
    btVector3 localInertia(3, 4, 5);
    boxShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(0, 10, 0));

    startTransform.setRotation(btQuaternion(1,1,1,1));

    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boxShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    dynamicsWorld->addRigidBody(body);


    int i;
    ///-----stepsimulation_start-----
    for (i = 0; i < 150; i++) {
        dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        //print positions of all objects
        for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--) {
            btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[j];
            btRigidBody *body = btRigidBody::upcast(obj);
            btTransform trans;
            if (body && body->getMotionState()) {
                body->getMotionState()->getWorldTransform(trans);
            } else {
                trans = obj->getWorldTransform();
            }
            printf("world rot object %d = %f, %f, %f, %f\n", j, float(trans.getRotation().getW()),
                   float(trans.getRotation().getX()), float(trans.getRotation().getY()),
                   float(trans.getRotation().getZ()));
            printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()),
                   float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
        }

        printf("%f", body->getOrientation().getAngle());
    }


    //remove the rigidbodies from the dynamics world and delete them
    for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody *body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    // delete shape
    delete boxShape;

    //delete dynamics world
    delete dynamicsWorld;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;
}
