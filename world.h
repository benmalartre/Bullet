#ifndef _BULLET_WORLD_H_
#define _BULLET_WORLD_H_

//#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btConvexHull.h"
#include "LinearMath/btConvexHullComputer.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"


#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"//picking

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btSerializer.h"
#include "LinearMath/btIDebugDraw.h"


#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

struct    btPhysicsSdk
{
    btVector3    m_worldAabbMin;
    btVector3     m_worldAabbMax;
    btSoftBodyWorldInfo m_softBodyWorldInfo;
    
    btBroadphaseInterface * m_broadphase;
    btCollisionDispatcher * m_dispatcher;
    btConstraintSolver * m_solver;
    btSoftBodySolver * m_softBodySolver;
    btCollisionAlgorithmCreateFunc*    m_boxBoxCF;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    btDynamicsWorld * m_world;
    btDynamicsWorld * m_pick;
    
    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>        m_collisionShapes;
    
    //todo: version, hardware/optimization settings etc?
    btPhysicsSdk()
    :m_worldAabbMin(-1000,-1000,-1000),
    m_worldAabbMax(1000,1000,1000),
    m_broadphase(NULL),
    m_dispatcher(NULL),
    m_solver(NULL),
    m_collisionConfiguration(NULL),
    m_boxBoxCF(NULL),
    m_world(NULL),
    m_pick(NULL)
    {
        
    }
    
};

#endif
