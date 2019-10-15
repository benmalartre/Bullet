
#ifndef _BULLET_C_API_
#define _BULLET_C_API_

//#include <stdbool.h>

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

const int maxProxies = 32766;
const int maxOverlap = 65535;

const float TRIANGLE_SIZE=8.f;
static float waveheight = 5.f;

#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifdef BT_USE_DOUBLE_PRECISION
typedef double	BTReal;
#else
typedef float	BTReal;
#endif

typedef BTReal BTVector3[3];
typedef BTReal BTQuaternion[4];
typedef BTReal BTMatrix4[16];

typedef struct BTXForm BTXForm;
struct BTXForm{
	BTVector3 pos;
	BTQuaternion rot;
	BTVector3 scl;
};

typedef struct BTConvexHullDescription BTConvexHullDescription;
struct BTConvexHullDescription{
	int num_points;
	int num_planes;
	int num_edges;
	BTReal* vertices;
	int* indices;
	
};

#ifdef __cplusplus
extern "C" { 
#endif

/**	Particular physics SDK (C-API) */
	BT_DECLARE_HANDLE(btPhysicsSdkHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
	BT_DECLARE_HANDLE(btDynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/	
	BT_DECLARE_HANDLE(btRigidBodyHandle);

/** Soft Body that can be part of a Dynamics World (C-API)*/	
	BT_DECLARE_HANDLE(btSoftBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	BT_DECLARE_HANDLE(btCollisionShapeHandle);

/** Constraint for Rigid Bodies (C-API)*/
	BT_DECLARE_HANDLE(btConstraintHandle);
	BT_DECLARE_HANDLE(btConstraintSolverHandle);

/** Triangle Mesh interface (C-API)*/
	BT_DECLARE_HANDLE(btMeshInterfaceHandle);

/** Broadphase Scene/Proxy Handles (C-API)*/
	BT_DECLARE_HANDLE(btCollisionBroadphaseHandle);
	BT_DECLARE_HANDLE(btBroadphaseProxyHandle);
	BT_DECLARE_HANDLE(btCollisionWorldHandle);

	typedef void * C3DObjectPointer ;

	btDynamicsWorldHandle			BTTestWorld(bool useMCLPSolver);
	void							BTTestXForm(BTXForm* T);
	btDynamicsWorldHandle			BTSoftBodyWorld();
	int								BTCheckSoftBodyWorldInfo(btPhysicsSdkHandle sdkHandle);
/** Create and Delete a Physics SDK	*/
	btPhysicsSdkHandle				BTCreateDynamicsSdk(void); //this could be also another sdk, like ODE, PhysX etc.
	void							BTDeleteDynamicsSdk(btPhysicsSdkHandle	physicsSdk);
	btDynamicsWorldHandle			BTGetDynamicsWorld(btPhysicsSdkHandle physicsSdk);

/** Collision World, not strictly necessary, you can also just create a Dynamics World with Rigid Bodies which internally manages the Collision World with Collision Objects */

	typedef void(*btBroadphaseCallback)(void* clientData, void* object1,void* object2);
	btCollisionBroadphaseHandle		BTCreateSapBroadphase(btBroadphaseCallback beginCallback,btBroadphaseCallback endCallback);
	void							BTDestroyBroadphase(btCollisionBroadphaseHandle bp);
	btBroadphaseProxyHandle			BTCreateProxy(btCollisionBroadphaseHandle bp, void* clientData, BTReal minX,BTReal minY,BTReal minZ, BTReal maxX,BTReal maxY, BTReal maxZ);
	void							BTDestroyProxy(btCollisionBroadphaseHandle bp, btBroadphaseProxyHandle proxyHandle);
	void							BTSetBoundingBox(btBroadphaseProxyHandle proxyHandle, BTReal minX,BTReal minY,BTReal minZ, BTReal maxX,BTReal maxY, BTReal maxZ);

/* todo: add pair cache support with queries like add/remove/find pair */
	

/* todo: add/remove objects */
	

/* Dynamics World */
	btDynamicsWorldHandle			BTCreateDynamicsWorld(btPhysicsSdkHandle physicsSdkHandle,bool useMCLPSolver);
	btDynamicsWorldHandle			BTCreateSoftRigidDynamicsWorld(btPhysicsSdkHandle physicsSdk);
	void							BTDeleteDynamicsWorld(btDynamicsWorldHandle world);
	void							BTDeleteSoftRigidDynamicsWorld(btDynamicsWorldHandle world);
	int								BTStepSimulation(btDynamicsWorldHandle,	BTReal	timeStep);
	void							BTAddRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object);
	void							BTRemoveRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object);
	void							BTAddSoftBody(btDynamicsWorldHandle world, btSoftBodyHandle object);
	void							BTRemoveSoftBody(btDynamicsWorldHandle world, btSoftBodyHandle object);
	int								BTGetNumCollideObjects(btDynamicsWorldHandle world);
	void							BTSetGravity(btDynamicsWorldHandle world, BTVector3 gravity);
	//void							BTSetCollisionProcessedCallback( ContactProcessedCallback fn );
	void							BTGetWorldBoundingBox(btPhysicsSdkHandle sdk,BTVector3 bb_min,BTVector3 bb_max);
	
	//btSoftBodyWorldInfoHandle		BTGetSoftBodyWorldInfo(btPhysicsSdkHandle physicsSdk);

	int								BTCheckSoftBodySolver(btPhysicsSdkHandle sdkHandle);
	void							BTResetSparseSDF(btPhysicsSdkHandle sdkHandle);
/* Rigid Body  */
	btRigidBodyHandle				BTCreateRigidBody(	void* user_data,  float mass, btCollisionShapeHandle cshape );
	void							BTDeleteRigidBody(btRigidBodyHandle body);
	btRigidBodyHandle				BTGetRigidBodyByID(btDynamicsWorldHandle world, int index);
	C3DObjectPointer				BTGetUserData(btRigidBodyHandle body);
	void							BTSetActivationState(btRigidBodyHandle cbody, int state);
	void							BTRigidBodySetCollisionFlags(btRigidBodyHandle cbody, int flags);
	int								BTRigidBodyGetCollisionFlags(btRigidBodyHandle cbody);
	void							BTSetLinearFactor(btRigidBodyHandle cbody,BTVector3 factor);
	void							BTSetAngularFactorF(btRigidBodyHandle cbody,float factor);
	void							BTSetAngularFactor(btRigidBodyHandle cbody,BTVector3 factor);
	void							BTSetLinearVelocity(btRigidBodyHandle cbody,BTVector3 velocity);
	void							BTSetAngularVelocity(btRigidBodyHandle cbody,BTVector3 velocity);

/* Soft Body */
	btSoftBodyHandle				BTCreateSoftBodyFromConvexHull(void* user_data,btPhysicsSdkHandle hsdk, BTReal* vertices,int* indices, int nb_triangles);
	btSoftBodyHandle				BTCreateSoftBodyFromTriMesh(void* user_data,btPhysicsSdkHandle hsdk, BTReal* vertices,int* indices, int nb_triangles);
	btSoftBodyHandle				BTCreateClusterSoftBodyFromTriMesh(void* user_data,btPhysicsSdkHandle hsdk, BTReal* vertices,int* indices, int nb_triangles,int nb_clusters);
	btSoftBodyHandle				BTGetSoftBodyByID(btDynamicsWorldHandle world, int id);
	int 							BTUpdatePointPosition(btSoftBodyHandle sbdh,BTReal* vertices);
	btSoftBodyHandle				BTSoftBox(btPhysicsSdkHandle sdkHandle,BTVector3 ip,BTVector3 is);
	int								BTGetNumSoftBodies(btDynamicsWorldHandle hWorld);
	btSoftBodyHandle				BTSoftBoulder(btPhysicsSdkHandle sdkHandle,BTVector3 p,BTVector3 s,int np,int id);
	void							BTUpdateSoftBodyGeometry(btSoftBodyHandle hBody,BTReal* vertices);
	int								BTGetSoftBodyNbVertices(btSoftBodyHandle hBody);
	int								BTGetSoftBodyNbFaces(btSoftBodyHandle hBody);
	int								BTGetSoftBodyNbNodes(btSoftBodyHandle hBody);
	//btSoftBodyHandle 			BTCreateSoftBody(	void* user_data, float mass, btCollisionShapeHandle cshape);
	//void						BTDeleteSoftBody

/* Collision Shape definition */
	btCollisionShapeHandle			BTNewGroundPlaneShape(BTVector3 pos, BTReal size);
	btCollisionShapeHandle			BTNewSphereShape(BTReal radius);
	btCollisionShapeHandle			BTNewBoxShape(BTReal x, BTReal y, BTReal z);
	btCollisionShapeHandle			BTNewCapsuleShape(BTReal radius, BTReal height);
	btCollisionShapeHandle			BTNewConeShape(BTReal radius, BTReal height);
	btCollisionShapeHandle			BTNewCylinderShape(BTReal radius, BTReal height);
	btCollisionShapeHandle			BTNewCompoundShape(void);
	void							BTAddChildShape(btCollisionShapeHandle compoundShape,btCollisionShapeHandle childShape, BTVector3 childPos,BTQuaternion childOrn);
	btCollisionShapeHandle			BTNewGImpactShape(int num_tri,int* indices,int num_vertices,BTReal* vertices); // not working properly
	btCollisionShapeHandle			BTNewBvhTriangleMeshShape(int num_tri,int* indices,int num_vertices,BTReal* vertices, bool aligned=true);
	void							BTDeleteShape(btCollisionShapeHandle shape);
	void							BTSetCollisionMargin(btRigidBodyHandle hBody,BTReal margin);
	BTReal							BTGetCollisionMargin(btRigidBodyHandle hBody);
	void							BTSetFriction(btRigidBodyHandle hBody,BTReal friction);

/* Convex Meshes */
	btCollisionShapeHandle			BTNewEmptyConvexHullShape(void);
	btCollisionShapeHandle			BTNewConvexHullShape(int num_tri,int* indices,int num_vertices,BTReal* vertices);
	void							BTAddVertex(btCollisionShapeHandle convexHull, BTReal x,BTReal y,BTReal z);
	void							BTGetConvexHullShapeDescription(btCollisionShapeHandle handle,BTConvexHullDescription desc);
	void							BTFillConvexHullShapeDescription(btCollisionShapeHandle handle,BTConvexHullDescription desc);
	int								BTDrawConvexHullShape(btCollisionShapeHandle handle, BTMatrix4 model, BTMatrix4 view, BTMatrix4 projection);
	void							BTDrawCollisionShape(btCollisionShapeHandle handle, BTMatrix4 modelview, BTMatrix4 projection);

/* Concave static triangle meshes */
	 btMeshInterfaceHandle			BTNewMeshInterface(void);
	 void							BTAddTriangle(btMeshInterfaceHandle meshHandle, BTVector3 v0,BTVector3 v1,BTVector3 v2);
	 btCollisionShapeHandle			BTNewTriangleMeshShape(int nbt, int nbv, float* vertices, int* indices);

	 void							BTSetScaling(btRigidBodyHandle handle, BTVector3 scale);

/* Constraints */
	//btConstraintHandle				BTNewHingeConstraint(btRigidBodyHandle body,BTVector3 pivot,BTVector3 axis,bool usereferenceframe);
	btConstraintHandle				BTNewHingeConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3 pivotA,BTVector3 pivotB,BTVector3 axisA,BTVector3 axisB,bool useReferenceFrameA);
	btConstraintHandle				BTNewHingeConstraint2(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB);
    void							BTSetHingeConstraintLimits(btConstraintHandle constraint,BTReal low, BTReal high, BTReal softness,BTReal biasFactor, BTReal relaxationFactor);
	btConstraintHandle				BTNewGearConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3 axisA,BTVector3 axisB,float ratio);
	btConstraintHandle				BTNewPoint2PointConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3 pivotA,BTVector3 pivotB);
	btConstraintHandle				BTNewSliderConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm* frameA,BTXForm* frameB,bool useReferenceFrameA);
	btConstraintHandle				BTNewConeTwistConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm* frameA,BTXForm* frameB);
	btConstraintHandle				BTNewGeneric6DofConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm* frameA,BTXForm* frameB,bool usereferenceframeA);
	void							BTSetGeneric6DofConstraintLimit(btConstraintHandle constraint,int axis, BTReal lo, BTReal hi);
	void							BTSetGeneric6DofConstraintLinearLowerLimit(btConstraintHandle constraint,BTVector3 limit);
	void							BTSetGeneric6DofConstraintLinearUpperLimit(btConstraintHandle constraint,BTVector3 limit);
	void							BTSetGeneric6DofConstraintAngularLowerLimit(btConstraintHandle constraintHandle,BTVector3 limit);
	void							BTSetGeneric6DofConstraintAngularUpperLimit(btConstraintHandle constraintHandle,BTVector3 limit);
	void							BTAddConstraint(btDynamicsWorldHandle world, btConstraintHandle constraint,bool disableCollisionBetweenLinkedBodies);	 
/* SOLID has Response Callback/Table/Management */
/* PhysX has Triggers, User Callbacks and filtering */
/* ODE has the typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2); */

/*	typedef void plUpdatedPositionCallback(void* userData, plRigidBodyHandle	rbHandle, plVector3 pos); */
/*	typedef void plUpdatedOrientationCallback(void* userData, plRigidBodyHandle	rbHandle, plQuaternion orientation); */

/* get world transform */
	void							BTGetMatrix(btRigidBodyHandle object, BTMatrix4 matrix);
	void							BTGetPosition(btRigidBodyHandle object,BTVector3 position);
	void							BTGetOrientation(btRigidBodyHandle object,BTQuaternion orientation);

/* set world transform (position/orientation) */
	void							BTSetPosition(btRigidBodyHandle object, BTVector3 position);
	void							BTSetOrientation(btRigidBodyHandle object, BTQuaternion orientation);
	void							BTSetEuler(BTReal yaw,BTReal pitch,BTReal roll, BTQuaternion orient);
	void							BTSetMatrix(btRigidBodyHandle object, BTMatrix4 matrix);

	void							BTTestVector3(BTVector3 io,float x, float y, float z);
	void							BTTestQuaternion(BTQuaternion io,float x, float y, float z,float w);

	void							BTTranslate(btRigidBodyHandle handle,BTReal x, BTReal y, BTReal z);
	void							BTTransform(btRigidBodyHandle object, BTQuaternion q, BTVector3 p);

	typedef struct btRayCastResult {
		btRigidBodyHandle		m_body;  
		btCollisionShapeHandle	m_shape; 		
		BTVector3				m_pointWorld; 		
		BTVector3				m_normalWorld;
		long					m_triangleIndex;

	} btRayCastResult;

    BT_DECLARE_HANDLE(btRayCastResultHandle);
    
	int			BTRayCast(btDynamicsWorldHandle world, const BTVector3 rayStart, const BTVector3 rayEnd, btRayCastResultHandle res);
	int			BTTest(btPhysicsSdkHandle hsdk);
	int		    BTRayCastHit(btDynamicsWorldHandle world);

	/* Sweep API */

	/*  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal); */

	/* Continuous Collision Detection API */
	
	// needed for source/blender/blenkernel/intern/collision.c
	double BTNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3]);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

