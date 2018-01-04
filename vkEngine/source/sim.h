#pragma once

#include <unordered_map>

#include "math/Vec3.h"
#include "math/Mat34.h"

#include "misc/SortRadix.h"
#include "misc/PoolBasic.h"

#include "physics/phys_helpers.h"
#include "physics/joints_fem.h"

#include "collision/convex_hull.h"
#include "collision/convex_shapes.h"
#include "collision/mpr.h"
#include "collision/contact_helpers.h"

namespace physics
{

typedef void (*pfn_resetDrawingCallback)();
typedef void (*pfn_drawLineCallback)(const math::Vec3 & p0, const math::Vec3 & p1, const math::Vec4 & c0, const math::Vec4 & c1);
typedef void (*pfn_drawPointCallback)(const math::Vec3 & p, const math::Vec4 & c);

struct PhysicsSimCallbacks
{
	pfn_resetDrawingCallback resetDrawingFn = nullptr;
	pfn_drawLineCallback drawLineFn = nullptr;
	pfn_drawPointCallback drawPointFn = nullptr;
} g_simCallbacks;

void simResetDrawing()
{
	if (g_simCallbacks.resetDrawingFn)
		g_simCallbacks.resetDrawingFn();
}
void simDrawLine(const math::Vec3 & p0, const math::Vec3 & p1, const math::Vec4 & c0, const math::Vec4 & c1)
{
	if (g_simCallbacks.drawLineFn)
		g_simCallbacks.drawLineFn(p0, p1, c0, c1);
}
void simDrawPoint(const math::Vec3 & p, const math::Vec4 & c)
{
	if (g_simCallbacks.drawPointFn)
		g_simCallbacks.drawPointFn(p, c);
}

PoolBasic<Contact> g_contactJointPool;
PoolBasic<TriNodeContact> g_triNodeContactJointPool;

// Rigid body contact caching structures
PoolBasic<ContactJointCache> g_contactJointCachePool;
std::vector<ContactJointCache *> g_contactJointCacheStorage;

#define CONTACT_CACHE_ACCEL_HASHMAP	1
std::vector<uint32_t> g_contactJointCacheBody0LIdx;
std::vector<uint32_t> g_contactJointCacheSearchAccel;
#if (CONTACT_CACHE_ACCEL_HASHMAP == 1)
std::unordered_map<uint32_t, uint32_t> g_contactJointCacheSearchAccelIndex;
#endif

std::vector<JointBase *> g_joints;
std::vector<Node *> g_nodes;

struct ContactMassScaleParameters
{
	bool isEnabled;
	float maxMassScale;
};
ContactMassScaleParameters g_contactMassScaleParams;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arrays, used purely for tracking and safe release
std::vector<GeometryShape *> g_rigidBodyShapes;
std::vector<ConvexHullShape *> g_rigidBodyConvexHulls;

GeometryShape * trackGeometryShape(GeometryShape * newGeometryShape)
{
	g_rigidBodyShapes.push_back(newGeometryShape);
	return newGeometryShape;
}

void cleanupTrackedGeometryShapes()
{
	for (size_t irbs = 0, irbsEnd = g_rigidBodyShapes.size(); irbs < irbsEnd; ++irbs)
	{
		delete g_rigidBodyShapes[irbs];
	}
	g_rigidBodyShapes.resize(0);
}

ConvexHullShape * trackConvexHull(ConvexHullShape * newConvexHull)
{
	g_rigidBodyConvexHulls.push_back(newConvexHull);
	return newConvexHull;
}

void cleanupTrackedConvexHulls()
{
	for (size_t irbs = 0, irbsEnd = g_rigidBodyConvexHulls.size(); irbs < irbsEnd; ++irbs)
	{
		delete g_rigidBodyConvexHulls[irbs];
	}
	g_rigidBodyConvexHulls.resize(0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<RigidBody *> g_rigidBodies;

RigidBody * createRigidBody()
{
	RigidBody * newRigidBody = new RigidBody;
	g_rigidBodies.push_back(newRigidBody);
	return newRigidBody;
}

void cleanupRigidBodies()
{
	for (size_t irb = 0, irbEnd = g_rigidBodies.size(); irb < irbEnd; ++irb)
	{
		delete g_rigidBodies[irb];
	}
	g_rigidBodies.resize(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

JointBase * addTrackJoint(JointBase * joint)
{
	g_joints.push_back(joint);
	return joint;
}

void cleanupJoints()
{
	for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
	{
		delete g_joints[iJoint];
	}
	g_joints.resize(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NodeTranslational * addTranslationalNode(float invMass, const math::Vec3 & pos, const math::Vec3 & vel)
{
	NodeTranslational * newNode = new NodeTranslational;
	g_nodes.push_back(newNode);

	newNode->initIndex();

	newNode->m_f = math::Vec3C();
	newNode->m_vel = vel;

	newNode->m_dampingMul = 0.999f;
	newNode->m_dampingSub = 0.0025f;

	newNode->m_pos = pos;
	newNode->m_invMass = invMass;

	newNode->m_skinWidth = 0.025f;
	newNode->m_frictionCoeff = 1.5f;
	newNode->m_restitutionCoeff = 0.1f;

	return newNode;
}

NodeRotational * addRotationalNode(const math::Mat33 & invInertia0, const math::Quat & orient, const math::Vec3 & angvel)
{
	NodeRotational * newNode = new NodeRotational;
	g_nodes.push_back(newNode);

	newNode->initIndex();

	newNode->m_flags = (NodeRotational::Flags)0;

	newNode->m_f = math::Vec3C();
	newNode->m_vel = angvel;

	newNode->m_dampingMul = 0.999f;
	newNode->m_dampingSub = 0.0025f;

	newNode->m_rot = orient;
	newNode->m_invInertia = invInertia0;

	newNode->m_com = math::Vec3C();

	return newNode;
}

void initNodes()
{
	// Setup actor 0 (world attachment actor)
	addTranslationalNode(0.0f, math::Vec3C(), math::Vec3C());
}

void cleanupNodes()
{
	for (size_t iNode = 0, iNodeEnd = g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		delete g_nodes[iNode];
	}
	g_nodes.resize(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initPhysics()
{
	g_contactMassScaleParams.isEnabled = false;

	g_contactJointPool.setPreallocationNum(1000);
	g_triNodeContactJointPool.setPreallocationNum(1000);
	initNodes();
}

void cleanupPhysics()
{
	cleanupTrackedGeometryShapes();
	cleanupTrackedConvexHulls();
	cleanupRigidBodies();
	cleanupJoints();
	cleanupNodes();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateNodesPreIntegrate(SolverBase * solver, float dt, const math::Vec3 & gravity)
{
	using namespace math;

	float *ptr_m_Ftot_x = &solver->m_Ftot_x[0];
	float *ptr_m_Ftot_y = &solver->m_Ftot_y[0];
	float *ptr_m_Ftot_z = &solver->m_Ftot_z[0];

	float *ptr_m_NodeVel_x = &solver->m_nodeVel_x[0];
	float *ptr_m_NodeVel_y = &solver->m_nodeVel_y[0];
	float *ptr_m_NodeVel_z = &solver->m_nodeVel_z[0];

	float *ptr_m_NodeInvMass_00 = &solver->m_nodeInvMass_00[0];
	float *ptr_m_NodeInvMass_01 = &solver->m_nodeInvMass_01[0];
	float *ptr_m_NodeInvMass_02 = &solver->m_nodeInvMass_02[0];
	float *ptr_m_NodeInvMass_10 = &solver->m_nodeInvMass_10[0];
	float *ptr_m_NodeInvMass_11 = &solver->m_nodeInvMass_11[0];
	float *ptr_m_NodeInvMass_12 = &solver->m_nodeInvMass_12[0];
	float *ptr_m_NodeInvMass_20 = &solver->m_nodeInvMass_20[0];
	float *ptr_m_NodeInvMass_21 = &solver->m_nodeInvMass_21[0];
	float *ptr_m_NodeInvMass_22 = &solver->m_nodeInvMass_22[0];

	for (size_t iNode = 0, iNodeEnd = g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		g_nodes[iNode]->updateIndex((uint32_t)iNode);
		if (g_nodes[iNode]->getType() == Node::Type::eTranslational)
		{
			NodeTranslational * curNodeTrn = static_cast<NodeTranslational *>(g_nodes[iNode]);

			ptr_m_NodeVel_x[iNode] = curNodeTrn->m_vel.x;
			ptr_m_NodeVel_y[iNode] = curNodeTrn->m_vel.y;
			ptr_m_NodeVel_z[iNode] = curNodeTrn->m_vel.z;

			ptr_m_NodeInvMass_00[iNode] = curNodeTrn->m_invMass;
			ptr_m_NodeInvMass_01[iNode] = 0.0f;
			ptr_m_NodeInvMass_02[iNode] = 0.0f;
			ptr_m_NodeInvMass_10[iNode] = 0.0f;
			ptr_m_NodeInvMass_11[iNode] = curNodeTrn->m_invMass;
			ptr_m_NodeInvMass_12[iNode] = 0.0f;
			ptr_m_NodeInvMass_20[iNode] = 0.0f;
			ptr_m_NodeInvMass_21[iNode] = 0.0f;
			ptr_m_NodeInvMass_22[iNode] = curNodeTrn->m_invMass;

			math::Vec3 fTotal;
			fTotal = curNodeTrn->m_vel + (dt * curNodeTrn->m_invMass) * curNodeTrn->m_f;

			if (curNodeTrn->m_invMass > 0.0f)
			{
				fTotal += dt * gravity;
			}

			ptr_m_Ftot_x[iNode] = fTotal.x;
			ptr_m_Ftot_y[iNode] = fTotal.y;
			ptr_m_Ftot_z[iNode] = fTotal.z;
		}
		else
		{
			NodeRotational * curNodeRot = static_cast<NodeRotational *>(g_nodes[iNode]);

			ptr_m_NodeVel_x[iNode] = curNodeRot->m_vel.x;
			ptr_m_NodeVel_y[iNode] = curNodeRot->m_vel.y;
			ptr_m_NodeVel_z[iNode] = curNodeRot->m_vel.z;

			math::Mat33 rot, tensor;
			rot = math::Quat::toMatrix33(curNodeRot->m_rot);

			tensor = rot * curNodeRot->m_invInertia * rot.getTransposed();

			ptr_m_NodeInvMass_00[iNode] = tensor.t[0][0];
			ptr_m_NodeInvMass_01[iNode] = tensor.t[0][1];
			ptr_m_NodeInvMass_02[iNode] = tensor.t[0][2];
			ptr_m_NodeInvMass_10[iNode] = tensor.t[1][0];
			ptr_m_NodeInvMass_11[iNode] = tensor.t[1][1];
			ptr_m_NodeInvMass_12[iNode] = tensor.t[1][2];
			ptr_m_NodeInvMass_20[iNode] = tensor.t[2][0];
			ptr_m_NodeInvMass_21[iNode] = tensor.t[2][1];
			ptr_m_NodeInvMass_22[iNode] = tensor.t[2][2];

			math::Vec3 fTotal;
			fTotal = curNodeRot->m_vel + dt * (tensor * curNodeRot->m_f);

			ptr_m_Ftot_x[iNode] = fTotal.x;
			ptr_m_Ftot_y[iNode] = fTotal.y;
			ptr_m_Ftot_z[iNode] = fTotal.z;
		}
	}
}

void readbackNodesIntegrate(SolverBase * solver, float dt)
{
	using namespace math;

	float *ptr_m_a_x = &solver->m_a_x[0];
	float *ptr_m_a_y = &solver->m_a_y[0];
	float *ptr_m_a_z = &solver->m_a_z[0];

	float *ptr_m_Ftot_x = &solver->m_Ftot_x[0];
	float *ptr_m_Ftot_y = &solver->m_Ftot_y[0];
	float *ptr_m_Ftot_z = &solver->m_Ftot_z[0];

	float *ptr_m_NodeInvMass_00 = &solver->m_nodeInvMass_00[0];
	float *ptr_m_NodeInvMass_01 = &solver->m_nodeInvMass_01[0];
	float *ptr_m_NodeInvMass_02 = &solver->m_nodeInvMass_02[0];
	float *ptr_m_NodeInvMass_10 = &solver->m_nodeInvMass_10[0];
	float *ptr_m_NodeInvMass_11 = &solver->m_nodeInvMass_11[0];
	float *ptr_m_NodeInvMass_12 = &solver->m_nodeInvMass_12[0];
	float *ptr_m_NodeInvMass_20 = &solver->m_nodeInvMass_20[0];
	float *ptr_m_NodeInvMass_21 = &solver->m_nodeInvMass_21[0];
	float *ptr_m_NodeInvMass_22 = &solver->m_nodeInvMass_22[0];

	// Skipping Node#0 - the fixed actor node
	for (size_t iNode = 1, iNodeEnd = g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		Node * curNode = static_cast<Node *>(g_nodes[iNode]);

		// Update velocities using values caluclated in solver
		curNode->m_vel.x = ptr_m_a_x[iNode] + ptr_m_Ftot_x[iNode];
		curNode->m_vel.y = ptr_m_a_y[iNode] + ptr_m_Ftot_y[iNode];
		curNode->m_vel.z = ptr_m_a_z[iNode] + ptr_m_Ftot_z[iNode];

		curNode->m_f = math::Vec3C(0.0f, 0.0f, 0.0f);

		// Integrate [semi-implicit Euler]
		if (g_nodes[iNode]->getType() == Node::Type::eTranslational)
		{
			NodeTranslational * curNodeTrn = static_cast<NodeTranslational *>(curNode);

			// Damping
			curNodeTrn->m_vel *= curNodeTrn->m_dampingMul;

			if (curNodeTrn->m_vel.sqLen() > curNodeTrn->m_dampingSub*curNodeTrn->m_dampingSub)
			{
				curNodeTrn->m_vel -= curNodeTrn->m_dampingSub * curNodeTrn->m_vel.getNormalized();
			}
			else
			{
				curNodeTrn->m_vel = Vec3C(0.0f, 0.0f, 0.0f);
			}

			curNodeTrn->m_pos += curNodeTrn->m_vel * dt;
		}
		else
		{
			NodeRotational * curNodeRot = static_cast<NodeRotational *>(curNode);

			// Damping
			curNodeRot->m_vel *= curNodeRot->m_dampingMul;

			if (curNodeRot->m_vel.sqLen() > curNodeRot->m_dampingSub*curNodeRot->m_dampingSub)
			{
				curNodeRot->m_vel -= curNodeRot->m_dampingSub * curNodeRot->m_vel.getNormalized();
			}
			else
			{
				curNodeRot->m_vel = Vec3C(0.0f, 0.0f, 0.0f);
			}
			
			if (fast_abs(curNodeRot->m_invInertia.det()) < 1e-6f)
				continue;


			if (((uint32_t)curNodeRot->m_flags & (uint32_t)NodeRotational::Flags::eNoGyroscopic) == 0)
			{
				Mat33 rot = Quat::toMatrix33(curNodeRot->m_rot);
				Mat33 inertiaL = curNodeRot->m_invInertia;
				inertiaL.invert();
				Vec3 velL = rot.getTransposed() * curNodeRot->m_vel;

				Vec3 angMomentumL = inertiaL * velL;
				Vec3 gyroscopicTorque = velL.cross(angMomentumL);

				Vec3 gyroscopicImpulseImp;

				// Implicit integration contribution to the gyroscopic
				float angVelSqLen = curNodeRot->m_vel.sqLen();
				// Empirical hack: interpolate between explicit and implicit integrations at the lower angular velocities
				//	to compensate for the implicit integration energy dissipation
				const float implK = clamp(0.5f + 0.5f * angVelSqLen / 5000.0f, 0.0f, 1.0f);

				if (implK != 0.0f)
				{
					// Implicit gyrscopic force integration
					//	J*dw = f (rhs)
					// For mathematical background of this implicit integration, please see E.Catto's GDC presentation
					//	"Numerical Methods"
					Vec3 rhs = dt * gyroscopicTorque;

					Mat33 J = inertiaL + dt * (Mat33().crossProdMat(velL) * inertiaL - Mat33().crossProdMat(inertiaL * velL));
					Mat33 invJ = J;
					invJ.invert();
					gyroscopicImpulseImp = rot * -(invJ * rhs);
				}
				else
				{
					gyroscopicImpulseImp = Vec3C();
				}

				Vec3 gyroscopicImpulseExp;
				if (implK != 1.0f)
				{
					// UNSTABLE
					// Explicit gyroscopic force integration in local space
					//	Tg = w (angular velocity) x L (angular momentum)
					//	v' = v - dt*I^{-1}*Tg
					gyroscopicImpulseExp = rot * (-dt * (curNodeRot->m_invInertia * gyroscopicTorque));
				}
				else
				{
					gyroscopicImpulseExp = Vec3C();
				}

				curNodeRot->m_vel += gyroscopicImpulseImp * (implK) + gyroscopicImpulseExp * (1.0f - implK);
			}

			math::Quat angVel(curNode->m_vel, 0.0f);
			curNodeRot->m_rot += dt * 0.5f * (angVel * curNodeRot->m_rot);
			curNodeRot->m_rot.normalize();
		}
	}
}

struct PerfTimings
{
	double pre_integrate;
	double bp_candidates;
	double bp_buildPairsList;
	double cd_NP_contactGen;
	double contactCache_match;
	double bp_candidates_store;
	double vel_numRows_precompute;
	double vel_updateConstr;
	double vel_solve;
	double vel_fetchLambdas;
	double velFric_numRows;
	double velFric_updateConstr;
	double velFric_solve;
	double velFric_fetchLambdas;
	double integrate;
	double contactCache_update;
	double contactCache_accel;
	double totalFrameTime;

	double getTotalTime()
	{
		return
			pre_integrate + 
			bp_candidates + 
			bp_buildPairsList + 
			cd_NP_contactGen + 
			contactCache_match + 
			bp_candidates_store + 
			vel_numRows_precompute + 
			vel_updateConstr + 
			vel_solve + 
			vel_fetchLambdas + 
			velFric_numRows + 
			velFric_updateConstr + 
			velFric_solve + 
			velFric_fetchLambdas + 
			integrate + 
			contactCache_update + 
			contactCache_accel; 
			//totalFrameTime;
	}

	void init()
	{
		pre_integrate = 0.0;
		bp_candidates = 0.0;
		bp_buildPairsList = 0.0;
		cd_NP_contactGen = 0.0;
		contactCache_match = 0.0;
		bp_candidates_store = 0.0;
		vel_numRows_precompute = 0.0;
		vel_updateConstr = 0.0;
		vel_solve = 0.0;
		vel_fetchLambdas = 0.0;
		velFric_numRows = 0.0;
		velFric_updateConstr = 0.0;
		velFric_solve = 0.0;
		velFric_fetchLambdas = 0.0;
		integrate = 0.0;
		contactCache_update = 0.0;
		contactCache_accel = 0.0;
		totalFrameTime = 0.0;
	}
};
PerfTimings g_perfTimings;
#define DBG_ENABLE_PERF_TIMINGS 0

struct CollisionCandidate
{
	enum class BitFlags : uint32_t
	{
		eSinglePointContact =	0b0000000000000000000000000001,
		eTriNodeCandidate =		0b0000000000000000000000000010,

		eNone = 0,
	};
	BitFlags bitFlags;

	NodeTranslational * nodeLin;

	// Not used if `eTriNodeCandidate` is not set
	NodeTranslational * nodeLinEx0;
	NodeTranslational * nodeLinEx1;

	NodeRotational * nodeRot;
	GeometryShape * shape;
	GeometryShape::Type shapeGeomType;
	math::Vec3 shapePos;
	math::Mat33 shapeRot;

	math::Vec3 m_aabbMin;
	math::Vec3 m_aabbMax;
};

PoolBasic<CollisionCandidate> g_collisionCandidatePool;

struct CollisionDataStorage
{
	std::vector<math::Vec3> supportPoints0;
	std::vector<math::Vec3> supportPoints1;

	std::vector<uint32_t> convexPtIndices;
	std::vector<math::Vec3> convexPolygon0;
	std::vector<math::Vec3> convexPolygon1;

	std::vector<math::Vec3> clipBuf0;
	std::vector<math::Vec3> clipBuf1;
	std::vector<float> clipPlanes;
	std::vector<math::Vec3> clippedPoly0;

	std::vector<math::Vec3> contactManifold;
	std::vector<float> contactDepths;

	std::vector<math::Vec3> finalContactPoints0;
};
CollisionDataStorage g_collisionData;

struct CollisionPairs
{
	CollisionCandidate * candidate0;
	CollisionCandidate * candidate1;
};
std::vector<CollisionPairs> g_collisionPairs;

void simPrepareStep()
{
	simResetDrawing();
	g_perfTimings.init();
}

std::vector<CollisionCandidate *> g_collisionCandidates;

// Contacts go here
std::vector<JointBase *> g_dynamicJoints;

// Fake shapes/geometries for FEM collision
ConvexSphere g_pointNodesConvexShape;
GeometryConvex g_pointNodesShape;

ConvexSphere g_pointTriNodesConvexShape;
GeometryConvex g_pointTriNodesShape;

void defaultBuildCollisionCandidatesList()
{
	using namespace math;

	// Generate list of candidates to feed into the broadphase CD

	// The function is a bridge function between the app and the physics framework - and it mixes higher
	//	level abstractions (rigid bodies) with lower level data structures like geometries and candidates.
	// Ideally, this function should be replaced with something more suitable for the app, with custom
	//	candidates, etc.

	// First, go through all higher-level abstractions like rigid body (they are higher level than nodes),
	//	and put the related geometries into candidates list, calculating all the required info
	// Second, go through the linear nodes and create proxy geometries/candidates for them (at the moment,
	//	if linear node is followed by a rotational node, the couple is considered a rigid body, otherwise -
	//	it is part of a deformable system). We don't want to have many contacts per single node, hence
	//	SinglePointContact flag is set for these proxy candidates.
	// Lastly, generate a sphere for each face of the FEM joint tetrahedron to improve collision robustness,
	//	however this is disabled by default as it significantly increases the amount of candidates. This path
	//	also sets flag - in addition to a single point contact - TriNodeCandidate, which specifies that
	//	different type of joint should be generated if contact is found, the joint that acts on a single rigid
	//	body (linear+rotational couple) and three linear nodes (FE face).

	// TODO:
	//	- FE face spheres - use ellipsoids instead of spheres to better fill the face
	//	- tetrahedron-based FE collision detecion

	auto drawWireBox = [](const Vec3 & meshOBBMin, const Vec3 & meshOBBMax, const Mat44 & meshModelMatrix, const Vec4 & color = Vec4C(0.0f, 1.0f, 0.0f, 1.0f))
	{
		Vec4 color0, color1;

		color0 = color;
		color1 = color;

		Vec3 meshOBB[8];
		meshOBB[0] = meshModelMatrix * Vec3C(meshOBBMin.x, meshOBBMin.y, meshOBBMin.z);
		meshOBB[1] = meshModelMatrix * Vec3C(meshOBBMax.x, meshOBBMin.y, meshOBBMin.z);
		meshOBB[2] = meshModelMatrix * Vec3C(meshOBBMax.x, meshOBBMax.y, meshOBBMin.z);
		meshOBB[3] = meshModelMatrix * Vec3C(meshOBBMin.x, meshOBBMax.y, meshOBBMin.z);
		meshOBB[4] = meshModelMatrix * Vec3C(meshOBBMin.x, meshOBBMin.y, meshOBBMax.z);
		meshOBB[5] = meshModelMatrix * Vec3C(meshOBBMax.x, meshOBBMin.y, meshOBBMax.z);
		meshOBB[6] = meshModelMatrix * Vec3C(meshOBBMax.x, meshOBBMax.y, meshOBBMax.z);
		meshOBB[7] = meshModelMatrix * Vec3C(meshOBBMin.x, meshOBBMax.y, meshOBBMax.z);

		{
			simDrawLine(meshOBB[0], meshOBB[1], color0, color1);
			simDrawLine(meshOBB[1], meshOBB[2], color0, color1);
			simDrawLine(meshOBB[2], meshOBB[3], color0, color1);
			simDrawLine(meshOBB[3], meshOBB[0], color0, color1);

			simDrawLine(meshOBB[4], meshOBB[5], color0, color1);
			simDrawLine(meshOBB[5], meshOBB[6], color0, color1);
			simDrawLine(meshOBB[6], meshOBB[7], color0, color1);
			simDrawLine(meshOBB[7], meshOBB[4], color0, color1);

			simDrawLine(meshOBB[0], meshOBB[4], color0, color1);
			simDrawLine(meshOBB[1], meshOBB[5], color0, color1);
			simDrawLine(meshOBB[2], meshOBB[6], color0, color1);
			simDrawLine(meshOBB[3], meshOBB[7], color0, color1);
		}
	};

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
#endif

	const bool renderRigidBodiesAABB = false;

	g_collisionCandidates.resize(0);
	g_collisionCandidates.reserve(g_rigidBodies.size() * 2);

	// Fill broadphase candidates array
	for (size_t irb = 0, irbEnd = g_rigidBodies.size(); irb < irbEnd; ++irb)
	{
		RigidBody * curBody = g_rigidBodies[irb];
		for (size_t gi = 0, giEnd = curBody->m_geometryShapes.size(); gi < giEnd; ++gi)
		{
			GeometryShape * curShape = curBody->m_geometryShapes[gi];

			CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
			new (collisionCandidatePtr) CollisionCandidate;

			CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

			collisionCandidate.bitFlags = CollisionCandidate::BitFlags::eNone;
			collisionCandidate.nodeLin = curBody->m_bodyL;
			collisionCandidate.nodeRot = curBody->m_bodyR;
			collisionCandidate.shape = curShape;
			collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

			Mat34 shapeTransform;
			Mat34 bodyTransform;
			Mat34 totalTransform;

			shapeTransform = curShape->m_rotation;
			shapeTransform.fillTranslation(curShape->m_origin - curBody->m_bodyR->m_com);

			bodyTransform = Quat::toMatrix33(curBody->m_bodyR->m_rot);
			bodyTransform.fillTranslation(curBody->m_bodyL->m_pos);

			totalTransform = bodyTransform * shapeTransform;

			collisionCandidate.shapePos = totalTransform.getBasis3();//curBody->m_bodyL->m_pos + curBody->m_bodyR->m_rot.rotate(collisionCandidate.shape->m_origin);
			collisionCandidate.shapeRot = totalTransform.getRotation33();//Quat::ToMatrix34(curBody->m_bodyR->m_rot);

			Quat invQuat = curBody->m_bodyR->m_rot.getConjugated();
			Vec3 shapeLocalDirX = curShape->m_rotation.invRotateCopy(invQuat.rotate(Vec3C(1.0f, 0.0f, 0.0f)));
			Vec3 shapeLocalDirY = curShape->m_rotation.invRotateCopy(invQuat.rotate(Vec3C(0.0f, 1.0f, 0.0f)));
			Vec3 shapeLocalDirZ = curShape->m_rotation.invRotateCopy(invQuat.rotate(Vec3C(0.0f, 0.0f, 1.0f)));

			Vec3 bpMinX = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint(-shapeLocalDirX)));
			Vec3 bpMaxX = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint( shapeLocalDirX)));
			Vec3 bpMinY = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint(-shapeLocalDirY)));
			Vec3 bpMaxY = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint( shapeLocalDirY)));
			Vec3 bpMinZ = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint(-shapeLocalDirZ)));
			Vec3 bpMaxZ = curBody->m_bodyR->m_rot.rotate(curShape->m_rotation * (curShape->getBoundingPoint( shapeLocalDirZ)));

			collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(bpMinX.x, bpMinY.y, bpMinZ.z);
			collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C(bpMaxX.x, bpMaxY.y, bpMaxZ.z);

			if (renderRigidBodiesAABB)
			{
				drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			}

			g_collisionCandidates.push_back(collisionCandidatePtr);
		}
	}

#if 1
	const float linearNodeRad = 0.3f;

	g_pointNodesConvexShape.m_radius = linearNodeRad;
	g_pointNodesConvexShape.m_origin = Vec3C();

	g_pointNodesShape.m_origin = Vec3C();
	g_pointNodesShape.m_rotation.identity();
	g_pointNodesShape.m_density = 0.1f;
	g_pointNodesShape.m_convexHullShape = &g_pointNodesConvexShape;

	// Add strictly linear nodes as candidates too
	// Skipping node0 as it is special static node
	for (size_t in = 1, inEnd = g_nodes.size(); in < inEnd; ++in)
	{
		//if (g_nodes[in]->getType() == Node::Type::eTranslational && ((in != inEnd-1) && g_nodes[in]->getType() != Node::Type::eTranslational))
		if (g_nodes[in]->getType() != Node::Type::eTranslational || (in != (inEnd-1) && physics::g_nodes[in+1]->getType() == Node::Type::eRotational))
			continue;

		NodeTranslational * curNodeTrn = static_cast<NodeTranslational *>(g_nodes[in]);

		CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
		new (collisionCandidatePtr) CollisionCandidate;

		CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

		collisionCandidate.bitFlags = CollisionCandidate::BitFlags::eSinglePointContact;
		collisionCandidate.nodeLin = curNodeTrn;
		collisionCandidate.nodeRot = nullptr;
		collisionCandidate.shape = &g_pointNodesShape;
		collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

		collisionCandidate.shapePos = curNodeTrn->m_pos;
		collisionCandidate.shapeRot.identity();

		collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(-linearNodeRad, -linearNodeRad, -linearNodeRad);
		collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C( linearNodeRad,  linearNodeRad,  linearNodeRad);

		if (false)
		{
			drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
		}

		g_collisionCandidates.push_back(collisionCandidatePtr);
	}
#endif

#if 0
	const float triNodeRad = 0.3f;

	g_pointTriNodesConvexShape.m_radius = triNodeRad;
	g_pointTriNodesConvexShape.m_origin = Vec3C();

	g_pointTriNodesShape.m_origin = Vec3C();
	g_pointTriNodesShape.m_rotation.identity();
	g_pointTriNodesShape.m_density = 0.1f;
	g_pointTriNodesShape.m_convexHullShape = &g_pointTriNodesConvexShape;

	// Add strictly linear nodes as candidates too
	for (size_t ij = 0, ijEnd = g_joints.size(); ij < ijEnd; ++ij)
	{
		if (g_joints[ij]->getType() != JointBase::Type::eFEM)
			continue;

		FEMJoint * curJoint = static_cast<FEMJoint *>(g_joints[ij]);

		NodeTranslational * n0 = curJoint->getNode0();
		NodeTranslational * n1 = curJoint->getNode1();
		NodeTranslational * n2 = curJoint->getNode2();
		NodeTranslational * n3 = curJoint->getNode3();

		const float drawTriNodeAABBs = false;

		{
			CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
			new (collisionCandidatePtr) CollisionCandidate;

			CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

			collisionCandidate.bitFlags = (CollisionCandidate::BitFlags)(
				(uint32_t)CollisionCandidate::BitFlags::eSinglePointContact | (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate
				);
			collisionCandidate.nodeLin = n0;
			collisionCandidate.nodeRot = nullptr;
			collisionCandidate.nodeLinEx0 = n1;
			collisionCandidate.nodeLinEx1 = n2;
			collisionCandidate.shape = &g_pointTriNodesShape;
			collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

			collisionCandidate.shapePos = (collisionCandidate.nodeLin->m_pos + collisionCandidate.nodeLinEx0->m_pos + collisionCandidate.nodeLinEx1->m_pos) / 3.0f;
			collisionCandidate.shapeRot.identity();

			collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(-triNodeRad, -triNodeRad, -triNodeRad);
			collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C( triNodeRad,  triNodeRad,  triNodeRad);

			if (drawTriNodeAABBs)
			{
				drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			}

			g_collisionCandidates.push_back(collisionCandidatePtr);
		}

		{
			CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
			new (collisionCandidatePtr) CollisionCandidate;

			CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

			collisionCandidate.bitFlags = (CollisionCandidate::BitFlags)(
				(uint32_t)CollisionCandidate::BitFlags::eSinglePointContact | (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate
				);
			collisionCandidate.nodeLin = n0;
			collisionCandidate.nodeRot = nullptr;
			collisionCandidate.nodeLinEx0 = n3;
			collisionCandidate.nodeLinEx1 = n1;
			collisionCandidate.shape = &g_pointTriNodesShape;
			collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

			collisionCandidate.shapePos = (collisionCandidate.nodeLin->m_pos + collisionCandidate.nodeLinEx0->m_pos + collisionCandidate.nodeLinEx1->m_pos) / 3.0f;
			collisionCandidate.shapeRot.identity();

			collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(-triNodeRad, -triNodeRad, -triNodeRad);
			collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C( triNodeRad,  triNodeRad,  triNodeRad);

			if (drawTriNodeAABBs)
			{
				drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			}

			g_collisionCandidates.push_back(collisionCandidatePtr);
		}

		{
			CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
			new (collisionCandidatePtr) CollisionCandidate;

			CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

			collisionCandidate.bitFlags = (CollisionCandidate::BitFlags)(
				(uint32_t)CollisionCandidate::BitFlags::eSinglePointContact | (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate
				);
			collisionCandidate.nodeLin = n0;
			collisionCandidate.nodeRot = nullptr;
			collisionCandidate.nodeLinEx0 = n2;
			collisionCandidate.nodeLinEx1 = n3;
			collisionCandidate.shape = &g_pointTriNodesShape;
			collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

			collisionCandidate.shapePos = (collisionCandidate.nodeLin->m_pos + collisionCandidate.nodeLinEx0->m_pos + collisionCandidate.nodeLinEx1->m_pos) / 3.0f;
			collisionCandidate.shapeRot.identity();

			collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(-triNodeRad, -triNodeRad, -triNodeRad);
			collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C( triNodeRad,  triNodeRad,  triNodeRad);

			if (drawTriNodeAABBs)
			{
				drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			}

			g_collisionCandidates.push_back(collisionCandidatePtr);
		}

		{
			CollisionCandidate * collisionCandidatePtr = g_collisionCandidatePool.getElementMemory();
			new (collisionCandidatePtr) CollisionCandidate;

			CollisionCandidate & collisionCandidate = *collisionCandidatePtr;

			collisionCandidate.bitFlags = (CollisionCandidate::BitFlags)(
				(uint32_t)CollisionCandidate::BitFlags::eSinglePointContact | (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate
				);
			collisionCandidate.nodeLin = n1;
			collisionCandidate.nodeRot = nullptr;
			collisionCandidate.nodeLinEx0 = n3;
			collisionCandidate.nodeLinEx1 = n2;
			collisionCandidate.shape = &g_pointTriNodesShape;
			collisionCandidate.shapeGeomType = collisionCandidate.shape->getType();

			collisionCandidate.shapePos = (collisionCandidate.nodeLin->m_pos + collisionCandidate.nodeLinEx0->m_pos + collisionCandidate.nodeLinEx1->m_pos) / 3.0f;
			collisionCandidate.shapeRot.identity();

			collisionCandidate.m_aabbMin = collisionCandidate.shapePos + Vec3C(-triNodeRad, -triNodeRad, -triNodeRad);
			collisionCandidate.m_aabbMax = collisionCandidate.shapePos + Vec3C( triNodeRad,  triNodeRad,  triNodeRad);

			if (drawTriNodeAABBs)
			{
				drawWireBox(collisionCandidate.m_aabbMin, collisionCandidate.m_aabbMax, Mat44().identity(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			}

			g_collisionCandidates.push_back(collisionCandidatePtr);
		}
	}
#endif

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.bp_candidates = perfTimer.time();
#endif
}

void defaultBroadphase()
{
	// Broadphase colision detection function

	// Given collision candidates (mostly, their AABBs) - cheaply reject pairs that do not overlap;
	//	perform additional filtering based on parent rigid body, etc.
	// This Broadphase implementation is straightforward O(N*N) pairwise tests.
	// There are a lot alternative ways to implement Broadphase CD, e.g. Sweep-and-Prune,
	//	H-Grid, uniform grid, various hierarchies/trees; but the bruteforce one was good enough for
	//	solvers comparison - and implementing better BP CD was quite low on the TODO list.

	// TODO:
	//	- implement computationally more effective algorithm;
	//	- additional filtering schemes to e.g. allow limb parts overlap and not generate collisions;
	//	- broadphase triggers - candidates that will not be propagated to the NP pairs list, but
	//		instead to user-facing pairs list to allow for things like event triggers etc.);

	using namespace math;

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
#endif

	const size_t numCandidates = g_collisionCandidates.size();
	CollisionCandidate ** ptr_g_collisionCandidates = g_collisionCandidates.data();

	g_collisionPairs.resize(0);
	g_collisionPairs.reserve(g_collisionCandidates.size() * 2);

	for (size_t ic0 = 0; ic0 < numCandidates; ++ic0)
	{
		CollisionCandidate * curCandidate0 = ptr_g_collisionCandidates[ic0];
		for (size_t ic1 = ic0 + 1; ic1 < numCandidates; ++ic1)
		{
			CollisionCandidate * curCandidate1 = ptr_g_collisionCandidates[ic1];

			// Do not collide linear nodes with each other
			// TODO: make this optional per-node
			if (curCandidate0->nodeRot == nullptr && curCandidate1->nodeRot == nullptr)
				continue;

			// Skip candidates if they both TriNode (should really be skipped with the above check, but this here is for clarity)
			if (((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate) &&
				((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate))
			{
				continue;
			}

			bool isNode0Dynamic = curCandidate0->nodeLin && (curCandidate0->nodeLin->m_invMass != 0.0f);
			bool isNode1Dynamic = curCandidate1->nodeLin && (curCandidate1->nodeLin->m_invMass != 0.0f);
			if (!isNode0Dynamic && !isNode1Dynamic)
				continue;

			// Skip shapes belonging to the same body
			if (curCandidate0->nodeLin != nullptr && (curCandidate0->nodeLin == curCandidate1->nodeLin))
				continue;
			if (curCandidate0->nodeRot != nullptr && (curCandidate0->nodeRot == curCandidate1->nodeRot))
				continue;

			if (curCandidate0->m_aabbMax.x < curCandidate1->m_aabbMin.x || curCandidate1->m_aabbMax.x < curCandidate0->m_aabbMin.x ||
				curCandidate0->m_aabbMax.y < curCandidate1->m_aabbMin.y || curCandidate1->m_aabbMax.y < curCandidate0->m_aabbMin.y ||
				curCandidate0->m_aabbMax.z < curCandidate1->m_aabbMin.z || curCandidate1->m_aabbMax.z < curCandidate0->m_aabbMin.z)
				continue;

			// Pair passed broadphase tests
			g_collisionPairs.push_back({curCandidate0, curCandidate1});
		}
	}

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.bp_buildPairsList = perfTimer.time();
#endif
}

void defaultMPRNarrowphase(const math::Vec3 & gravityVec)
{
	using namespace math;

	// Function that checks if filtered set of pairs really overlaps, and generates contacts

	// This implementation is based on convex-convex collision overlapping test called
	//	Minkowsi Portal Refinement (MPR)

	// Function calculates minimum translation distance approximation using two-step MPR,
	//	and then uses the normal to produce contact features and generate contacts based
	//	on the features produced.

	// The high-level logic:
	//	- for a pair, run the MPR algorithm in the direction of some CSO interoir point to the CSO origin;
	//		interoir point is selected as difference between geometric centers of convex hulls;
	//	- if we know there's an overlap, try to improve contact normal by running MPR once again,
	//		but now - in the direction of the previously generated portal normal, this should produce
	//		portal normal that is a better approximation of a minimum translation distance;
	//	- now as we have good enough approximation of the MTD, we could use it as a contact normal, and
	//		we now want to obtain features to generate contacts from; if SinglePointContact flag is set for
	//		the candidate - feature would be vertex/point. Otherwise, several samples around the contact
	//		normals will be taken that will define the contact feature;
	//	- generated points of the contact feature go through the duplicate removal procedure, since often
	//		the feature expansion will sample same support maps;
	//	- now as we have unique feature vertices num for both collision features, contact generation will
	//		start based on these numbers (Face-Vertex, Edge-Edge, etc.); the most interesting out of those
	//		is Face-Face contact generation;
	//	- contact generation produces dynamic contact joints of desired type (either conventional contacts
	//		or TriNodeContacts);

	// Face-Face contact generation high-level overview:
	//	- the first step is to get ordered convex polygons for each Face feature, in order to do this,
	//		simple gift wrap algorithm is launched, and then the normals calculated for each Face, with
	//		additional winding check; important note is that face feature vertices produced - considered
	//		to be planar;
	//	- with the convex wrapping of Face features, the polygon clipping routine is launched, based on
	//		a simple Sutherland-Hodgeman clipping; it simply clips one polygon with the planes built upon
	//		the edges of a second polygon (efectively extruding the polygon into prism);
	//	- each clipped point gets checked whether it actually lies under the contact feature that was
	//		forming the clipping prism, and if it doesn't - it gets rejected;
	//	- (optional) contact reduction - for a stable convex-convex resting contact up to 4 points are
	//		needed, and in most cases excessive contacts could be removed, and routine that builds maximum
	//		area polygon with no more than 4 vertices could be launched;

	// TODO:
	//	- alternative contact normal calculation schemes (GJK+EPA);
	//	- simplified specialized narrowphase CD functions (e.g. sphere-sphere, capsule-sphere,
	//		SAT-based cube-cube, etc.);
	//	- revive and finalize contact reduction;

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
#endif

	auto auxSphSph = [](const Vec3 & p0, scalar r0, const Vec3 & p1, scalar r1, Vec3 & contactPoint0, Vec3 & contactPoint1, Vec3 & normal) -> int
	{
		Vec3 vSph2Sph = p1 - p0;
		scalar distSq = vSph2Sph.sqLen();

		if (distSq < sqr(r0 + r1))
		{
			if (distSq < FP_PRECISION * FP_PRECISION)
			{
				normal = Vec3C(0, 1, 0);
			}
			else
			{
				normal = vSph2Sph.getNormalized();
			}
			contactPoint0 = p0 + normal * r0;
			contactPoint1 = p1 - normal * r1;

			return 1;
		}
		else
			return 0;
	};

	g_collisionData.supportPoints0.resize(0);
	g_collisionData.supportPoints1.resize(0);

	g_collisionData.convexPtIndices.resize(0);
	g_collisionData.convexPolygon0.resize(0);
	g_collisionData.convexPolygon1.resize(0);

	g_collisionData.clipBuf0.resize(0);
	g_collisionData.clipBuf1.resize(0);
	g_collisionData.clipPlanes.resize(0);
	g_collisionData.clippedPoly0.resize(0);

	g_collisionData.contactManifold.resize(0);
	g_collisionData.contactDepths.resize(0);

	g_collisionData.finalContactPoints0.resize(0);

	// Collision detection
	const float triContactERP = 0.2f;
	const float triContactCFM = 0.005f;
	const float contactERP = 0.5f;
	const float contactCFM = 0.01f;

	CollisionPairs * ptr_g_collisionPairs = g_collisionPairs.data();
	for (size_t pairIdx = 0, pairIdxEnd = g_collisionPairs.size(); pairIdx < pairIdxEnd; ++pairIdx)
	{
		CollisionCandidate * curCandidate0 = ptr_g_collisionPairs[pairIdx].candidate0;
		CollisionCandidate * curCandidate1 = ptr_g_collisionPairs[pairIdx].candidate1;

		GeometryShape::Type curCandidate0GeomType = curCandidate0->shapeGeomType;
		GeometryShape::Type curCandidate1GeomType = curCandidate1->shapeGeomType;

		if (false && curCandidate0GeomType == GeometryShape::Type::eSphere && curCandidate1GeomType == GeometryShape::Type::eSphere)
		{
			float sphere1Rad = static_cast<GeometrySphere *>(curCandidate0->shape)->m_radius;
			float sphere2Rad = static_cast<GeometrySphere *>(curCandidate1->shape)->m_radius;
			Vec3 cp0, cp1, cn;
			if (auxSphSph(curCandidate0->shapePos, sphere1Rad, curCandidate1->shapePos, sphere2Rad, cp0, cp1, cn))
			{
				Contact * contactJoint = (Contact *)g_contactJointPool.getElementMemory();
				new (contactJoint) Contact;
				contactJoint->init(contactERP, contactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeRot);
				contactJoint->setContactInfo(cp0, cp1, cn);
				contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);
				g_dynamicJoints.push_back(contactJoint);
			}
		}
		else if (curCandidate0GeomType == GeometryShape::Type::eGenericConvex && curCandidate1GeomType == GeometryShape::Type::eGenericConvex)
		{
			GeometryConvex * convex0 = static_cast<GeometryConvex *>(curCandidate0->shape);
			GeometryConvex * convex1 = static_cast<GeometryConvex *>(curCandidate1->shape);

			ConvexHullShape * hullShape0 = convex0->m_convexHullShape;
			ConvexHullShape * hullShape1 = convex1->m_convexHullShape;

			Vec3 contactNormal, cp0, cp1;
			// Portals
			Vec3 pp00, pp01, pp02;
			Vec3 pp10, pp11, pp12;

			// TODO: convex hull scould be spheres as well, add the check here

			Quat s0rot = Quat::fromMatrix33(curCandidate0->shapeRot);
			s0rot.normalize();
			Quat s1rot = Quat::fromMatrix33(curCandidate1->shapeRot);
			s1rot.normalize();

			// First run of the MPR - setting interior points to the shape origins, as we
			//	don't know yet if collision happened, and what is the search direction
			const Vec3 shape0InteriorPoint = curCandidate0->shapePos;//hullShape0->getOrigin();
			const Vec3 shape1InteriorPoint = curCandidate1->shapePos;//hullShape1->getOrigin();
			bool collisionHit = mprContact(
									hullShape0, hullShape1,
									curCandidate0->shapePos, s0rot,
									curCandidate1->shapePos, s1rot,
									shape0InteriorPoint, shape1InteriorPoint,
									&contactNormal,
									&cp0, &cp1,
									&pp00, &pp01, &pp02,
									&pp10, &pp11, &pp12
									);

			if (collisionHit)
			{
				const bool enhanceMPRContact = true;
				if (enhanceMPRContact)
				{
					// Try to re-run the MPR, now in the direction of previously calculated contact normal -
					//	this could give better normal approximation for certain cases; however doesn't proove useful
					//	in most cases
					// TODO: try out additional enhancing, which will just sample several circles around the suggested normal
					//	and then find the descend direction and perform several steps in that direction, with possible subsampling
					//	to adjust the descending direction
					Vec3 interiorPoint0 = cp0 + contactNormal;
					Vec3 interiorPoint1 = cp1 - contactNormal;
					mprContact(
						hullShape0, hullShape1,
						curCandidate0->shapePos, s0rot,
						curCandidate1->shapePos, s1rot,
						interiorPoint0, interiorPoint1,
						&contactNormal,
						&cp0, &cp1,
						&pp00, &pp01, &pp02,
						&pp10, &pp11, &pp12
						);
				}

				// Make sure normal looks in the right direction
				// TODO: replace body pos with shape geometries world pos
				if (contactNormal.dot(curCandidate0->shapePos - curCandidate1->shapePos) < 0.0f)
					contactNormal = -contactNormal;

				// TEMP: figure out better calculations
				// Find support points along the contact normal
				Vec3 supportPoint0 = hullShape0->getSupportVertex(-contactNormal, curCandidate0->shapePos, s0rot);
				Vec3 supportPoint1 = hullShape1->getSupportVertex( contactNormal, curCandidate1->shapePos, s1rot);

				// Project the internal points to the surface
				float projDepth0 = (supportPoint0 - cp0).dot(contactNormal);
				float projDepth1 = (supportPoint1 - cp1).dot(contactNormal);

				// Move the reported points up to the shape surface
				Vec3 contactPoint0 = projDepth0 * contactNormal + cp0;
				Vec3 contactPoint1 = projDepth1 * contactNormal + cp1;


				// Expand contact surface
				const size_t startContactVertices = 0;
				const size_t numContactVertices = startContactVertices + 8;
				size_t numContactVertices0, numContactVertices1;

				g_collisionData.supportPoints0.resize(numContactVertices);
				g_collisionData.supportPoints1.resize(numContactVertices);

				Vec3 cnTan0, cnTan1;
				const float dirDelta = 0.1f;
				contactNormal.tangentSpace(cnTan0, cnTan1);

				// sin(pi/4)==cos(pi/4)==sqrt(2)/2
				const float sinPi_4 = 0.70710678118654752440084436210485f;
				const float dirDeltaDiag = dirDelta*sinPi_4;

				if ((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eSinglePointContact)
				{
					g_collisionData.supportPoints0[startContactVertices+0] = hullShape0->getSupportVertex(-contactNormal, curCandidate0->shapePos, s0rot);
					numContactVertices0 = startContactVertices+1;
				}
				else
				{
					// +-t0
					g_collisionData.supportPoints0[startContactVertices+0] = hullShape0->getSupportVertex(-(contactNormal+cnTan0*dirDelta), curCandidate0->shapePos, s0rot);
					g_collisionData.supportPoints0[startContactVertices+1] = hullShape0->getSupportVertex(-(contactNormal-cnTan0*dirDelta), curCandidate0->shapePos, s0rot);
					// +-t1
					g_collisionData.supportPoints0[startContactVertices+2] = hullShape0->getSupportVertex(-(contactNormal+cnTan1*dirDelta), curCandidate0->shapePos, s0rot);
					g_collisionData.supportPoints0[startContactVertices+3] = hullShape0->getSupportVertex(-(contactNormal-cnTan1*dirDelta), curCandidate0->shapePos, s0rot);
					// +-t0 +t1
					g_collisionData.supportPoints0[startContactVertices+4] = hullShape0->getSupportVertex( -(contactNormal+dirDeltaDiag*( cnTan0+cnTan1)), curCandidate0->shapePos, s0rot );
					g_collisionData.supportPoints0[startContactVertices+5] = hullShape0->getSupportVertex( -(contactNormal+dirDeltaDiag*(-cnTan0+cnTan1)), curCandidate0->shapePos, s0rot );
					// +-t0 -t1
					g_collisionData.supportPoints0[startContactVertices+6] = hullShape0->getSupportVertex( -(contactNormal+dirDeltaDiag*( cnTan0-cnTan1)), curCandidate0->shapePos, s0rot );
					g_collisionData.supportPoints0[startContactVertices+7] = hullShape0->getSupportVertex( -(contactNormal+dirDeltaDiag*(-cnTan0-cnTan1)), curCandidate0->shapePos, s0rot );
					numContactVertices0 = numContactVertices;
				}

				if ((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eSinglePointContact)
				{
					g_collisionData.supportPoints1[startContactVertices+0] = hullShape1->getSupportVertex( contactNormal, curCandidate1->shapePos, s1rot);
					numContactVertices1 = startContactVertices+1;
				}
				else
				{
					// +-t0
					g_collisionData.supportPoints1[startContactVertices+0] = hullShape1->getSupportVertex( (contactNormal+cnTan0*dirDelta), curCandidate1->shapePos, s1rot);
					g_collisionData.supportPoints1[startContactVertices+1] = hullShape1->getSupportVertex( (contactNormal-cnTan0*dirDelta), curCandidate1->shapePos, s1rot);
					// +-t1
					g_collisionData.supportPoints1[startContactVertices+2] = hullShape1->getSupportVertex( (contactNormal+cnTan1*dirDelta), curCandidate1->shapePos, s1rot);
					g_collisionData.supportPoints1[startContactVertices+3] = hullShape1->getSupportVertex( (contactNormal-cnTan1*dirDelta), curCandidate1->shapePos, s1rot);

					// +-t0 +t1
					g_collisionData.supportPoints1[startContactVertices+4] = hullShape1->getSupportVertex(  (contactNormal+dirDeltaDiag*( cnTan0+cnTan1)), curCandidate1->shapePos, s1rot );
					g_collisionData.supportPoints1[startContactVertices+5] = hullShape1->getSupportVertex(  (contactNormal+dirDeltaDiag*(-cnTan0+cnTan1)), curCandidate1->shapePos, s1rot );
					// +-t0 -t1
					g_collisionData.supportPoints1[startContactVertices+6] = hullShape1->getSupportVertex(  (contactNormal+dirDeltaDiag*( cnTan0-cnTan1)), curCandidate1->shapePos, s1rot );
					g_collisionData.supportPoints1[startContactVertices+7] = hullShape1->getSupportVertex(  (contactNormal+dirDeltaDiag*(-cnTan0-cnTan1)), curCandidate1->shapePos, s1rot );
					numContactVertices1 = numContactVertices;
				}

				uint32_t numUniqueVertices0;
				uint32_t numUniqueVertices1;
				removeDuplicatePointsInplace(g_collisionData.supportPoints0.data(), (uint32_t)numContactVertices0, &numUniqueVertices0);
				removeDuplicatePointsInplace(g_collisionData.supportPoints1.data(), (uint32_t)numContactVertices1, &numUniqueVertices1);

				const bool specialCaseContactProcessing = true;

				auto fallbackContact = [&curCandidate0, &curCandidate1, &contactERP, &contactCFM, &triContactERP, &triContactCFM, &gravityVec](const Vec3 & cp0, const Vec3 & cp1, const Vec3 & cn, bool renderContact = false)
				{
					bool flipContactNormal = ((cp0 - cp1).dot(cn) > 0.0f);
					if (((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate) ||
						((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate))
					{
						TriNodeContact * contactJoint = (TriNodeContact *)g_triNodeContactJointPool.getElementMemory();
						new (contactJoint) TriNodeContact;

						// Both TriNode contacts is not possible - should be filtered higher up the pipeline
						if ((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate)
						{
							// Candidate 0 is tri-node, need to exchange contact info
							contactJoint->init(triContactERP, triContactCFM, curCandidate1->nodeLin, curCandidate1->nodeRot, curCandidate0->nodeLin, curCandidate0->nodeLinEx0, curCandidate0->nodeLinEx1);
							contactJoint->setContactInfo(cp1, cp0, (!flipContactNormal) ? cn : -cn);
						}
						else
						{
							contactJoint->init(triContactERP, triContactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeLinEx0, curCandidate1->nodeLinEx1);
							contactJoint->setContactInfo(cp0, cp1, flipContactNormal ? cn : -cn);
						}

						g_dynamicJoints.push_back(contactJoint);
					}
					else
					{
						Contact * contactJoint = (Contact *)g_contactJointPool.getElementMemory();
						new (contactJoint) Contact;
						contactJoint->init(contactERP, contactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeRot);
						contactJoint->setContactInfo(cp0, cp1, flipContactNormal ? cn : -cn);
						contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);

						g_dynamicJoints.push_back(contactJoint);

						if (renderContact)
						{
							simDrawPoint(cp0, Vec4C(0.5f, 0.0f, 1.0f, 1.0f));
							simDrawPoint(cp1, Vec4C(0.0f, 0.5f, 1.0f, 1.0f));
							simDrawLine(cp0, cp0 + contactJoint->getContactNormalWorld(), Vec4C(0.5f, 0.0f, 1.0f, 1.0f), Vec4C(0.5f, 0.0f, 1.0f, 1.0f));
						}
					}
				};

				auto acceptVerticesBelowFace = [&curCandidate0, &curCandidate1, &contactERP, &contactCFM, &triContactERP, &triContactCFM, &gravityVec](const Vec3 * faceVertices, uint32_t numFaceVertices, const Vec3 & faceNormal, const Vec3 * candidateVertices, uint32_t numCandidateVertices, bool swapOrder, bool renderContact = false)
				{
					float facePlaneD = faceNormal.dot(faceVertices[0]);

					for (uint32_t ei = 0; ei < numCandidateVertices; ++ei)
					{
						const Vec3 & candidateVertex = candidateVertices[ei];
						float faceDist = faceNormal.dot(candidateVertex) - facePlaneD;
						if (faceDist < 0.0f)
						{
							if (((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate) ||
								((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate))
							{
								TriNodeContact * contactJoint = (TriNodeContact *)g_triNodeContactJointPool.getElementMemory();
								new (contactJoint) TriNodeContact;

								Vec3 cp0, cp1, cn;
								if (swapOrder)
								{
									cp0 = candidateVertex;
									cp1 = candidateVertex - faceNormal*faceDist;
									cn = -faceNormal;
								}
								else
								{
									cp0 = candidateVertex - faceNormal*faceDist;
									cp1 = candidateVertex;
									cn = faceNormal;
								}


								// Both TriNode contacts is not possible - should be filtered higher up the pipeline
								if ((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate)
								{
									// Candidate 0 is tri-node, need to exchange contact info
									contactJoint->init(triContactERP, triContactCFM, curCandidate1->nodeLin, curCandidate1->nodeRot, curCandidate0->nodeLin, curCandidate0->nodeLinEx0, curCandidate0->nodeLinEx1);
									contactJoint->setContactInfo(cp1, cp0, -cn);
								}
								else
								{
									contactJoint->init(triContactERP, triContactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeLinEx0, curCandidate1->nodeLinEx1);
									contactJoint->setContactInfo(cp0, cp1, cn);
								}

								g_dynamicJoints.push_back(contactJoint);
							}
							else
							{
								Contact * contactJoint = (Contact *)g_contactJointPool.getElementMemory();
								new (contactJoint) Contact;
								contactJoint->init(contactERP, contactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeRot);
								if (swapOrder)
								{
									contactJoint->setContactInfo(candidateVertex, candidateVertex - faceNormal*faceDist, -faceNormal);
									contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);
								}
								else
								{
									contactJoint->setContactInfo(candidateVertex - faceNormal*faceDist, candidateVertex, faceNormal);
									contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);
								}
								g_dynamicJoints.push_back(contactJoint);

								if (renderContact)
								{
									simDrawPoint(contactJoint->getContactPoint0World(), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
									simDrawPoint(contactJoint->getContactPoint1World(), Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
									simDrawLine(contactJoint->getContactPoint0World(), contactJoint->getContactPoint1World() + contactJoint->getContactNormalWorld(), Vec4C(1.0f, 0.0f, 0.0f, 1.0f), Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
								}
							}
						}
					}
				};

				Vec3 * supportPoints0 = g_collisionData.supportPoints0.data();
				Vec3 * supportPoints1 = g_collisionData.supportPoints1.data();

				if (specialCaseContactProcessing && (numUniqueVertices0 >= 3 && numUniqueVertices1 >= 3))
				{
					// Face-Face contact
					// TODO: reimplement most of the functions using indices rather than vertices moving where possible, because:
					//	a) it is faster, no need to actually perform copying
					//	b) some additional info beyond points could be required
					// NOTE: this is not possible for vertex-modifying algorithms, e.g. polygon clipping

					// Get convex wrapping of the set of points 0
					g_collisionData.convexPtIndices.resize(numUniqueVertices0);
					uint32_t numConvexHullVertices0 = 0;
					convexWrap2D(supportPoints0, numUniqueVertices0, -contactNormal, g_collisionData.convexPtIndices.data(), &numConvexHullVertices0);
					g_collisionData.convexPolygon0.resize(numConvexHullVertices0);
					Vec3 * convexPolygon0 = g_collisionData.convexPolygon0.data();
					for (size_t pi = 0; pi < numConvexHullVertices0; ++pi)
					{
						convexPolygon0[pi] = supportPoints0[g_collisionData.convexPtIndices[pi]];
					}

					// Computing the convex polygon 0 normals
					Vec3 refPolyNormal0 = Vec3C();
					for (uint32_t pi = 1; pi < numConvexHullVertices0; ++pi)
					{
						int pim1 = pi - 1;
						int pip1 = (pi + 1) % numConvexHullVertices0;
						refPolyNormal0 += (convexPolygon0[pim1]-convexPolygon0[pi]).cross(convexPolygon0[pip1]-convexPolygon0[pi]);
					}
					refPolyNormal0.normalize();
					// Normal 0 should point in direction -contactNormal
					if (refPolyNormal0.dot(contactNormal) > 0.0f)
					{
						// If it doesn't, flip normal, and reverse convex points
						refPolyNormal0 = -refPolyNormal0;
						for (uint32_t pi = 0; pi < numConvexHullVertices0 / 2; ++pi)
						{
							Vec3 temp = convexPolygon0[pi];
							convexPolygon0[pi] = convexPolygon0[numConvexHullVertices0 - pi - 1];
							convexPolygon0[numConvexHullVertices0 - pi - 1] = temp;
						}
					}

					// Get convex wrapping of the set of points 1
					g_collisionData.convexPtIndices.resize(numUniqueVertices1);
					uint32_t numConvexHullVertices1 = 0;
					convexWrap2D(supportPoints1, numUniqueVertices1, contactNormal, g_collisionData.convexPtIndices.data(), &numConvexHullVertices1);
					g_collisionData.convexPolygon1.resize(numConvexHullVertices1);
					Vec3 * convexPolygon1 = g_collisionData.convexPolygon1.data();
					for (size_t pi = 0; pi < numConvexHullVertices1; ++pi)
					{
						convexPolygon1[pi] = supportPoints1[g_collisionData.convexPtIndices[pi]];
					}

					// Computing the convex polygon 1 normals
					Vec3 refPolyNormal1 = Vec3C();
					for (uint32_t pi = 1; pi < numConvexHullVertices1; ++pi)
					{
						int pim1 = pi - 1;
						int pip1 = (pi + 1) % numConvexHullVertices1;
						refPolyNormal1 += (convexPolygon1[pim1]-convexPolygon1[pi]).cross(convexPolygon1[pip1]-convexPolygon1[pi]);
					}
					refPolyNormal1.normalize();
					// Normal 1 should point in direction contactNormal
					if (refPolyNormal1.dot(contactNormal) < 0.0f)
					{
						// If it doesn't, flip normal, and reverse convex points
						refPolyNormal1 = -refPolyNormal1;
						for (uint32_t pi = 0; pi < numConvexHullVertices1 / 2; ++pi)
						{
							Vec3 temp = convexPolygon1[pi];
							convexPolygon1[pi] = convexPolygon1[numConvexHullVertices1 - pi - 1];
							convexPolygon1[numConvexHullVertices1 - pi - 1] = temp;
						}
					}

					if (numConvexHullVertices0 < 2 || numConvexHullVertices1 < 2)
					{
						// Degenerate cases - resort to the previous contact gen scheme for now
						// TODO: process degenerate case when convex wrapping generates edge

						fallbackContact(contactPoint0, contactPoint1, -contactNormal);

						continue;
					}
					else
					{
						// Clip contact polygons
						// TODO: possibly flatten contact polygins, since they could be marginally non-flat

						// Clipping could produce more vertices than initially fed into
						//	so allocating buffer with arbitrary number (3) times more memory
						const size_t clipBufsSize = 3*m_max(numConvexHullVertices0, numConvexHullVertices1);
						g_collisionData.clipBuf0.resize(clipBufsSize);
						g_collisionData.clipBuf1.resize(clipBufsSize);

						g_collisionData.clipPlanes.resize(numContactVertices*4);

						g_collisionData.clippedPoly0.resize(clipBufsSize);
						uint32_t clippedVertsNum0;

						clipPolygons(
							convexPolygon0, numConvexHullVertices0,
							convexPolygon1, numConvexHullVertices1,
							refPolyNormal1,
							g_collisionData.clipBuf0.data(), g_collisionData.clipBuf1.data(), clipBufsSize,
							g_collisionData.clipPlanes.data(),
							g_collisionData.clippedPoly0.data(), &clippedVertsNum0
							);

						// Degenerate case - due to ill-oriented polygons, whole face could be clipped out,
						//	resort to the points available
						if (clippedVertsNum0 == 0)
						{
							fallbackContact(contactPoint0, contactPoint1, -contactNormal);

							continue;
						}

						Vec3 * clippedPoly0 = g_collisionData.clippedPoly0.data();

						// Throwing away vertices that are above the contact plane
						float contactPlaneD0 = refPolyNormal0.dot(contactPoint1);
						float contactPlaneD1 = refPolyNormal1.dot(contactPoint1);
						uint32_t cmSize = 0;
						g_collisionData.contactManifold.resize(clippedVertsNum0);
						Vec3 * contactManifold = g_collisionData.contactManifold.data();

						g_collisionData.contactDepths.resize(clippedVertsNum0);
						float * contactDepths = g_collisionData.contactDepths.data();

						for (uint32_t pi = 0; pi < clippedVertsNum0; ++pi)
						{
							float contactD = refPolyNormal1.dot(clippedPoly0[pi]) - contactPlaneD1;
							if (contactD < 0.0f)
							{
								contactManifold[cmSize] = clippedPoly0[pi];
								contactDepths[cmSize] = refPolyNormal0.dot(clippedPoly0[pi]) - contactPlaneD0;
								++cmSize;
							}
						}

						// No need to check for TriNode contacts, as they should all produce single support point at the moment
						// TODO: add tri node contacts here as well

						assert(!(
							((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate) ||
							((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate)
							));

						for (uint32_t ci = 0; ci < cmSize; ++ci)
						{
							Contact * contactJoint = (Contact *)g_contactJointPool.getElementMemory();
							new (contactJoint) Contact;
							contactJoint->init(contactERP, contactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeRot);
							contactJoint->setContactInfo(contactManifold[ci], contactManifold[ci] - refPolyNormal0*contactDepths[ci], refPolyNormal0);
							contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);

							g_dynamicJoints.push_back(contactJoint);
						}

#if 0
						// TODO: reimplement contact reduction using indices (similar to convex wrapping) rather than
						//	vertex shuffle - since here depth needs to be indexed along with contact points
						uint32_t numFinalContactPoints0 = 0;
						g_finalContactPoints0.resize(clipBufsSize);

						reduceContacts(contactManifold, cmSize, g_collisionData.finalContactPoints0.data(), &numFinalContactPoints0);

						Vec3 * finalContactPoints0 = g_collisionData.finalContactPoints0.data();
#endif
					}
				}
				else if (specialCaseContactProcessing && (numUniqueVertices0 == 1 && numUniqueVertices1 >= 3))
				{
					// Vertex-Face contact, vertex is supportPoints0[0]
					// Computing the face normal
					Vec3 refPolyNormal1 = computeFaceNormal(supportPoints1, numUniqueVertices1, contactNormal);
					acceptVerticesBelowFace(supportPoints1, numUniqueVertices1, refPolyNormal1, supportPoints0, 1, true);
				}
				else if (specialCaseContactProcessing && (numUniqueVertices0 >= 3 && numUniqueVertices1 == 1))
				{
					// Face-Vertex contact, vertex is supportPoints1[0]
					// Computing the face normal
					Vec3 refPolyNormal0 = computeFaceNormal(supportPoints0, numUniqueVertices0, -contactNormal);
					acceptVerticesBelowFace(supportPoints0, numUniqueVertices0, refPolyNormal0, supportPoints1, 1, false);
				}
				else if (specialCaseContactProcessing && (numUniqueVertices0 == 2 && numUniqueVertices1 >= 3))
				{
					// Edge-Face case
						
					// 0. Get convex wrapping of the incoming face for proper edge traversing
					g_collisionData.convexPtIndices.resize(numUniqueVertices1);
					uint32_t numConvexHullVertices1 = 0;
					convexWrap2D(supportPoints1, numUniqueVertices1, contactNormal, g_collisionData.convexPtIndices.data(), &numConvexHullVertices1);
					g_collisionData.convexPolygon1.resize(numConvexHullVertices1);
					Vec3 * convexPolygon1 = g_collisionData.convexPolygon1.data();
					for (size_t pi = 0; pi < numConvexHullVertices1; ++pi)
					{
						convexPolygon1[pi] = supportPoints1[g_collisionData.convexPtIndices[pi]];
					}

					// 1. Calculate face normal
					Vec3 refPolyNormal1 = computeFaceNormal(convexPolygon1, numConvexHullVertices1, contactNormal);

					// 2. Clip the edge against prism formed by the face polygon, to avoid
					//	cliff hanging artifacts
					clipEdgeAgainstFace(supportPoints0, convexPolygon1, numConvexHullVertices1, refPolyNormal1);

					// 3. Accept only the edge vertices that are below the contact plane
					acceptVerticesBelowFace(supportPoints1, numUniqueVertices1, refPolyNormal1, supportPoints0, 2, true);
				}
				else if (specialCaseContactProcessing && (numUniqueVertices0 >= 3 && numUniqueVertices1 == 2))
				{
					// Face-Edge case

					// 0. Get convex wrapping of the incoming face for proper edge traversing
					g_collisionData.convexPtIndices.resize(numUniqueVertices0);
					uint32_t numConvexHullVertices0 = 0;
					convexWrap2D(supportPoints0, numUniqueVertices0, -contactNormal, g_collisionData.convexPtIndices.data(), &numConvexHullVertices0);
					g_collisionData.convexPolygon0.resize(numConvexHullVertices0);
					Vec3 * convexPolygon0 = g_collisionData.convexPolygon0.data();
					for (size_t pi = 0; pi < numConvexHullVertices0; ++pi)
					{
						convexPolygon0[pi] = supportPoints0[g_collisionData.convexPtIndices[pi]];
					}

					// 1. Calculate face normal
					Vec3 refPolyNormal0 = computeFaceNormal(convexPolygon0, numConvexHullVertices0, -contactNormal);

					// 2. Clip the edge against prism formed by the face polygon, to avoid
					//	cliff hanging artifacts
					clipEdgeAgainstFace(supportPoints1, convexPolygon0, numConvexHullVertices0, refPolyNormal0);

					// 3. Accept only the edge vertices that are below the contact plane
					acceptVerticesBelowFace(supportPoints0, numUniqueVertices0, refPolyNormal0, supportPoints1, 2, false);
				}
				else if (specialCaseContactProcessing && (numUniqueVertices0 == 2 && numUniqueVertices1 == 2))
				{
					// Edge-edge case
					Vec3 edge0 = supportPoints0[1] - supportPoints0[0];
					Vec3 edge1 = supportPoints1[1] - supportPoints1[0];

					// At the intersection, vector between closest points is orthogonal to both
					//	edge vectors:
					//
					//	dist dot edge0 = dist dot edge1 = 0
					//	dist = p0 + edge0*t0 - (p1 + edge1*t1)
					//
					// This results in a system of linear equations:
					//	/ edge0 dot (p0 + edge0*t0 - (p1 + edge1*t1)) = 0
					//	\ edge1 dot (p0 + edge0*t0 - (p1 + edge1*t1)) = 0
					//
					// The coordinates could be inferred from the resulting system of equations.
					// One important note, distance between line segments cannot be calculated
					//	simply as distance between two lines, because clamp of one edge affects
					//	the closest point on another edge (imagine two intersecting lines, and
					//	then chop one line prior to the intersection point - closest point will
					//	now be different from the original intersection point on both lines).

					Vec3 diff = supportPoints0[0] - supportPoints1[0];

					float e0_e0 = edge0.dot(edge0);
					float e1_e1 = edge1.dot(edge1);
					float e1_diff = edge1.dot(diff);

					float e0_diff = edge0.dot(diff);

					float e0_e1 = edge0.dot(edge1);

					float divider = e0_e0*e1_e1 - e0_e1*e0_e1;

					float t0, t1;
					if (divider != 0.0f)
					{
						// This solves the mentioned linear system, 
						t0 = (e0_e1*e1_diff - e0_diff*e1_e1) / divider;
						t0 = clamp(t0, 0.0f, 1.0f);
					}
					else
					{
						// Divider is 0 => parallel lines, picking starting edge point.
						// In case of parallel lines actually, Edge-Edge case could generate
						//	two contact points instead of one for increased stability.
						t0 = 0.0f;
					}

					t1 = (t0*e0_e1 + e1_diff) / e1_e1;
						
					// Clamping of the second intersection point is a little trickier - we need
					//	to adjust the first intersection point as we clamp
					if (t1 < 0.0f)
					{
						t1 = 0.0f;
						t0 = -e0_diff / e0_e0;
						t0 = clamp(t0, 0.0f, 1.0f);
					}
					else if (t1 > 1.0f)
					{
						t1 = 1.0f;
						t0 = (e0_e1 - e0_diff) / e0_e0;
						t0 = clamp(t0, 0.0f, 1.0f);
					}

					Vec3 distP0 = supportPoints0[0] + edge0 * t0, distP1 = supportPoints1[0] + edge1 * t1;
					Vec3 normal = (distP0 - distP1).getNormalized();

					// No need to check for TriNode contacs here, as they produce single support point at the moment
					// TODO: add tri node contact here as well

					assert(!(
						((uint32_t)curCandidate0->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate) ||
						((uint32_t)curCandidate1->bitFlags & (uint32_t)CollisionCandidate::BitFlags::eTriNodeCandidate)
						));

					Contact * contactJoint = (Contact *)g_contactJointPool.getElementMemory();
					new (contactJoint) Contact;
					contactJoint->init(contactERP, contactCFM, curCandidate0->nodeLin, curCandidate0->nodeRot, curCandidate1->nodeLin, curCandidate1->nodeRot);
					contactJoint->setContactInfo(distP0, distP1, normal);
					contactJoint->setLocalMassScalingParams(g_contactMassScaleParams.isEnabled, g_contactMassScaleParams.maxMassScale, gravityVec);
					g_dynamicJoints.push_back(contactJoint);
				}
				else
				{
					// If not handled by special case methods - resort to the previous contact gen for now

					fallbackContact(contactPoint0, contactPoint1, -contactNormal);

					continue;
				}
			}
		}
	}

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.cd_NP_contactGen = perfTimer.time();
#endif
}

void defaultCleanupCollisionDetection()
{
	// Collision Detection stage cleanup function
	// At the moment the only thing it does - puts the CD candidate memory back into pool

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
#endif

	CollisionCandidate ** ptr_g_collisionCandidates = g_collisionCandidates.data();
	for (size_t ic = 0, icEnd = g_collisionCandidates.size(); ic < icEnd; ++ic)
	{
		CollisionCandidate * curCandidate = ptr_g_collisionCandidates[ic];
		curCandidate->~CollisionCandidate();
		g_collisionCandidatePool.putElementMemory(curCandidate);
	}

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.bp_candidates_store = perfTimer.time();
#endif
}

void defaultContactCacheMatch()
{
	// This function searches matching contact cache record, so that contact joint could
	//	be warm-started and also perform friction anchoring. Performs either a simple
	//	binary search in contact cache storage array sorted by the node0 idx (linear node
	//	of body0), or hashmap offset lookup

	// TODO:
	//	- see the list in complementary defaultContactCacheMatch function

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
#endif

	// Try to find matching contact cache
	for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
	{
		if (g_dynamicJoints[iJoint]->getType() == JointBase::Type::eContact)
		{
			Contact * curContact = static_cast<Contact *>(g_dynamicJoints[iJoint]);
			NodeTranslational * contactBody0L = curContact->getBody0L();

#if (CONTACT_CACHE_ACCEL_HASHMAP == 1)
			auto cacheAccelIndexIt = g_contactJointCacheSearchAccelIndex.find(contactBody0L->m_idxPrev);
			if (cacheAccelIndexIt == g_contactJointCacheSearchAccelIndex.end())
				continue;

			uint32_t startingIndex = cacheAccelIndexIt->second;
#else
			auto binSearchFirstEntry = [](uint32_t * input, size_t numElements, uint32_t val) -> uint32_t
			{
				const uint32_t notFound = (uint32_t)-1;

				if (input == nullptr || numElements == 0)
					return notFound;

				uint32_t firstEntryIdx = notFound;

				int low = 0;
				int high = (int)numElements - 1;
				while (low <= high)
				{
					int middle = low + ((high - low) >> 1);

					if (input[middle] < val)
					{
						low = middle + 1;
					}
					else
					{
						if (input[middle] == val)
						{
							firstEntryIdx = (uint32_t)middle;
						}

						high = middle - 1;
					}
				}

				return firstEntryIdx;
			};

			uint32_t startingIndex = binSearchFirstEntry(g_contactJointCacheBody0LIdx.data(), g_contactJointCacheBody0LIdx.size(), contactBody0L->m_idxPrev);
			if (startingIndex == (uint32_t)-1)
				continue;
#endif

			for (size_t cci = startingIndex, cciEnd = g_contactJointCacheSearchAccel.size(); cci < cciEnd; ++cci)
			{
				uint32_t sortedIndex = g_contactJointCacheSearchAccel[cci];
				ContactJointCache * curContactCache = g_contactJointCacheStorage[sortedIndex];

				// The index is sorted, so if body0L doesn't match anymore => contact not found
				if (curContactCache->body0L != contactBody0L)
				{
					break;
				}

				if (curContactCache->isUsed)
					continue;

				if (curContactCache->body0R == curContact->getBody0R() &&
					curContactCache->body1L == curContact->getBody1L() &&
					curContactCache->body1R == curContact->getBody1R())
				{
					math::Vec3 cp0_b0o, cp1_b0o, cp0_b1o, cp1_b1o;
					curContact->getContactInfoOrigin(&cp0_b0o, &cp1_b0o, &cp0_b1o, &cp1_b1o);
					const float contactCacheMatchEps = 0.005f;
					const float contactCacheMatchEpsSq = contactCacheMatchEps*contactCacheMatchEps;
					if ((curContactCache->cp0_b0o - cp0_b0o).sqLen() < contactCacheMatchEpsSq &&
						(curContactCache->cp1_b0o - cp1_b0o).sqLen() < contactCacheMatchEpsSq &&
						(curContactCache->cp0_b1o - cp0_b1o).sqLen() < contactCacheMatchEpsSq &&
						(curContactCache->cp1_b1o - cp1_b1o).sqLen() < contactCacheMatchEpsSq)
					{
						curContactCache->isUsed = true;
						curContact->setCache(curContactCache);
						curContact->setLambda(0, curContactCache->lambdas[0]);
						curContact->setLambda(1, curContactCache->lambdas[1]);
						curContact->setLambda(2, curContactCache->lambdas[2]);
						break;
					}
				}
			}
		}
	}

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.contactCache_match = perfTimer.time();
#endif
}

void defaultContactCacheStore()
{
	// This is a complementary fruinction to the defaultContactCacheMatch function
	// Stores unused contact cache records back to the pool, and prepares arrays for
	//	the contact cache search acceleration (sorted array of body0 linear node
	//	indices), and, if required, hashmap that stores offset of the first entry
	//	of the body0 linear node index into the sorted array.

	// TODO:
	//	- implement logic that uses body1 linear node index in case body0 is static,
	//		currently if body0 is static floor - it will have a broad range of entries
	//		to lookup, causing slowdowns;
	//	- robust search body detection - in addition to being dynamic, out of two
	//		dynamic bodies it should calculate one in a determenistic fashion, to
	//		be invariant to bodies reshuffling in the candidates array;
	//	- incorporate trinode contacts into the caching system;

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	windows::Timer perfTimer;
	perfTimer.start();
	double perfTimingsPrevTime = perfTimer.time();

#	define PHYS_PERF_TIMING_SPOT(timing_var) \
			timing_var = perfTimer.time() - perfTimingsPrevTime; \
			perfTimingsPrevTime = perfTimer.time();
#else
#	define PHYS_PERF_TIMING_SPOT(timing_var)
#endif

	// Put unused cache items back into contact cache pool
	auto contactCacheItEnd = g_contactJointCacheStorage.end();
	for (auto contactCacheIt = g_contactJointCacheStorage.begin(); contactCacheIt != contactCacheItEnd; ++contactCacheIt)
	{
		ContactJointCache * curContactCache = *contactCacheIt;
		if (!curContactCache->isUsed)
		{
			// POD
			//curContactCache->~ContactJointCache();
			g_contactJointCachePool.putElementMemory(curContactCache);
		}
	}

	g_contactJointCacheStorage.resize(0);
	g_contactJointCacheStorage.reserve(g_dynamicJoints.size());
	g_contactJointCacheBody0LIdx.resize(0);
	g_contactJointCacheBody0LIdx.reserve(g_dynamicJoints.size());
	for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
	{
		if (g_dynamicJoints[iJoint]->getType() == JointBase::Type::eContact)
		{
			Contact * curContact = static_cast<Contact *>(g_dynamicJoints[iJoint]);

			ContactJointCache * curContactCache = curContact->getCache();
			
			// If cache wasn't found for this joint, request new record from the pool
			if (curContactCache == nullptr)
			{
				curContactCache = g_contactJointCachePool.getElementMemory();
				// POD
				//new (curContactCache) ContactJointCache;
				curContactCache->body0L = curContact->getBody0L();
				curContactCache->body0R = curContact->getBody0R();
				curContactCache->body1L = curContact->getBody1L();
				curContactCache->body1R = curContact->getBody1R();
				curContact->getContactInfoOrigin(
					&curContactCache->cp0_b0o,
					&curContactCache->cp1_b0o,
					&curContactCache->cp0_b1o,
					&curContactCache->cp1_b1o
					);
			}
			// If joint had associated cache, it will update the contact info if required
			if (0)
			{
				curContact->getContactInfoOrigin(
					&curContactCache->cp0_b0o,
					&curContactCache->cp1_b0o,
					&curContactCache->cp0_b1o,
					&curContactCache->cp1_b1o
					);

			}
			// Fill contact cache info
			curContactCache->isUsed = false;
			curContactCache->lambdas[0] = curContact->getLambda(0);
			curContactCache->lambdas[1] = curContact->getLambda(1);
			curContactCache->lambdas[2] = curContact->getLambda(2);

			g_contactJointCacheStorage.push_back(curContactCache);
			g_contactJointCacheBody0LIdx.push_back(curContactCache->body0L->m_idx);

			curContact->~Contact();
			g_contactJointPool.putElementMemory(curContact);
		}
		else if (g_dynamicJoints[iJoint]->getType() == JointBase::Type::eTriNodeContact)
		{
			TriNodeContact * curContact = static_cast<TriNodeContact *>(g_dynamicJoints[iJoint]);
			curContact->~TriNodeContact();
			g_triNodeContactJointPool.putElementMemory(curContact);
		}
	}
	g_dynamicJoints.resize(0);

	PHYS_PERF_TIMING_SPOT(g_perfTimings.contactCache_update);

	// Sort the joint cache body indices, to accelerate the search on the next frame
	uint32_t * jointCacheBodyIndices = g_contactJointCacheBody0LIdx.data();
	SortRadix & radixSort = SortRadix::getInstance();
	radixSort.sort(g_contactJointCacheBody0LIdx.data(), g_contactJointCacheBody0LIdx.size());
	g_contactJointCacheSearchAccel.resize(radixSort.getLastResultSize());
	uint32_t * storedIndices = g_contactJointCacheSearchAccel.data();
	const uint32_t * sortedIndices = radixSort.getLastResultIndices();
	memcpy(storedIndices, sortedIndices, radixSort.getLastResultSize() * radixSort.getIndexElementSize());

#if (CONTACT_CACHE_ACCEL_HASHMAP == 1)
	// Build a map of body handle -> index in sorted array
	// Binary search could be used as well on the next frame
	g_contactJointCacheSearchAccelIndex.clear();
	if (radixSort.getLastResultSize() > 0)
	{
		uint32_t prevBodyIndex = jointCacheBodyIndices[storedIndices[0]];
		g_contactJointCacheSearchAccelIndex.insert(std::make_pair(prevBodyIndex, 0));
		for (size_t acci = 0, acciEnd = g_contactJointCacheSearchAccel.size(); acci < acciEnd; ++acci)
		{
			uint32_t curBodyIndex = jointCacheBodyIndices[storedIndices[acci]];
			if (curBodyIndex != prevBodyIndex)
			{
				g_contactJointCacheSearchAccelIndex.insert(std::make_pair(curBodyIndex, (uint32_t)acci));
				prevBodyIndex = curBodyIndex;
			}
		}
	}
#endif

	PHYS_PERF_TIMING_SPOT(g_perfTimings.contactCache_accel);

#undef PHYS_PERF_TIMING_SPOT
}

void advanceDynamics(SolverBase * solver, float dt, const math::Vec3 & gravityVec)
{
	using namespace math;

	// Main dynamics function, takes state of the system (joints and nodes), and calculates
	//	new positions/velocities of the system so that it advances on a timestep dt, and
	//	the joints are not violated very much (the solution is approximate).

	// Performs two velocity solves, one is frictionless to estimate the normal forces acting
	//	in the system, and the next one is full solve with friction, where friction parameters
	//	are set accordingly to the estimated normal force.
	
	// High-level logic:
	//	- preintegrate (calculate total impulse acting on the nodes v_tot = v + dt*M^-1*Fe);
	//	- for frictionless solve:
	//		* calculate total amount of constraint rows to allocate
	//		* fill in constraint data (Jacobians, right hand side vectors, etc.)
	//		* solve the MLCP using solver provided, of the form
	//			dt*(J*M^-1*J^T + D)*lambda = 1/dt*c_t(t) - J*(v + dt*M^-1*Fe)
	//			subject to: lambda_lo <= lambda <= lambda_hi
	//		* fetch constraint-space force values and store them into the joints
	//	- for full solve steps are same, but the constraint data filling procedure now
	//		has constraint force values, which could be used to calculate fritcion limits
	//	- integrate (calculate final velocities and positions, using symplectic Euler,
	//		not dropping out the gyroscopic term, i.e. including w x Iw)

	// TODO:
	//	- add splitting to islands (will help with CG-based solver dependence too)
	//	- add sleeping
	//	- add pseudo-velocities

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	double perfTimingsPrevTime;
	windows::Timer perfTimer;
	perfTimer.start();
	perfTimingsPrevTime = perfTimer.time();

#	define PHYS_PERF_TIMING_SPOT(timing_var) \
			timing_var = perfTimer.time() - perfTimingsPrevTime; \
			perfTimingsPrevTime = perfTimer.time();
#else
#	define PHYS_PERF_TIMING_SPOT(timing_var)
#endif

	solver->setNumNodes((uint32_t)g_nodes.size());
	updateNodesPreIntegrate(solver, dt, gravityVec);
	PHYS_PERF_TIMING_SPOT(g_perfTimings.pre_integrate);

	{
		uint32_t numRowsTotal = 0;
		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_joints[iJoint]->onStepStart(dt);
			numRowsTotal += g_joints[iJoint]->getNumRowsVel();
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_dynamicJoints[iJoint]->onStepStart(dt);
			numRowsTotal += g_dynamicJoints[iJoint]->getNumRowsVel();
		}
		solver->setNumRows(numRowsTotal);

		PHYS_PERF_TIMING_SPOT(g_perfTimings.vel_numRows_precompute);

		solver->m_numJoints = 0;
		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_joints[iJoint]->updateCopyVel(dt, *solver);
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_dynamicJoints[iJoint]->updateCopyVel(dt, *solver);
		}

		PHYS_PERF_TIMING_SPOT(g_perfTimings.vel_updateConstr);

		// Use less iterations for the frictionless solve
		// This solve step determines the normal force, this estimation
		//	will be used in the subsequent solve to set friction limits
		uint32_t maxIter = solver->getMaxIterations();
		solver->setMaxIterations(maxIter / 2);
		solver->solve(dt);
		solver->setMaxIterations(maxIter);

		PHYS_PERF_TIMING_SPOT(g_perfTimings.vel_solve);

		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_joints[iJoint]->fetchLambdasVel(*solver);
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_dynamicJoints[iJoint]->fetchLambdasVel(*solver);
		}

		PHYS_PERF_TIMING_SPOT(g_perfTimings.vel_fetchLambdas);
	}
	{
		uint32_t numRowsTotal = 0;
		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			numRowsTotal += g_joints[iJoint]->getNumRowsVelFriction();
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			numRowsTotal += g_dynamicJoints[iJoint]->getNumRowsVelFriction();
		}
		solver->setNumRows(numRowsTotal);

		PHYS_PERF_TIMING_SPOT(g_perfTimings.velFric_numRows);

		solver->m_numJoints = 0;
		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_joints[iJoint]->updateCopyVelFriction(dt, *solver);
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_dynamicJoints[iJoint]->updateCopyVelFriction(dt, *solver);
		}

		PHYS_PERF_TIMING_SPOT(g_perfTimings.velFric_updateConstr);

		solver->solve(dt);

		PHYS_PERF_TIMING_SPOT(g_perfTimings.velFric_solve);

		for (size_t iJoint = 0, iJointEnd = g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_joints[iJoint]->fetchLambdasVelFriction(*solver);
		}
		for (size_t iJoint = 0, iJointEnd = g_dynamicJoints.size(); iJoint < iJointEnd; ++iJoint)
		{
			g_dynamicJoints[iJoint]->fetchLambdasVelFriction(*solver);
		}

		PHYS_PERF_TIMING_SPOT(g_perfTimings.velFric_fetchLambdas);
	}
	readbackNodesIntegrate(solver, dt);
	PHYS_PERF_TIMING_SPOT(g_perfTimings.integrate);

#undef PHYS_PERF_TIMING_SPOT
}

math::Vec3 g_gravity = math::Vec3C(0.0f, -9.81f, 0.0f);
void updateSim(SolverBase * solver, float dt)
{
	if (dt <= 0.0f)
		return;

	// Resets rendering primitives and perf timings
	simPrepareStep();

	// Collision detection stages
	defaultBuildCollisionCandidatesList();
	defaultBroadphase();

	g_dynamicJoints.resize(0);
	g_dynamicJoints.reserve(g_collisionPairs.size() * 2);

	defaultMPRNarrowphase(g_gravity);
	defaultCleanupCollisionDetection();
	// End of collision detection stages

	defaultContactCacheMatch();
	advanceDynamics(solver, dt, g_gravity);
	defaultContactCacheStore();

#if (DBG_ENABLE_PERF_TIMINGS == 1)
	g_perfTimings.totalFrameTime = g_perfTimings.getTotalTime();

#if 0
	double perfDiv = 1.0f / g_perfTimings.totalFrameTime;

	printf("Physics timings:\n");
	printf("  pre-integrate: \t\t%3.7f\n", g_perfTimings.pre_integrate * perfDiv);
	printf("  bp_candidates: \t\t%3.7f\n", g_perfTimings.bp_candidates * perfDiv);
	printf("  bp_buildPairsList: \t\t%3.7f\n", g_perfTimings.bp_buildPairsList * perfDiv);
	printf("  cd_NP_contactGen: \t\t%3.7f\n", g_perfTimings.cd_NP_contactGen * perfDiv);
	printf("  contactCache_match: \t\t%3.7f\n", g_perfTimings.contactCache_match * perfDiv);
	printf("  vel_numRows_precompute: \t%3.7f\n", g_perfTimings.vel_numRows_precompute * perfDiv);
	printf("  vel_updateConstr: \t\t%3.7f\n", g_perfTimings.vel_updateConstr * perfDiv);
	printf("  vel_solve: \t\t\t%3.7f\n", g_perfTimings.vel_solve * perfDiv);
	printf("  vel_fetchLambdas: \t\t%3.7f\n", g_perfTimings.vel_fetchLambdas * perfDiv);
	printf("  velFric_numRows: \t\t%3.7f\n", g_perfTimings.velFric_numRows * perfDiv);
	printf("  velFric_updateConstr: \t%3.7f\n", g_perfTimings.velFric_updateConstr * perfDiv);
	printf("  velFric_solve: \t\t%3.7f\n", g_perfTimings.velFric_solve * perfDiv);
	printf("  velFric_fetchLambdas: \t%3.7f\n", g_perfTimings.velFric_fetchLambdas * perfDiv);
	printf("  integrate: \t\t\t%3.7f\n", g_perfTimings.integrate * perfDiv);
	printf("  contactCache_update: \t\t%3.7f\n", g_perfTimings.contactCache_update * perfDiv);
	printf("  contactCache_accel: \t\t%3.7f\n", g_perfTimings.contactCache_accel * perfDiv);
	printf("  totalFrameTime: \t\t%3.7f ms\n", g_perfTimings.totalFrameTime);
#endif
#endif
}

}
