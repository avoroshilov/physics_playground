#pragma once

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void createStaticPlaypen(const math::Vec3 & pos, const math::Vec3 & size, bool walled = false, float wallHeight = 5.0f, float wallThickness = 0.25f)
{
	using namespace math;

	physics::NodeTranslational * bodyL = physics::addTranslationalNode(0.0f, pos, Vec3C());
	physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().zero(), Quat().id(), Vec3C());

	physics::RigidBody * body = physics::createRigidBody();
	body->m_bodyL = bodyL;
	body->m_bodyR = bodyR;

	ConvexBox * convexBox = new ConvexBox;
	physics::trackConvexHull(convexBox);
	convexBox->m_halfExtents = size;
	convexBox->m_origin = Vec3C();
	convexBox->m_rotation.identity();

	physics::GeometryConvex * convex = new physics::GeometryConvex;
	physics::trackGeometryShape(convex);
	convex->m_convexHullShape = convexBox;
	convex->m_density = 0.0f;
	convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
	convex->m_rotation.identity();
	body->m_geometryShapes.push_back(convex);

	if (walled)
	{
		convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = Vec3C(size.x, wallHeight, wallThickness);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_convexHullShape = convexBox;
		convex->m_density = 0.0f;
		convex->m_origin = Vec3C(0.0f, wallHeight, size.z);
		convex->m_rotation.identity();
		body->m_geometryShapes.push_back(convex);

		////////////////////////////////////////////////////////////
		convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = Vec3C(size.x, wallHeight, wallThickness);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_convexHullShape = convexBox;
		convex->m_density = 0.0f;
		convex->m_origin = Vec3C(0.0f, wallHeight, -size.z);
		convex->m_rotation.identity();
		body->m_geometryShapes.push_back(convex);

		////////////////////////////////////////////////////////////
		convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = Vec3C(wallThickness, wallHeight, size.z);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_convexHullShape = convexBox;
		convex->m_density = 0.0f;
		convex->m_origin = Vec3C(size.x, wallHeight, 0.0f);
		convex->m_rotation.identity();
		body->m_geometryShapes.push_back(convex);

		////////////////////////////////////////////////////////////
		convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = Vec3C(wallThickness, wallHeight, size.z);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_convexHullShape = convexBox;
		convex->m_density = 0.0f;
		convex->m_origin = Vec3C(-size.x, wallHeight, 0.0f);
		convex->m_rotation.identity();
		body->m_geometryShapes.push_back(convex);
	}

	body->prepareGeometry();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void sceneSample(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(25.0f, 0.5f, 25.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.25f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);
}

inline void sceneBoxContact(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	// Sliding sub-scene
	if (1)
	{
		{
			physics::NodeTranslational * bodyL = physics::addTranslationalNode(0.0f, Vec3C(0.0f, -2.0f, 0.0f), Vec3C());
			Quat identityQuat = Quat::fromAxisAngle(Vec3C(0.0f, 0.0f, 1.0f), PI / 8.0f);
			physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().zero(), identityQuat, Vec3C());

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			Vec3 staticSize = Vec3C(15.0f, 0.5f, 10.0f);

			ConvexBox * convexBox = new ConvexBox;
			physics::trackConvexHull(convexBox);
			convexBox->m_halfExtents = staticSize;
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			physics::GeometryConvex * convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_convexHullShape = convexBox;
			convex->m_density = 0.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			body->m_geometryShapes.push_back(convex);
		}

		int cdStartNumNodes = (int)physics::g_nodes.size();
		const int numObjectsSliding = 5;
		for (int i = 0; i < numObjectsSliding; ++i)
		{
			physics::NodeTranslational * bodyL = physics::addTranslationalNode(1.0f / mass, Vec3C(2, 0.0f, (i / (float)(numObjectsSliding - 1) - 0.5f) * 15.0f), Vec3C());
			Quat rotationQuat = Quat::fromAxisAngle(Vec3C(0.0f, 0.0f, 1.0f), PI / 8.0f);
			physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, rotationQuat, Vec3C());

			bodyL->m_frictionCoeff = (i / (float)(numObjectsSliding - 1)) * 0.1f;

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			ConvexBox * convexBox = new ConvexBox;
			physics::trackConvexHull(convexBox);
			convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			physics::GeometryConvex * convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexBox;
			body->m_geometryShapes.push_back(convex);

			body->prepareGeometry();
		}
	}

	// Restitution sub-scene
	if (1)
	{
		const Vec3 bounceGroundPos = Vec3C(0.0f, 0.0f, -20.0f);
		const Vec3 bounceGroundSize = Vec3C(10.0f, 0.5f, 5.0f);
		{
			physics::NodeTranslational * bodyL = physics::addTranslationalNode(0.0f, bounceGroundPos, Vec3C());
			physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().zero(), Quat().id(), Vec3C());

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			bodyL->m_restitutionCoeff = 1.0f;

			ConvexBox * convexBox = new ConvexBox;
			physics::trackConvexHull(convexBox);
			convexBox->m_halfExtents = bounceGroundSize;
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			physics::GeometryConvex * convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_convexHullShape = convexBox;
			convex->m_density = 0.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			body->m_geometryShapes.push_back(convex);
		}

		// Eventually, in this test, the body with restitution coeff ~1.0 will start jumping around instead of keep jumping upwards.
		//	This is due to the fact that rotation error will be accumulated, *and* also the penetration will be very shallow.
		//	Combination of these two factors will lead to only single contact point being completely beneath the opposite contact
		//	feature plane, causing only one contact to be registered out of 4, inducing rotation.
		// This is expected behavior, there are ways to fix it by improving the contact detection, or modifying it to handle such
		//	edge cases, but it is not handle currently in this engine.
		const int numObjectsBouncing = 5;
		for (int i = 0; i < numObjectsBouncing; ++i)
		{
			physics::NodeTranslational * bodyL = physics::addTranslationalNode(1.0f / mass, Vec3C(bounceGroundPos.x + (i / (float)(numObjectsBouncing - 1) - 0.5f) * bounceGroundSize.x, 2.0f, bounceGroundPos.z), Vec3C());
			physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

			bodyL->m_restitutionCoeff = (i / (float)(numObjectsBouncing - 1)) * 1.0f;
			bodyL->m_dampingMul = 1.0f;
			bodyL->m_dampingSub = 0.0f;
			bodyR->m_dampingMul = 1.0f;
			bodyR->m_dampingSub = 0.0f;

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			ConvexBox * convexBox = new ConvexBox;
			physics::trackConvexHull(convexBox);
			convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			physics::GeometryConvex * convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexBox;
			body->m_geometryShapes.push_back(convex);

			body->prepareGeometry();
		}
	}
}

inline void sceneSimpleBallsocket(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	int NumLinks = 10;

	// Add rigid bodies
	for (int i = 0; i < NumLinks + 1; ++i)
	{
		physics::addTranslationalNode((i == 0 || i == NumLinks+1)?0.0f:(1.0f / mass), Vec3C(25 - i*2.0f, 10.3f, -0.3f), Vec3C());
		physics::addRotationalNode((i == 0 || i == NumLinks+1)?Mat33().zero():inertiaMatrix, Quat().id(), Vec3C());
	}

#if 1
	for (size_t iNode = 1, iNodeEnd = physics::g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational)
			continue;
		physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);

		physics::PlaneConstraint * newPlaneConstraint = new physics::PlaneConstraint;
		newPlaneConstraint->init(0.1f, 0.001f, math::Vec3C(), math::Vec3C(0.0f, 1.0f, 0.0f), curNodeTrn, 0.01f);

		physics::addTrackJoint(newPlaneConstraint);
	}
#endif

	for (int i = 0; i < NumLinks; ++i)
	{
		physics::BallSocket * newBallSocket = new physics::BallSocket;

		Vec3 anchor = 0.5f * (static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i) * 2])->m_pos + 
							static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i+1) * 2])->m_pos);
		newBallSocket->init(
			0.5f, 0.01f, anchor,
			static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i)*2]),
			static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i)*2+1]),
			static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i+1)*2]),
			static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i+1)*2+1]));

		physics::addTrackJoint(newBallSocket);
	}
}

inline void sceneBallSocketCoMTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	int NumLinks = 10;

	// Add rigid bodies
	for (int i = 0; i < NumLinks + 1; ++i)
	{
		physics::NodeTranslational * bodyL = physics::addTranslationalNode((i == 0 || i == NumLinks+1)?0.0f:(1.0f / mass), Vec3C(25 - i*4.0f, 10.3f, -0.3f), Vec3C());
		physics::NodeRotational * bodyR = physics::addRotationalNode((i == 0 || i == NumLinks+1)?Mat33().zero():inertiaMatrix, Quat().id(), Vec3C());

		physics::RigidBody * body = physics::createRigidBody();
		body->m_bodyL = bodyL;
		body->m_bodyR = bodyR;

		ConvexBox * convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = (i == 0 || i == NumLinks+1)?Vec3C(0.3f, 0.3f, 0.3f):Vec3C(0.5f, 0.5f, 0.5f);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		physics::GeometryConvex * convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_density = (i == 0 || i == NumLinks+1)?0.0f:1.0f;

		if (i == 1)
			convex->m_density = 100.0f;

		convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
		convex->m_rotation.identity();
		convex->m_convexHullShape = convexBox;
		body->m_geometryShapes.push_back(convex);

		body->prepareGeometry();

		//if (i == 1)
			//body->adjustCoM(Vec3C(0.0f, 1.0f, 0.0f));
	}

	for (int i = 0; i < NumLinks; ++i)
	{
		physics::NodeTranslational * body0L = static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i) * 2]);
		physics::NodeRotational * body0R = static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i) * 2 + 1]);
		physics::NodeTranslational * body1L = static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i+1) * 2]);
		physics::NodeRotational * body1R = static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i+1) * 2 + 1]);

		Mat33 body0RotMatrix;
		body0RotMatrix = math::Quat::toMatrix33(body0R->m_rot);

		Mat33 body1RotMatrix;
		body1RotMatrix = math::Quat::toMatrix33(body1R->m_rot);

		Vec3 body0OriginPos = body0L->m_pos + body0RotMatrix * (-body0R->m_com);
		Vec3 body1OriginPos = body1L->m_pos + body1RotMatrix * (-body1R->m_com);

		physics::BallSocket * newBallSocket = new physics::BallSocket;

		Vec3 anchor = (body0OriginPos + body1OriginPos) * 0.5f;
		newBallSocket->init(
			0.25f, 0.05f, anchor,
			body0L,
			body0R,
			body1L,
			body1R
			);
		
		physics::addTrackJoint(newBallSocket);
	}

	{
		physics::RigidBody * adjBody = physics::g_rigidBodies[1];
		adjBody->adjustCoM(Vec3C(0.0f, 1.0f, 0.0f));
	}
}

inline void sceneJointsTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(5, 0.5f, -1.3f);
	const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
	const bool staticWalls = false;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.25f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	const bool applyLocalMassScale = false;
	const float localMassScaleRatio = 1.2f;

	if (1)
	{
		uint32_t bodyStartId = (uint32_t)physics::g_rigidBodies.size();

		const uint32_t numLinks = 10;
		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::NodeTranslational * body0L;
			body0L = physics::addTranslationalNode(1.0f / mass, staticPos + Vec3C(1 + bi * 3.0f, 6.0f, -10.0f), Vec3C());
			physics::NodeRotational * body0R = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

			physics::RigidBody * body0 = physics::createRigidBody();
			body0->m_bodyL = body0L;
			body0->m_bodyR = body0R;

			{
				ConvexBox * convexBox = new ConvexBox;
				physics::trackConvexHull(convexBox);
				convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
				convexBox->m_origin = Vec3C();
				convexBox->m_rotation.identity();

				physics::GeometryConvex * convex = new physics::GeometryConvex;
				physics::trackGeometryShape(convex);
				convex->m_density = 0.1f;
				convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
				convex->m_rotation.identity();
				convex->m_convexHullShape = convexBox;
				body0->m_geometryShapes.push_back(convex);

				body0->prepareGeometry();
			}
		}

		const float angLo = -PI / 40.0f;
		const float angHi =  PI / 40.0f;
		//const float angLo = -PI / 4.0f;
		//const float angHi =  PI / 4.0f;

		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::RigidBody * body = physics::g_rigidBodies[bodyStartId+bi];
			physics::RigidBody * bodyPrev = nullptr;
			if (bi >= 1)
				bodyPrev = physics::g_rigidBodies[bodyStartId+bi-1];

			if (bodyPrev == nullptr)
			{
				physics::BallSocket * newBallSocket = new physics::BallSocket;
				Vec3 anchor = body->m_bodyL->m_pos + Vec3C(-1.5f, 0.0f, 0.0f);
				newBallSocket->init(
					0.5f, 0.01f, anchor,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);

				physics::addTrackJoint(newBallSocket);

				physics::AxisRotation * simpleAxisRot = new physics::AxisRotation;
				simpleAxisRot->init(
					0.5f, 0.03f,
					Vec3C(0.0f, 1.0f, 0.0f),
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);

				physics::addTrackJoint(simpleAxisRot);

				if (1)
				{
					physics::AxisRotationLimit * simpleAxisRotLim = new physics::AxisRotationLimit;
					simpleAxisRotLim->init(
						0.5f, 0.03f,
						Vec3C(0.0f, 1.0f, 0.0f),
						angLo,
						angHi,
						body->m_bodyL,
						body->m_bodyR,
						nullptr,
						nullptr
						);

					physics::addTrackJoint(simpleAxisRotLim);
				}
			}
			else
			{
				physics::BallSocket * newBallSocket = new physics::BallSocket;
				Vec3 anchor = (body->m_bodyL->m_pos + bodyPrev->m_bodyL->m_pos) * 0.5f;
				newBallSocket->init(
					0.5f, 0.01f, anchor,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);

				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					newBallSocket->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(newBallSocket);

				physics::AxisRotation * simpleAxisRot = new physics::AxisRotation;
				simpleAxisRot->init(
					0.5f, 0.03f,
					Vec3C(0.0f, 1.0f, 0.0f),
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);

				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleAxisRot->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleAxisRot);

				if (1)
				{
					physics::AxisRotationLimit * simpleAxisRotLim = new physics::AxisRotationLimit;
					simpleAxisRotLim->init(
						0.5f, 0.03f,
						Vec3C(0.0f, 1.0f, 0.0f),
						angLo,
						angHi,
						body->m_bodyL,
						body->m_bodyR,
						bodyPrev->m_bodyL,
						bodyPrev->m_bodyR
						);

					if (applyLocalMassScale)
					{
						// Prev body is closer to the world attachment, and goes into joint second,
						//	also, invMass scale is setup here, so smaller scale -> heavier
						simpleAxisRotLim->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
					}
					physics::addTrackJoint(simpleAxisRotLim);
				}
			}
		}
	}
	if (1)
	{
		uint32_t bodyStartId = (uint32_t)physics::g_rigidBodies.size();

		const uint32_t numLinks = 10;
		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::NodeTranslational * body0L;
			body0L = physics::addTranslationalNode(1.0f / mass, staticPos + Vec3C(1 + bi * 3.0f, 6.0f, 0.0f), Vec3C());
			physics::NodeRotational * body0R = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

			physics::RigidBody * body0 = physics::createRigidBody();
			body0->m_bodyL = body0L;
			body0->m_bodyR = body0R;

			{
				ConvexBox * convexBox = new ConvexBox;
				physics::trackConvexHull(convexBox);
				convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
				convexBox->m_origin = Vec3C();
				convexBox->m_rotation.identity();

				physics::GeometryConvex * convex = new physics::GeometryConvex;
				physics::trackGeometryShape(convex);
				convex->m_density = 0.1f;
				convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
				convex->m_rotation.identity();
				convex->m_convexHullShape = convexBox;
				body0->m_geometryShapes.push_back(convex);

				body0->prepareGeometry();
			}
		}

		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::RigidBody * body = physics::g_rigidBodies[bodyStartId+bi];
			physics::RigidBody * bodyPrev = nullptr;
			if (bi >= 1)
				bodyPrev = physics::g_rigidBodies[bodyStartId+bi-1];

			if (bodyPrev == nullptr)
			{
				physics::BallSocket * newBallSocket = new physics::BallSocket;
				Vec3 anchor = body->m_bodyL->m_pos + Vec3C(-1.5f, 0.0f, 0.0f);
				newBallSocket->init(
					0.5f, 0.01f, anchor,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);

				physics::addTrackJoint(newBallSocket);

				physics::FixedRotation * simpleFixedRot = new physics::FixedRotation;
				simpleFixedRot->init(
					0.5f, 0.03f,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);

				physics::addTrackJoint(simpleFixedRot);
			}
			else
			{
				physics::BallSocket * newBallSocket = new physics::BallSocket;
				Vec3 anchor = (body->m_bodyL->m_pos + bodyPrev->m_bodyL->m_pos) * 0.5f;
				newBallSocket->init(
					0.5f, 0.01f, anchor,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);

				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					newBallSocket->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(newBallSocket);

				physics::FixedRotation * simpleFixedRot = new physics::FixedRotation;
				simpleFixedRot->init(
					0.5f, 0.03f,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);

				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleFixedRot->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleFixedRot);
			}
		}
	}
	if (1)
	{
		uint32_t bodyStartId = (uint32_t)physics::g_rigidBodies.size();

		const uint32_t numLinks = 10;
		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::NodeTranslational * body0L;
			body0L = physics::addTranslationalNode(1.0f / mass, staticPos + Vec3C(1 + bi * 1.5f, 6.0f, 10.0f), Vec3C());
			physics::NodeRotational * body0R = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

			physics::RigidBody * body0 = physics::createRigidBody();
			body0->m_bodyL = body0L;
			body0->m_bodyR = body0R;

			if (bi == numLinks - 1)
				body0L->m_vel = Vec3C(50.0f, 1.0f, 1.0f);

			{
				ConvexBox * convexBox = new ConvexBox;
				physics::trackConvexHull(convexBox);
				convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
				convexBox->m_origin = Vec3C();
				convexBox->m_rotation.identity();

				physics::GeometryConvex * convex = new physics::GeometryConvex;
				physics::trackGeometryShape(convex);
				convex->m_density = 0.1f;
				convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
				convex->m_rotation.identity();
				convex->m_convexHullShape = convexBox;
				body0->m_geometryShapes.push_back(convex);

				body0->prepareGeometry();
			}
		}

		for (uint32_t bi = 0; bi < numLinks; ++bi)
		{
			physics::RigidBody * body = physics::g_rigidBodies[bodyStartId+bi];
			physics::RigidBody * bodyPrev = nullptr;
			if (bi >= 1)
				bodyPrev = physics::g_rigidBodies[bodyStartId+bi-1];

			if (bodyPrev == nullptr)
			{
				const Vec3 sliderAxis = Vec3C(1.0f, 0.0f, 0.0f);
				physics::Slider * simpleSlider = new physics::Slider;
				simpleSlider->init(
					0.5f, 0.01f,
					sliderAxis,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);
				physics::addTrackJoint(simpleSlider);

				physics::AxisLinearLimit * simpleLinearLimit = new physics::AxisLinearLimit;
				simpleLinearLimit->init(
					0.5f, 0.01f,
					sliderAxis, 1.0f, 2.0f,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);
				physics::addTrackJoint(simpleLinearLimit);

#if 1
				physics::FixedRotation * simpleFixedRot = new physics::FixedRotation;
				simpleFixedRot->init(
					0.5f, 0.03f,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);
				physics::addTrackJoint(simpleFixedRot);
#else
				physics::AxisRotation * simpleAxisRot = new physics::AxisRotation;
				simpleAxisRot->init(
					0.5f, 0.03f,
					sliderAxis,
					body->m_bodyL,
					body->m_bodyR,
					nullptr,
					nullptr
					);
				physics::addTrackJoint(simpleAxisRot);
#endif
			}
			else
			{
				physics::Slider * simpleSlider = new physics::Slider;
				const Vec3 sliderAxis = Vec3C(1.0f, 0.0f, 0.0f);
				simpleSlider->init(
					0.5f, 0.01f,
					sliderAxis,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);
				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleSlider->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleSlider);

				physics::AxisLinearLimit * simpleLinearLimit = new physics::AxisLinearLimit;
				simpleLinearLimit->init(
					0.5f, 0.01f,
					sliderAxis, 1.0f, 3.0f,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);
				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleLinearLimit->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleLinearLimit);

#if 1
				physics::FixedRotation * simpleFixedRot = new physics::FixedRotation;
				simpleFixedRot->init(
					0.5f, 0.03f,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);
				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleFixedRot->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleFixedRot);
#else
				physics::AxisRotation * simpleAxisRot = new physics::AxisRotation;
				simpleAxisRot->init(
					0.5f, 0.03f,
					sliderAxis,
					body->m_bodyL,
					body->m_bodyR,
					bodyPrev->m_bodyL,
					bodyPrev->m_bodyR
					);
				if (applyLocalMassScale)
				{
					// Prev body is closer to the world attachment, and goes into joint second,
					//	also, invMass scale is setup here, so smaller scale -> heavier
					simpleAxisRot->setInvMassScales(localMassScaleRatio, 1.0f / localMassScaleRatio);
				}
				physics::addTrackJoint(simpleAxisRot);
#endif
			}
		}
	}

	for (size_t iNode = 1, iNodeEnd = physics::g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational)
			continue;
		physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);

		physics::PlaneConstraint * newPlaneConstraint = new physics::PlaneConstraint;
		newPlaneConstraint->init(0.1f, 0.001f, math::Vec3C(), math::Vec3C(0.0f, 1.0f, 0.0f), curNodeTrn, 0.01f);

		physics::addTrackJoint(newPlaneConstraint);
	}
}

inline void sceneStackCollisionTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.25f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	const bool tightStack = true;
	const bool tumblingStack = false;

	enum class ShapeType
	{
		eSphere,
		eBox,
		eCustomConvex
	};
	ShapeType stackShapeType = ShapeType::eBox;

	const size_t numConvexPoints = 25;
	std::vector<Vec3> convexHullpoints;
	convexHullpoints.resize(numConvexPoints);
	int cdStartNumNodes = (int)physics::g_nodes.size();
	for (int i = 0; i < 10; ++i)
	{
		const float skinWidth = 0.01f;

		Vec3 additionalOffset = Vec3C((i&1)*0.1f + (tumblingStack?(i*0.1f):0.0f), 0.0f, 0.0f);

		physics::NodeTranslational * bodyL;
		if (tightStack)
			bodyL = physics::addTranslationalNode(1.0f / mass, staticPos + Vec3C(1, 1.0f + i * (1.0f - skinWidth), 0.0f) + additionalOffset, Vec3C());
		else
			bodyL = physics::addTranslationalNode(1.0f / mass, staticPos + Vec3C(1, 2.0f + i * (2.0f - skinWidth), 0.0f) + additionalOffset, Vec3C());
		physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

		bodyL->m_skinWidth = skinWidth;

		physics::RigidBody * body = physics::createRigidBody();
		body->m_bodyL = bodyL;
		body->m_bodyR = bodyR;

		physics::GeometryConvex * convex;
		if (stackShapeType == ShapeType::eCustomConvex)
		{
			Vec3 halfExtents = Vec3C(0.85f, 0.85f, 0.85f);
			for (uint32_t pi = 0; pi < numConvexPoints; ++pi)
			{
				Vec3 randomVec = Vec3C((rand()%10000) / 10000.0f, (rand()%10000) / 10000.0f, (rand()%10000) / 10000.0f);
				convexHullpoints[pi] = Vec3C(0.0f, 0.0f, 0.0f) + randomVec + Vec3C(-0.5f, -0.5f, -0.5f);
				convexHullpoints[pi].x *= halfExtents.x;
				convexHullpoints[pi].y *= halfExtents.y;
				convexHullpoints[pi].z *= halfExtents.z;
			}

			ConvexCustom * convexHull = new ConvexCustom;
			physics::trackConvexHull(convexHull);
			convexHull->m_origin = Vec3C();
			convexHull->m_rotation.identity();
			convexHull->m_points = convexHullpoints;

			convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexHull;
		}
		else if (stackShapeType == ShapeType::eBox)
		{
			ConvexBox * convexBox = new ConvexBox;
			physics::trackConvexHull(convexBox);
			convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexBox;
		}
		else
		{
			ConvexSphere * convexSphere = new ConvexSphere;
			physics::trackConvexHull(convexSphere);
			convexSphere->m_radius = 0.5f;
			convexSphere->m_origin = Vec3C();

			convex = new physics::GeometryConvex;
			physics::trackGeometryShape(convex);
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexSphere;
		}
		body->m_geometryShapes.push_back(convex);

		body->prepareGeometry();
	}
	for (size_t iNode = cdStartNumNodes, iNodeEnd = physics::g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational)
			continue;
		physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);

		physics::PlaneConstraint * newPlaneConstraint = new physics::PlaneConstraint;
		newPlaneConstraint->init(0.1f, 0.001f, math::Vec3C(), math::Vec3C(0.0f, 1.0f, 0.0f), curNodeTrn, 0.01f);
			
		physics::addTrackJoint(newPlaneConstraint);
	}
}

inline void sceneStackCollisionTestMassive(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.5f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	const bool stableBoxStack = false;

	int cdStartNumNodes = (int)physics::g_nodes.size();
	const int stackNumI = 10;
	const int stackNumJ = 10;
	for (int sj = 0; sj < stackNumJ; ++sj)
	{
		float sj_nrm = sj / (float)(stackNumJ - 1);
		for (int si = 0; si < stackNumI; ++si)
		{
			float si_nrm = si / (float)(stackNumI - 1);
			
			Vec3 stackOffset = Vec3C(10.0f * (si_nrm * 2.0f - 1.0f), 0.0f, 10.0f * (sj_nrm * 2.0f - 1.0f));
			for (int i = 0; i < 10; ++i)
			{
				const float skinWidth = 0.01f;

				physics::NodeTranslational * bodyL;
				if (stableBoxStack)
					bodyL = physics::addTranslationalNode(1.0f / mass, staticPos + stackOffset + Vec3C(1 + (i&1)*0.1f, 1.0f + i * (1.0f - skinWidth), 0.0f), Vec3C());
				else
					bodyL = physics::addTranslationalNode(1.0f / mass, staticPos + stackOffset + Vec3C(1 + (i&1)*0.1f, 2.0f + i * (2.0f - skinWidth), 0.0f), Vec3C());
				physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

				bodyL->m_skinWidth = skinWidth;

				physics::RigidBody * body = physics::createRigidBody();
				body->m_bodyL = bodyL;
				body->m_bodyR = bodyR;

				ConvexBox * convexBox = new ConvexBox;
				physics::trackConvexHull(convexBox);
				convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
				convexBox->m_origin = Vec3C();
				convexBox->m_rotation.identity();

				physics::GeometryConvex * convex = new physics::GeometryConvex;
				physics::trackGeometryShape(convex);
				convex->m_density = 1.0f;
				convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
				convex->m_rotation.identity();
				convex->m_convexHullShape = convexBox;
				body->m_geometryShapes.push_back(convex);

				body->prepareGeometry();
			}
		}
	}
	for (size_t iNode = cdStartNumNodes, iNodeEnd = physics::g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational)
			continue;
		physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);

		physics::PlaneConstraint * newPlaneConstraint = new physics::PlaneConstraint;
		newPlaneConstraint->init(0.1f, 0.001f, math::Vec3C(), math::Vec3C(0.0f, 1.0f, 0.0f), curNodeTrn, 0.01f);

		physics::addTrackJoint(newPlaneConstraint);
	}
}

inline void sceneShapesCollisionTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.5f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	auto addUnitMassConvexBody = [](const Vec3 & pos, float skinWidth, ConvexHullShape * hullShape)
	{
		physics::NodeTranslational * bodyL;
		bodyL = physics::addTranslationalNode(0.0f, pos, Vec3C());
		physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().zero(), Quat().id(), Vec3C());

		bodyL->m_skinWidth = skinWidth;

		physics::RigidBody * body = physics::createRigidBody();
		body->m_bodyL = bodyL;
		body->m_bodyR = bodyR;

		physics::GeometryConvex * convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_density = 1.0f;
		convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
		convex->m_rotation.identity();
		convex->m_convexHullShape = hullShape;
		body->m_geometryShapes.push_back(convex);

		// Setting desired mass to 1.0 so that all bodies have equal mass to avoid heavy-on-light stacking artifacts
		body->prepareGeometry(1.0f);
	};

	int cdStartNumNodes = (int)physics::g_nodes.size();
	for (int i = 0; i < 5; ++i)
	{
		const float skinWidth = 0.01f;

		ConvexCylinder * convexCylinder = new ConvexCylinder;
		physics::trackConvexHull(convexCylinder);
		convexCylinder->m_radHeight = Vec3C(((rand()%1000) / 1000.0f) * 0.8f + 0.2f, ((rand()%1000) / 1000.0f) * 0.8f + 0.2f, 0.5f);
		convexCylinder->m_origin = Vec3C();
		convexCylinder->m_rotation.identity();

		addUnitMassConvexBody(staticPos + Vec3C( 2 + (i&1)*0.1f, 1.5f + i * (2.0f - skinWidth), 0.0f), skinWidth, convexCylinder);
	}
	for (int i = 0; i < 5; ++i)
	{
		const float skinWidth = 0.01f;

		ConvexCone * convexCone = new ConvexCone;
		physics::trackConvexHull(convexCone);
		convexCone->m_radHeight = Vec3C(((rand()%1000) / 1000.0f) * 0.8f + 0.2f, ((rand()%1000) / 1000.0f) * 0.7f + 0.3f, 0.5f);
		convexCone->m_origin = Vec3C();
		convexCone->m_rotation.identity();

		addUnitMassConvexBody(staticPos + Vec3C(-2 + (i&1)*0.1f, 1.5f + i * (2.0f - skinWidth), 0.0f), skinWidth, convexCone);
	}
	for (int i = 0; i < 5; ++i)
	{
		const float skinWidth = 0.01f;

		ConvexCapsule * convexCapsule = new ConvexCapsule;
		physics::trackConvexHull(convexCapsule);
		convexCapsule->m_radHeight = Vec3C(((rand()%1000) / 1000.0f) * 0.15f + 0.1f, ((rand()%1000) / 1000.0f) * 0.7f + 0.3f, 0.5f);
		convexCapsule->m_origin = Vec3C();
		convexCapsule->m_rotation.identity();

		addUnitMassConvexBody(staticPos + Vec3C( 2 + (i&1)*0.1f, 1.5f + i * (2.0f - skinWidth), 3.0f), skinWidth, convexCapsule);
	}

	const size_t numConvexPoints = 25;
	std::vector<Vec3> convexHullpoints;
	convexHullpoints.resize(numConvexPoints);

	for (int i = 0; i < 5; ++i)
	{
		const float skinWidth = 0.01f;

		// Not the most effective way to create convex, as it will have to test points that do not belong to
		//	the convex hull when query for the support point is happening, this increasing NP CD time.
		//	Points which do not belong to the convex hull should be removed, but for the simplistic test
		//	like this, it should work.
		Vec3 halfExtents = Vec3C(0.85f, 0.85f, 0.85f);
		for (uint32_t pi = 0; pi < numConvexPoints; ++pi)
		{
			Vec3 randomVec = Vec3C((rand()%10000) / 10000.0f, (rand()%10000) / 10000.0f, (rand()%10000) / 10000.0f);
			convexHullpoints[pi] = Vec3C(0.0f, 0.0f, 0.0f) + 2.0f*randomVec + Vec3C(-1.0f, -1.0f, -1.0f);
			convexHullpoints[pi].x *= halfExtents.x;
			convexHullpoints[pi].y *= halfExtents.y;
			convexHullpoints[pi].z *= halfExtents.z;
		}

		ConvexCustom * convexHull = new ConvexCustom;
		physics::trackConvexHull(convexHull);
		convexHull->m_origin = Vec3C();
		convexHull->m_rotation.identity();

		convexHull->m_points = convexHullpoints;

		addUnitMassConvexBody(staticPos + Vec3C(-2 + (i&1)*0.1f, 1.5f + i * (2.0f - skinWidth), 3.0f), skinWidth, convexHull);
	}

	for (size_t iNode = cdStartNumNodes, iNodeEnd = physics::g_nodes.size(); iNode < iNodeEnd; ++iNode)
	{
		if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational)
			continue;
		physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);

		physics::PlaneConstraint * newPlaneConstraint = new physics::PlaneConstraint;
		newPlaneConstraint->init(0.1f, 0.001f, math::Vec3C(), math::Vec3C(0.0f, 1.0f, 0.0f), curNodeTrn, 0.01f);

		physics::addTrackJoint(newPlaneConstraint);
	}
}

inline void sceneGyro(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	physics::g_gravity = Vec3C();

	auto addTBodyNoDamping = [](const Vec3 & pos, const Vec3 & rotVel, physics::NodeTranslational ** outNodeL = nullptr, physics::NodeRotational ** outNodeR = nullptr)
	{
		physics::NodeTranslational * bodyL = physics::addTranslationalNode(1.0f, pos, Vec3C());
		physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().identity(), Quat().id(), Vec3C());

		physics::RigidBody * body = physics::createRigidBody();
		body->m_bodyL = bodyL;
		body->m_bodyR = bodyR;

		if (outNodeL)
			*outNodeL = bodyL;

		if (outNodeR)
			*outNodeR = bodyR;

		bodyR->m_vel = rotVel;
		bodyR->m_dampingMul = 1.0f;
		bodyR->m_dampingSub = 0.0f;

		ConvexBox * convexBox = new ConvexBox;
		physics::trackConvexHull(convexBox);
		convexBox->m_halfExtents = Vec3C(0.1f, 0.5f, 0.1f);
		convexBox->m_origin = Vec3C();
		convexBox->m_rotation.identity();

		physics::GeometryConvex * convex0 = new physics::GeometryConvex;
		physics::trackGeometryShape(convex0);
		convex0->m_density = 1.0f;
		convex0->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
		convex0->m_rotation.identity();
		convex0->m_convexHullShape = convexBox;
		body->m_geometryShapes.push_back(convex0);

		ConvexBox * convexBox2 = new ConvexBox;
		physics::trackConvexHull(convexBox2);
		convexBox2->m_halfExtents = Vec3C(1.0f, 0.1f, 0.1f);
		convexBox2->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
		convexBox2->m_rotation.identity();

		physics::GeometryConvex * convex1 = new physics::GeometryConvex;
		physics::trackGeometryShape(convex1);
		convex1->m_density = 10.0f;
		convex1->m_origin = Vec3C(0.0f, 0.5f, 0.0f);
		convex1->m_rotation.identity();
		convex1->m_convexHullShape = convexBox2;
		body->m_geometryShapes.push_back(convex1);

		body->prepareGeometry();
	};

	// Default, body rotational part will be integrated with gyroscopic term,
	//	illustrating intermediate axis theorem
	addTBodyNoDamping(Vec3C(-5, 1.6f, -7.0f), Vec3C(0.01f, 20.0f, 0.0f));

	// Second body will have gyropoic term omitted
	physics::NodeTranslational * noGyroBodyL;
	physics::NodeRotational * noGyroBodyR;
	addTBodyNoDamping(Vec3C( 5, 1.6f, -7.0f), Vec3C(0.01f, 20.0f, 0.0f), &noGyroBodyL, &noGyroBodyR);
	noGyroBodyR->m_flags = physics::NodeRotational::Flags::eNoGyroscopic;
}

inline void sceneCompoundBodyTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(25.0f, 0.5f, 25.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.25f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	physics::NodeTranslational * bodyL = physics::addTranslationalNode(1.0f / mass, Vec3C(0, 2.1f, 0.0f), Vec3C());

	physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, Quat().id(), Vec3C());

	physics::RigidBody * body = physics::createRigidBody();
	body->m_bodyL = bodyL;
	body->m_bodyR = bodyR;

	ConvexBox * convexBox0 = new ConvexBox;
	physics::trackConvexHull(convexBox0);
	convexBox0->m_halfExtents = Vec3C(0.1f, 1.0f, 0.1f);
	convexBox0->m_origin = Vec3C();
	convexBox0->m_rotation.identity();

	physics::GeometryConvex * convex0 = new physics::GeometryConvex;
	physics::trackGeometryShape(convex0);
	convex0->m_density = 1.0f;
	convex0->m_origin = Vec3C(-1.0f, 0.0f, 0.0f);
	convex0->m_rotation.identity();
	convex0->m_convexHullShape = convexBox0;
	body->m_geometryShapes.push_back(convex0);

	ConvexBox * convexBox1 = new ConvexBox;
	physics::trackConvexHull(convexBox1);
	convexBox1->m_halfExtents = Vec3C(0.9f, 0.1f, 0.1f);
	convexBox1->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
	convexBox1->m_rotation.identity();

	physics::GeometryConvex * convex1 = new physics::GeometryConvex;
	physics::trackGeometryShape(convex1);
	convex1->m_density = 1.0f;
	convex1->m_origin = Vec3C(0.0f, 0.9f, 0.0f);
	convex1->m_rotation.identity();
	convex1->m_convexHullShape = convexBox1;
	body->m_geometryShapes.push_back(convex1);

	body->prepareGeometry(100.01f);
}

inline void sceneSpinningTest(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const Vec3 staticPos = Vec3C(0, 0.0f, 0.0f);
	const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
	const bool staticWalls = true;
	const float wallHeight = 5.0f;
	const float wallThickness = 0.25f;
	createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

	if (1)
	{
		// Edge
		physics::NodeTranslational * bodyL = physics::addTranslationalNode(1.0f / mass, Vec3C(1, 2.1f, 1.0f), Vec3C());
		Quat weirdQuat(1.0f, 0.4f, 0.0f, 0.0f);
		weirdQuat.normalize();
		physics::NodeRotational * bodyR = physics::addRotationalNode(inertiaMatrix, weirdQuat, Vec3C());

		physics::RigidBody * body = physics::createRigidBody();
		body->m_bodyL = bodyL;
		body->m_bodyR = bodyR;

		//bodyR->m_flags = physics::NodeRotational::Flags::eNoGyroscopic;

		bodyL->m_frictionCoeff = 0.01f;
		bodyR->m_vel = weirdQuat.rotate(Vec3C(0.0f, 50.0f, 0.0f));

		ConvexCapsule * convexCapsule = new ConvexCapsule;
		physics::trackConvexHull(convexCapsule);
		convexCapsule->m_radHeight = Vec3C(0.25f, 1.5f, 0.5f);
		convexCapsule->m_origin = Vec3C();
		convexCapsule->m_rotation.identity();

		physics::GeometryConvex * convex = new physics::GeometryConvex;
		physics::trackGeometryShape(convex);
		convex->m_convexHullShape = convexCapsule;
		convex->m_density = 1.0f;
		convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
		convex->m_rotation.identity();
		body->m_geometryShapes.push_back(convex);

		body->prepareGeometry();
	}
}

// FEM objects creation helpers
class FEMTetrahedron
{
public:
	uint32_t ind1, ind2, ind3, ind4;
};
class FEMObject
{
public:
	std::vector<FEMTetrahedron> FEMTetras;
	std::vector<math::Vec3> FEMNodes;
};

inline void buildCube12(
	FEMObject &Object,
	const math::Vec3 & pos, const math::Vec3 & extents, const math::Mat33 & rot,
	unsigned segWidth, unsigned segHeight, unsigned segDepth
	)
{
	using namespace math;

	unsigned i, j, k;

	float	dX = 2.0f * extents.x / (float)(segWidth),
			dY = 2.0f * extents.y / (float)(segHeight),
			dZ = 2.0f * extents.z / (float)(segDepth);

	Vec3 curNodePos;

	uint32_t firstVertexIdx = (uint32_t)Object.FEMNodes.size();
	Object.FEMNodes.reserve(firstVertexIdx + (segWidth + 1) * (segHeight + 1) * (segDepth + 1) + segWidth * segHeight * segDepth);

	// Place cubes' border
	curNodePos.x = -extents.x;
	for (i = 0; i < segWidth + 1; ++i, curNodePos.x += dX)
	{
		curNodePos.y = -extents.y;
		for (j = 0; j < segHeight + 1; ++j, curNodePos.y += dY)
		{
			curNodePos.z = -extents.z;
			for (k = 0; k < segDepth + 1; ++k, curNodePos.z += dZ)
			{
				Object.FEMNodes.push_back(rot * curNodePos + pos);
			}
		}
	}

	// Place cubes' middle
	curNodePos.x = -extents.x + dX * 0.5f;
	for (i = 0; i < segWidth; ++i, curNodePos.x += dX)
	{
		curNodePos.y = -extents.y + dY * 0.5f;
		for (j = 0; j < segHeight; ++j, curNodePos.y += dY)
		{
			curNodePos.z = -extents.z + dZ * 0.5f;
			for (k = 0; k < segDepth; ++k, curNodePos.z += dZ)
			{
				Object.FEMNodes.push_back(rot * curNodePos + pos);
			}
		}
	}

	unsigned MidShift = firstVertexIdx + (segWidth + 1) * (segHeight + 1) * (segDepth + 1);

	// 12 - number of tetrahedrons per cube (6 pyramids, 2 tetra per pyramid)
	Object.FEMTetras.reserve(Object.FEMTetras.size() + segWidth * segHeight * segDepth * 12);

#define idx(i, j, k)	(firstVertexIdx + (i) * (segHeight + 1) * (segDepth + 1) + (j) * (segDepth + 1) + (k))

	// pyr_ind - pyramid's base indices, CW (from the vertex)
	auto formTetraFromPyramid = [](
		FEMObject & object, unsigned pyr_vertex,
		unsigned pyr_ind1, unsigned pyr_ind2, unsigned pyr_ind3, unsigned pyr_ind4
		)
	{
		FEMTetrahedron tmpTetra;

		// Tetra1
		tmpTetra.ind1 = pyr_ind1;
		tmpTetra.ind2 = pyr_ind2;
		tmpTetra.ind3 = pyr_ind3;
		tmpTetra.ind4 = pyr_vertex;
		object.FEMTetras.push_back(tmpTetra);

		// Tetra2
		tmpTetra.ind1 = pyr_ind3;
		tmpTetra.ind2 = pyr_ind4;
		tmpTetra.ind3 = pyr_ind1;
		tmpTetra.ind4 = pyr_vertex;
		object.FEMTetras.push_back(tmpTetra);
	};

	// Mark up tetrahedrons
	for (i = 0; i < segWidth; ++i)
	{
		for (j = 0; j < segHeight; ++j)
		{
			for (k = 0; k < segDepth; ++k)
			{
				unsigned MidIndex = (i * (segHeight * segDepth) + j * segDepth + k) + MidShift;

				// PYRAMID TOP
				formTetraFromPyramid(Object, MidIndex,	idx( i , j+1,  k ), idx( i , j+1, k+1),
														idx(i+1, j+1, k+1), idx(i+1, j+1,  k ));

				// PYRAMID BOTTOM
				formTetraFromPyramid(Object, MidIndex,  idx( i ,  j , k+1), idx( i ,  j ,  k ),
														idx(i+1,  j ,  k ), idx(i+1,  j , k+1));

				// PYRAMID RIGHT
				formTetraFromPyramid(Object, MidIndex,  idx(i+1, j+1, k+1), idx(i+1,  j , k+1),
														idx(i+1,  j ,  k ), idx(i+1, j+1,  k ));

				// PYRAMID LEFT
				formTetraFromPyramid(Object, MidIndex,  idx( i , j+1,  k ), idx( i ,  j ,  k ),
														idx( i ,  j , k+1), idx( i , j+1, k+1));

				// PYRAMID BACK
				formTetraFromPyramid(Object, MidIndex,  idx(i+1, j+1,  k ), idx(i+1,  j ,  k ),
														idx( i ,  j ,  k ), idx( i , j+1,  k ));

				// PYRAMID FRONT
				formTetraFromPyramid(Object, MidIndex,  idx( i , j+1, k+1), idx( i ,  j , k+1),
														idx(i+1,  j , k+1), idx(i+1, j+1, k+1));
			}
		}
	}

#undef idx
};

inline void buildCube5(
	FEMObject & object,
	const math::Vec3 & pos, const math::Vec3 & extents, const math::Mat33 & rot,
	unsigned segWidth, unsigned segHeight, unsigned segDepth
	)
{
	using namespace math;

	unsigned i, j, k;

	float	dX = 2.0f * extents.x / (float)(segWidth),
			dY = 2.0f * extents.y / (float)(segHeight),
			dZ = 2.0f * extents.z / (float)(segDepth);

	Vec3 curNodePos;

	uint32_t firstVertexIdx = (uint32_t)object.FEMNodes.size();
	object.FEMNodes.reserve(firstVertexIdx + (segWidth + 1) * (segHeight + 1) * (segDepth + 1));

	// Place cubes' border
	curNodePos.x = -extents.x;
	for (i = 0; i < segWidth + 1; ++i, curNodePos.x += dX)
	{
		curNodePos.y = -extents.y;
		for (j = 0; j < segHeight + 1; ++j, curNodePos.y += dY)
		{
			curNodePos.z = -extents.z;
			for (k = 0; k < segDepth + 1; ++k, curNodePos.z += dZ)
			{
				object.FEMNodes.push_back(rot * curNodePos + pos);
			}
		}
	}

	const uint32_t numTetrasPerCube = 5;
	object.FEMTetras.reserve(object.FEMTetras.size() + segWidth * segHeight * segDepth * numTetrasPerCube);

#define idx(i, j, k)	(firstVertexIdx + (i) * (segHeight + 1) * (segDepth + 1) + (j) * (segDepth + 1) + (k))

	// Mark up tetrahedrons
	for (i = 0; i < segWidth; ++i)
	{
		for (j = 0; j < segHeight; ++j)
		{
			for (k = 0; k < segDepth; ++k)
			{
				uint32_t cubeIndices[8] =
					{
						idx(i  , j  , k  ),
						idx(i+1, j  , k  ),
						idx(i  , j+1, k  ),
						idx(i+1, j+1, k  ),
						idx(i  , j  , k+1),
						idx(i+1, j  , k+1),
						idx(i  , j+1, k+1),
						idx(i+1, j+1, k+1)
					};

				FEMTetrahedron tmpTetra;

				// Corner tetra 0
				//int t0idx0 = 1, t0idx1 = 0, t0idx2 = 2, t0idx3 = 4;
				tmpTetra.ind1 = cubeIndices[1];
				tmpTetra.ind2 = cubeIndices[0];
				tmpTetra.ind3 = cubeIndices[2];
				tmpTetra.ind4 = cubeIndices[4];
				object.FEMTetras.push_back(tmpTetra);

				// Corner tetra 1
				//int t1idx0 = 5, t1idx1 = 4, t1idx2 = 1, t1idx3 = 7;
				tmpTetra.ind1 = cubeIndices[5];
				tmpTetra.ind2 = cubeIndices[4];
				tmpTetra.ind3 = cubeIndices[1];
				tmpTetra.ind4 = cubeIndices[7];
				object.FEMTetras.push_back(tmpTetra);

				// Corner tetra 2
				//int t2idx0 = 2, t2idx1 = 3, t2idx2 = 1, t2idx3 = 7;
				tmpTetra.ind1 = cubeIndices[2];
				tmpTetra.ind2 = cubeIndices[3];
				tmpTetra.ind3 = cubeIndices[1];
				tmpTetra.ind4 = cubeIndices[7];
				object.FEMTetras.push_back(tmpTetra);

				// Corner tetra 3
				//int t3idx0 = 6, t3idx1 = 7, t3idx2 = 2, t3idx3 = 4;
				tmpTetra.ind1 = cubeIndices[6];
				tmpTetra.ind2 = cubeIndices[7];
				tmpTetra.ind3 = cubeIndices[2];
				tmpTetra.ind4 = cubeIndices[4];
				object.FEMTetras.push_back(tmpTetra);

				// Center tetra (4)
				//int t4idx0 = 2, t4idx1 = 7, t4idx2 = 1, t4idx3 = 4;
				tmpTetra.ind1 = cubeIndices[2];
				tmpTetra.ind2 = cubeIndices[7];
				tmpTetra.ind3 = cubeIndices[1];
				tmpTetra.ind4 = cubeIndices[4];
				object.FEMTetras.push_back(tmpTetra);
			}
		}
	}

#undef idx
};

inline void sceneFEMBeam(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	const bool finerCubeSegmentation = false;

	// FEM test
	if (1)
	{
		FEMObject FEMCube;
		FEMCube.FEMNodes.push_back(Vec3C(0.0f, 0.0f, 0.0f));
		uint32_t firstNodeIdx = (uint32_t)physics::g_nodes.size();

		uint32_t CubeSegWidth = 48, CubeSegHeight = 2, CubeSegDepth = 1;
		if (finerCubeSegmentation)
			buildCube12(FEMCube, Vec3C(0.0f, 10.5f, 0.0f), Vec3C(24.0f, 1.0f, 0.5f), Mat33().identity(), CubeSegWidth, CubeSegHeight, CubeSegDepth);
		else
			buildCube5(FEMCube, Vec3C(0.0f, 10.5f, 0.0f), Vec3C(24.0f, 1.0f, 0.5f), Mat33().identity(), CubeSegWidth, CubeSegHeight, CubeSegDepth);
		float TotalMass = 2200.0f;

		int size = (int)FEMCube.FEMNodes.size();
		// TODO: remember the offset for the cube in the solver
		for (int node = 0; node < size; ++node)
		{
			float mass = TotalMass / (float)size;

			float invmass = node < 2 ? 0.0f : (1.0f / mass);
			if (node == 5 || node == 13) invmass = 0.0f;

			physics::NodeTranslational * curNode = physics::addTranslationalNode(invmass, FEMCube.FEMNodes[node], Vec3C(0.0f, 0.0f, 0.0f));

		}

		// Elasticity
		float Young_Modulus = 1000.0f * 1000.0f * 1* 1.0f * 1.0f;
		float Poisson_Ratio = 0.35f;

		// Plasticity
		// c_creep [0..1/dt]
		// c_yeld - threshold for plastic strain
		// c_max - maximum plastic strain for FE
		float c_yeld = 0.05f, c_creep = 0.01f, c_max = 0.3f;
		float beta = 0.03f;										// Beta (C.O.)

		unsigned FE_cnt = 0;
		for (auto itTetra = FEMCube.FEMTetras.begin(), itTetraEnd = FEMCube.FEMTetras.end(); itTetra != itTetraEnd; ++itTetra)
		{
			++FE_cnt;

			physics::NodeTranslational * n0 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind1]);
			physics::NodeTranslational * n1 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind2]);
			physics::NodeTranslational * n2 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind3]);
			physics::NodeTranslational * n3 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind4]);

			physics::FEMJoint * newFEMJoint = new physics::FEMJoint;
			// Regularization = 0.0001
			newFEMJoint->init(solver,
							n0, n1, n2, n3,
							Young_Modulus, Poisson_Ratio, c_yeld, c_creep, c_max, beta, 0.000005f);

			physics::addTrackJoint(newFEMJoint);
		}
	}
}

inline void sceneFEMBeamSwirl(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	const bool finerCubeSegmentation = true;

	// FEM test
	if (1)
	{
		// Adding FEM beam
		///////////////////////////////////////////////////////////////////////////////////////
		FEMObject FEMCube;
		// It is a requirement to add a node 0 to the simulator that represents static node
		physics::addTranslationalNode(0.0f, Vec3C(0.0f, 0.0f, 0.0f), Vec3C(0.0f, 0.0f, 0.0f));
		uint32_t firstNodeIdx = (uint32_t)physics::g_nodes.size();

		uint32_t cubeSegWidth = 48, cubeSegHeight = 1, cubeSegDepth = 1;
		if (finerCubeSegmentation)
			buildCube12(FEMCube, Vec3C(0.0f, 10.5f, 0.0f), Vec3C(24.0f, 0.5f, 0.5f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		else
			buildCube5(FEMCube, Vec3C(0.0f, 10.5f, 0.0f), Vec3C(24.0f, 0.5f, 0.5f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		float totalMass = 2200.0f;

		int totalNumSubCubes = cubeSegWidth * cubeSegHeight * cubeSegDepth;
		int totalNumNodes = (int)FEMCube.FEMNodes.size();
		for (int node = 0; node < totalNumNodes; ++node)
		{
			float mass = totalMass / (float)totalNumNodes;

			float invmass = 1.0f / mass;

			// Fixed nodes
			if (node == 0  || node == 1 || node == 2 || node == 3)
				invmass = 0.0f;

			physics::NodeTranslational * curNode = physics::addTranslationalNode(invmass, FEMCube.FEMNodes[node], Vec3C(0.0f, 0.0f, 0.0f));
		}

		physics::NodeTranslational * lastFourNodes[4];
		if (finerCubeSegmentation)
		{
			lastFourNodes[0] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - totalNumSubCubes - 1]);
			lastFourNodes[1] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - totalNumSubCubes - 2]);
			lastFourNodes[2] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - totalNumSubCubes - 3]);
			lastFourNodes[3] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - totalNumSubCubes - 4]);
		}
		else
		{
			lastFourNodes[0] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - 1]);
			lastFourNodes[1] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - 2]);
			lastFourNodes[2] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - 3]);
			lastFourNodes[3] = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx + totalNumNodes - 4]);
		}

		// Elasticity
		float youngModulus = 1000.0f * 10.0f * 1* 1.0f * 1.0f;
		float poissonRatio = 0.35f;

		// Plasticity [disabled for this test]
		// c_creep [0..1/dt]
		// c_yeld - threshold for plastic strain
		// c_max - maximum plastic strain for FE
		float c_yeld = 0.05f, c_creep = 0.01f, c_max = 0.0f;
		float beta = 0.03f;										// Beta (C.O.)

		unsigned cntFE = 0;
		for (auto itTetra = FEMCube.FEMTetras.begin(), itTetraEnd = FEMCube.FEMTetras.end(); itTetra != itTetraEnd; ++itTetra)
		{
			++cntFE;

			physics::NodeTranslational * n0 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind1]);
			physics::NodeTranslational * n1 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind2]);
			physics::NodeTranslational * n2 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind3]);
			physics::NodeTranslational * n3 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind4]);

			physics::FEMJoint * newFEMJoint = new physics::FEMJoint;
			// Regularization = 0.0001
			newFEMJoint->init(
				solver,
				n0, n1, n2, n3,
				youngModulus, poissonRatio,
				c_yeld, c_creep, c_max,
				beta, 0.000005f
				);

			physics::addTrackJoint(newFEMJoint);
		}

		Vec3 femSocket0Anchor = (lastFourNodes[0]->m_pos + lastFourNodes[1]->m_pos + lastFourNodes[2]->m_pos) / 3.0f;
		Vec3 femSocket1Anchor = (lastFourNodes[1]->m_pos + lastFourNodes[3]->m_pos + lastFourNodes[2]->m_pos) / 3.0f;

		// Adding proxy body
		///////////////////////////////////////////////////////////////////////////////////////
		float bodyMass = 0.1f * totalMass;
		physics::NodeTranslational * bodyL;
		physics::NodeRotational * bodyR;
		{
			Vec3 bodyPos = 0.5f*(femSocket0Anchor + femSocket1Anchor) + Vec3C(1.0f, 0.0f, 0.0);
			bodyL = physics::addTranslationalNode(0.0f, bodyPos, Vec3C());
			bodyR = physics::addRotationalNode(Mat33().zero(), Quat().id(), Vec3C());

			bodyL->m_skinWidth = 0.01f;

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			ConvexBox * convexBox0 = new ConvexBox;
			physics::trackConvexHull(convexBox0);
			convexBox0->m_halfExtents = Vec3C(0.2f, 0.2f, 1.0f);
			convexBox0->m_origin = Vec3C();
			convexBox0->m_rotation.identity();

			physics::GeometryConvex * convexBox0Geom = new physics::GeometryConvex;
			physics::trackGeometryShape(convexBox0Geom);
			convexBox0Geom->m_density = 1.0f;
			convexBox0Geom->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convexBox0Geom->m_rotation.identity();
			convexBox0Geom->m_convexHullShape = convexBox0;
			body->m_geometryShapes.push_back(convexBox0Geom);

			ConvexBox * convexBox1 = new ConvexBox;
			physics::trackConvexHull(convexBox1);
			convexBox1->m_halfExtents = Vec3C(0.25f, 1.0f, 0.25f);
			convexBox1->m_origin = Vec3C();
			convexBox1->m_rotation.identity();

			physics::GeometryConvex * convexBox1Geom = new physics::GeometryConvex;
			physics::trackGeometryShape(convexBox1Geom);
			convexBox1Geom->m_density = 1.0f;
			convexBox1Geom->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convexBox1Geom->m_rotation.identity();
			convexBox1Geom->m_convexHullShape = convexBox1;
			body->m_geometryShapes.push_back(convexBox1Geom);

			// Setting desired mass to 1.0 so that all bodies have equal mass to avoid heavy-on-light stacking artifacts
			body->prepareGeometry(bodyMass);
		}

		// Add Hinge joint that attaches body to the static world
		///////////////////////////////////////////////////////////////////////////////////////
		//BallSocket * hingePoint = new BallSocket;
		//Vec3 hingeAnchor = bodyL->m_pos + Vec3C(1.0f, 0.0f, 0.0f);
		//hingePoint->init(
		//	0.5f, 0.01f, hingeAnchor,
		//	bodyL,
		//	bodyR,
		//	nullptr,
		//	nullptr
		//	);

		//physics::addTrackJoint(hingePoint);

		physics::Slider * hingeSlider = new physics::Slider;
		Vec3 hingeAxis = Vec3C(1.0f, 0.0f, 0.0f);
		Vec3 hingeAnchor = bodyL->m_pos + hingeAxis;
		hingeSlider->init(
			0.5f, 0.01f,
			hingeAxis,
			bodyL,
			bodyR,
			nullptr,
			nullptr
			);
		physics::addTrackJoint(hingeSlider);

		physics::AxisLinearLimit * simpleLinearLimit = new physics::AxisLinearLimit;
		simpleLinearLimit->init(
			0.5f, 0.01f,
			hingeAxis, -5.0f, 5.0f,
			bodyL,
			bodyR,
			nullptr,
			nullptr
			);
		physics::addTrackJoint(simpleLinearLimit);

		physics::AxisRotation * hingeAxisRot = new physics::AxisRotation;
		hingeAxisRot->init(
			0.7f, 0.001f,
			hingeAxis,
			bodyL,
			bodyR,
			nullptr,
			nullptr
			);
		physics::addTrackJoint(hingeAxisRot);

		physics::MotorRotation * hingeAxisMotor = new physics::MotorRotation;
		hingeAxisMotor->init(
			0.01f,
			hingeAxis,
			0.1f,
			bodyL,
			bodyR,
			nullptr,
			nullptr
			);
		hingeAxisMotor->setForceMin(-500000.0f);
		hingeAxisMotor->setForceMax( 500000.0f);
		physics::addTrackJoint(hingeAxisMotor);

		// Add FEM BallSockets
		///////////////////////////////////////////////////////////////////////////////////////
		physics::BallSocket_FEM_Simple * simpleFemSocket0 = new physics::BallSocket_FEM_Simple;
		simpleFemSocket0->init(solver, 0.7f, 0.001f, femSocket0Anchor, bodyL, bodyR, lastFourNodes[0], lastFourNodes[1], lastFourNodes[2]);
		physics::addTrackJoint(simpleFemSocket0);

		physics::BallSocket_FEM_Simple * simpleFemSocket1 = new physics::BallSocket_FEM_Simple;
		simpleFemSocket1->init(solver, 0.7f, 0.001f, femSocket1Anchor, bodyL, bodyR, lastFourNodes[1], lastFourNodes[3], lastFourNodes[2]);
		physics::addTrackJoint(simpleFemSocket1);

		Vec3 femSocket2Anchor = (3.0f * lastFourNodes[1]->m_pos + lastFourNodes[3]->m_pos + lastFourNodes[2]->m_pos) / 5.0f;
		physics::BallSocket_FEM_Simple * simpleFemSocket2 = new physics::BallSocket_FEM_Simple;
		simpleFemSocket2->init(solver, 0.7f, 0.001f, femSocket2Anchor, bodyL, bodyR, lastFourNodes[1], lastFourNodes[3], lastFourNodes[2]);
		physics::addTrackJoint(simpleFemSocket2);
	}
}

inline void sceneFEMChain(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	float mass = 2.0f;
	Mat33 inertiaMatrix;
	inertiaMatrix.identity();
	inertiaMatrix.t[0][0] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[1][1] = 1.0f / (0.1f * mass);
	inertiaMatrix.t[2][2] = 1.0f / (0.1f * mass);

	const bool finerCubeSegmentation = true;

	if (1)
	{
		const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
		const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
		const bool staticWalls = true;
		const float wallHeight = 5.0f;
		const float wallThickness = 0.25f;
		createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);

		FEMObject FEMCube;
		physics::addTranslationalNode(0.0f, Vec3C(0.0f, 0.0f, 0.0f), Vec3C(0.0f, 0.0f, 0.0f));
		uint32_t firstNodeIdx = (uint32_t)physics::g_nodes.size();

		uint32_t cubeSegWidth = 8, cubeSegHeight = 2, cubeSegDepth = 1;
		if (finerCubeSegmentation)
			buildCube12(FEMCube, Vec3C(-10.0f, 8.5f, 0.0f), Vec3C(4.0f, 1.0f, 0.5f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		else
			buildCube5(FEMCube, Vec3C(-10.0f, 8.5f, 0.0f), Vec3C(4.0f, 1.0f, 0.5f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		float totalMass = 5.0f;

		int size = (int)FEMCube.FEMNodes.size();
		// TODO: remember the offset for the cube in the solver
		for (int node = 0; node < size; ++node)
		{
			float mass = totalMass / (float)size;
			float invMass = (1.0f / mass);

			physics::NodeTranslational * curNode = physics::addTranslationalNode(invMass, FEMCube.FEMNodes[node], Vec3C(0.0f, 0.0f, 0.0f));
		}

		// Elasticity
		float youngModulus = 10.0f * 1.0f * 1* 1.0f * 1.0f;
		float poissonRatio = 0.35f;

		// Plasticity
		// c_creep [0..1/dt]
		// c_yeld - threshold for plastic strain
		// c_max - maximum plastic strain for FE
		float c_yeld = 0.05f, c_creep = 0.01f, c_max = 0.3f;
		float beta = 0.03f;										// Beta (C.O.)

		physics::FEMJoint * lastFEMJoint = nullptr;
		unsigned cntFE = 0;
		for (auto itTetra = FEMCube.FEMTetras.begin(), itTetraEnd = FEMCube.FEMTetras.end(); itTetra != itTetraEnd; ++itTetra)
		{
			++cntFE;

			physics::NodeTranslational * n0 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind1]);
			physics::NodeTranslational * n1 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind2]);
			physics::NodeTranslational * n2 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind3]);
			physics::NodeTranslational * n3 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind4]);

			physics::FEMJoint * newFEMJoint = new physics::FEMJoint;
			// Regularization = 0.0001
			newFEMJoint->init(
				solver,
				n0, n1, n2, n3,
				youngModulus, poissonRatio,
				c_yeld, c_creep, c_max,
				beta, 0.000005f
				);

			physics::addTrackJoint(newFEMJoint);
		}

		physics::NodeTranslational * n0;
		physics::NodeTranslational * n1;
		physics::NodeTranslational * n2;
		if (finerCubeSegmentation)
		{
			lastFEMJoint = static_cast<physics::FEMJoint *>(physics::g_joints[physics::g_joints.size()-8]);
			n0 = lastFEMJoint->getNode0();
			n1 = lastFEMJoint->getNode1();
			n2 = lastFEMJoint->getNode2();
		}
		else
		{
			lastFEMJoint = static_cast<physics::FEMJoint *>(physics::g_joints[physics::g_joints.size()-3]);
			n0 = lastFEMJoint->getNode3();
			n1 = lastFEMJoint->getNode1();
			n2 = lastFEMJoint->getNode2();
		}

		uint32_t tr0idx, tr1idx, tr2idx;
		tr0idx = n0->m_idx;
		tr1idx = n1->m_idx;
		tr2idx = n2->m_idx;

		uint32_t startNumNodes = (uint32_t)physics::g_nodes.size();

		int numLinks = 5;

		Vec3 anchorFEMRigidPos = (n0->m_pos + n1->m_pos + n2->m_pos) / 3.0f;
		anchorFEMRigidPos.x += 0.5f;

		float mass = totalMass / 10.0f;

		// Add rigid bodies
		for (int i = 0; i < numLinks + 1; ++i)
		{
			physics::addTranslationalNode((i == 0)?0.0f:(1.0f / mass), anchorFEMRigidPos + Vec3C(1.0f + numLinks*2.0f - i*2.0f, 0.0f, 0.0f), Vec3C());
			physics::addRotationalNode((i == 0)?Mat33().zero():inertiaMatrix, Quat().id(), Vec3C());
		}

		for (int i = 0; i < numLinks; ++i)
		{
			physics::BallSocket * newBallSocket = new physics::BallSocket;
			Vec3 anchorPoint =
				(static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i) * 2])->m_pos + 
				 static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i+1) * 2])->m_pos) * 0.5f;
			newBallSocket->init(
				0.5f, 0.01f, anchorPoint,
				static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i)*2]),
				static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i)*2+1]),
				static_cast<physics::NodeTranslational *>(physics::g_nodes[startNumNodes + (i+1)*2]),
				static_cast<physics::NodeRotational *>(physics::g_nodes[startNumNodes + (i+1)*2+1]));

			physics::addTrackJoint(newBallSocket);
		}

		//physics::NodeTranslational * lastNodeL = nullptr;
		//physics::NodeRotational * lastNodeR = nullptr;
		physics::NodeTranslational * lastNodeL = static_cast<physics::NodeTranslational *>(physics::g_nodes[physics::g_nodes.size() - 2]);
		physics::NodeRotational * lastNodeR = static_cast<physics::NodeRotational *>(physics::g_nodes[physics::g_nodes.size() - 1]);

#if 1
		physics::BallSocket_FEM * simpleFemSocket = new physics::BallSocket_FEM;
		simpleFemSocket->init(solver, 0.5f, 0.01f, anchorFEMRigidPos, lastNodeL, lastNodeR, n0, n1, n2);
#else
		physics::BallSocket_FEM_Simple * simpleFemSocket = new physics::BallSocket_FEM_Simple;
		simpleFemSocket->init(solver, 0.5f, 0.01f, anchorFEMRigidPos, lastNodeL, lastNodeR, n0, n1, n2);
#endif

		physics::addTrackJoint(simpleFemSocket);
	}
}

inline void sceneFEMPlayground(physics::SolverBase & solver)
{
	using namespace math;

	int startNumNodes = (int)physics::g_nodes.size();

	const bool finerCubeSegmentation = false;

	if (1)
	{
		bool staticLast = false;

		const Vec3 staticPos = Vec3C(0.0f, 0.0f, 0.0f);
		const Vec3 staticSize = Vec3C(15.0f, 0.5f, 15.0f);
		const bool staticWalls = true;
		const float wallHeight = 5.0f;
		const float wallThickness = 0.25f;
		if (!staticLast)
		{
			createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);
		}

		FEMObject FEMCube;
		physics::addTranslationalNode(0.0f, Vec3C(0.0f, 0.0f, 0.0f), Vec3C(0.0f, 0.0f, 0.0f));
		uint32_t firstNodeIdx = (uint32_t)physics::g_nodes.size();

		uint32_t cubeSegWidth = 2, cubeSegHeight = 2, cubeSegDepth = 2;
		if (finerCubeSegmentation)
			buildCube12(FEMCube, Vec3C(0.0f, 10.0f, 0.0f), Vec3C(1.0f, 1.0f, 1.0f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		else
			buildCube5(FEMCube, Vec3C(0.0f, 10.0f, 0.0f), Vec3C(1.0f, 1.0f, 1.0f), Mat33().identity(), cubeSegWidth, cubeSegHeight, cubeSegDepth);
		float totalMass = 2.0f;

		int size = (int)FEMCube.FEMNodes.size();
		// TODO: remember the offset for the cube in the solver
		for (int node = 0; node < size; ++node)
		{
			float mass = totalMass / (float)size;
			float invMass = (1.0f / mass);
			physics::NodeTranslational * curNode = physics::addTranslationalNode(invMass, FEMCube.FEMNodes[node], Vec3C(0.0f, 0.0f, 0.0f));

		}

		// Elasticity
		float youngModulus = 2.0f * 1.0f * 1* 1.0f * 1.0f;
		float poissonRatio = 0.35f;

		// Plasticity
		// c_creep [0..1/dt]
		// c_yeld - threshold for plastic strain
		// c_max - maximum plastic strain for FE
		float c_yeld = 0.05f, c_creep = 0.01f, c_max = 0.3f;
		float beta = 0.03f;										// Beta (C.O.)

		unsigned FE_cnt = 0;
		for (auto itTetra = FEMCube.FEMTetras.begin(), itTetraEnd = FEMCube.FEMTetras.end(); itTetra != itTetraEnd; ++itTetra)
		{
			++FE_cnt;

			physics::NodeTranslational * n0 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind1]);
			physics::NodeTranslational * n1 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind2]);
			physics::NodeTranslational * n2 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind3]);
			physics::NodeTranslational * n3 = static_cast<physics::NodeTranslational *>(physics::g_nodes[firstNodeIdx+itTetra->ind4]);

			physics::FEMJoint * newFEMJoint = new physics::FEMJoint;
			// Regularization = 0.0001
			newFEMJoint->init(
				solver,
				n0, n1, n2, n3,
				youngModulus, poissonRatio,
				c_yeld, c_creep, c_max,
				beta, 0.000005f
				);

			physics::addTrackJoint(newFEMJoint);
		}

		if (staticLast)
		{
			createStaticPlaypen(staticPos, staticSize, staticWalls, wallHeight, wallThickness);
		}
	}
}