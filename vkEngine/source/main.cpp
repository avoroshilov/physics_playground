#include "app.h"

#include <windows/timer.h>
#include <windows/window.h>
#include "vulkan/manager.h"
#include "camera.h"

#include "drawing_primitives.h"

// Physics simulation framework includes
#include <collision/convex_hull.h>
#include <collision/mpr.h>
#include <collision/convex_raycast.h>
#include <collision/contact_helpers.h>

#include <physics/solver_pgs.h>
#include <physics/solver_pgs_ordered.h>
#include <physics/solver_lcpcg.h>
#include <physics/joints.h>
#include <physics/hl_rigidbody.h>

#include "sim.h"
#include "physics_scenes.h"

// All physics-related code in this file is guarded with this preprocessor definition
#define ENABLE_PHYSICS		1

#define MOUSE_USE_RAWINPUT	1

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
	VkDebugReportFlagsEXT flags,
	VkDebugReportObjectTypeEXT objType,
	uint64_t obj,
	size_t location,
	int32_t code,
	const char * layerPrefix,
	const char * msg,
	void * userData
	)
{
	printf("[VAL] %s\n", msg);
	return VK_FALSE;
}

void resizeCallback(void * pUserData, int width, int height)
{
	if (pUserData)
	{
		CallbackData * callbackData = reinterpret_cast<CallbackData *>(pUserData);
		callbackData->app->onWindowResize(width, height);
	}
}
void chageFocusCallback(void * pUserData, bool isInFocus)
{
}
void chageActiveCallback(void * pUserData, bool isActive)
{
	printf("Active: %s\n", isActive ? "true" : "false");
	if (pUserData)
	{
		CallbackData * callbackData = reinterpret_cast<CallbackData *>(pUserData);
		if (isActive)
		{
			windows::hideMouse();
			windows::setMouseCoordinates(callbackData->mx, callbackData->my);
		}
		else
		{
			windows::showMouse();
		}
		callbackData->isActive = isActive;
	}
}
void keyStateCallback(void * pUserData, windows::KeyCode keyCode, windows::KeyState keyState)
{
	using namespace windows;

#define DBG_KEY_TESTING 0

#if (DBG_KEY_TESTING == 1)
	char * keyStatus = "UNKNOWN";
	if (keyState == KeyState::ePressed)
	{
		keyStatus = "pressed";
	}
	else if (keyState == KeyState::eHeldDown)
	{
		keyStatus = "held down";
	}
	else if (keyState == KeyState::eReleased)
	{
		keyStatus = "released";
	}
#endif

	if (!pUserData)
		return;

	CallbackData * callbackData = reinterpret_cast<CallbackData *>(pUserData);
	App * app = callbackData->app;

	if (!app)
		return;

	switch (keyCode)
	{
		case KeyCode::eEscape:
		{
			app->setIsExitting(true);
			break;
		}
		case KeyCode::eF12:
		{
			if (keyState == KeyState::ePressed)
			{
				app->requestCapture(true);
			}
			break;
		}
		case (KeyCode)'E':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->reqDropBox |= true;

			break;
		}
		case (KeyCode)'Q':
		{
			if (keyState == KeyState::ePressed)
			{
				callbackData->isPaused = !callbackData->isPaused;
			}
			break;
		}
		case (KeyCode)'P':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
			{
				callbackData->isAnimStep = true;
				callbackData->isPaused = false;
			}
			break;
		}
		case (KeyCode)'W':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eForward;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eForward;

			break;
		}
		case (KeyCode)'S':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eBackward;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eBackward;

			break;
		}
		case (KeyCode)'A':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eLeft;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eLeft;

			break;
		}
		case (KeyCode)'D':
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eRight;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eRight;

			break;
		}
		case KeyCode::ePgDown:
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eDown;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eDown;

			break;
		}
		case KeyCode::ePgUp:
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eUp;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eUp;

			break;
		}
		case KeyCode::eLShift:
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eAccel;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eAccel;

			break;
		}
		case KeyCode::eLCtrl:
		{
			if (keyState == KeyState::ePressed || keyState == KeyState::eHeldDown)
				callbackData->movementFlags |= (uint32_t)CallbackData::MovementKindBits::eDeccel;
			else
				callbackData->movementFlags &= ~(uint32_t)CallbackData::MovementKindBits::eDeccel;

			break;
		}
#if (DBG_KEY_TESTING == 1)
		case KeyCode::eEnter:
		{
			printf("Enter %s\n", keyStatus);
			break;
		}
		case KeyCode::eRCtrl:
		{
			printf("RCtrl %s\n", keyStatus);
			break;
		}
		case KeyCode::eRShift:
		{
			printf("RShift %s\n", keyStatus);
			break;
		}
		case KeyCode::eLAlt:
		{
			printf("LAlt %s\n", keyStatus);
			break;
		}
		case KeyCode::eRAlt:
		{
			printf("RAlt %s\n", keyStatus);
			break;
		}
		default:
		{
			printf("Key %d %s\n", (int)keyCode, keyStatus);
			break;
		}
#endif
	}
}

void mouseEventCallback(void * pUserData, windows::MouseEvent mouseEvent)
{
	if (!pUserData)
		return;

	CallbackData * callbackData = reinterpret_cast<CallbackData *>(pUserData);
	App * app = callbackData->app;

	if (!app)
		return;

	if (mouseEvent.type == windows::MouseEvent::Type::eMove)
	{
		callbackData->dmx += mouseEvent.dX;
		callbackData->dmy += mouseEvent.dY;
	}
	else if (mouseEvent.type == windows::MouseEvent::Type::eLBDown)
	{
		callbackData->lmbState = 1;
	}
	else if (mouseEvent.type == windows::MouseEvent::Type::eLBUp)
	{
		callbackData->lmbState = 0;
	}
	else if (mouseEvent.type == windows::MouseEvent::Type::eRBDown)
	{
		callbackData->mouseMode = (CallbackData::MouseMode)((uint32_t)callbackData->mouseMode + 1);
		if (callbackData->mouseMode == CallbackData::MouseMode::eNUM_ENTRIES)
			callbackData->mouseMode = CallbackData::MouseMode::eStartingMode;
	}
	else if (mouseEvent.type == windows::MouseEvent::Type::eWheel)
	{
		callbackData->dwheel += mouseEvent.dX;
	}
}

class CubeMesh
{
	vulkan::Mesh * m_mesh = nullptr;

public:

	void init(vulkan::Mesh * mesh, const math::Vec3 & offset, const math::Vec3 & cubeSize, const math::Vec4 & color)
	{
		using namespace math;

		m_mesh = mesh;

		const size_t numVertices = 24;
		std::vector<vulkan::Vertex> vertices;
		vertices.resize(numVertices);

		const size_t numIndices = 36;
		std::vector<uint16_t> indices;
		indices.resize(numIndices);

		m_mesh->initBuffers(
			vulkan::BufferUsage::eStatic, (uint32_t)numVertices, sizeof(vulkan::Vertex),
			vulkan::BufferUsage::eStatic, (uint32_t)numIndices
			);

		Mat44 modelMatrix;
		modelMatrix.identity();
		modelMatrix.fillTranslation(offset);
		m_mesh->setModelMatrix(modelMatrix);

		auto finalizeFace = [&vertices](int vtxOffset, const Vec3 & offset, const Vec3 & normal, const Vec4 & faceColor)
		{
			vertices[vtxOffset+0].pos += offset;
			vertices[vtxOffset+1].pos += offset;
			vertices[vtxOffset+2].pos += offset;
			vertices[vtxOffset+3].pos += offset;

			vertices[vtxOffset+0].nrm = normal;
			vertices[vtxOffset+1].nrm = normal;
			vertices[vtxOffset+2].nrm = normal;
			vertices[vtxOffset+3].nrm = normal;

			vertices[vtxOffset+0].col = faceColor;
			vertices[vtxOffset+1].col = faceColor;
			vertices[vtxOffset+2].col = faceColor;
			vertices[vtxOffset+3].col = faceColor;

			vertices[vtxOffset+0].tc = Vec2C( 0.0f,  0.0f);
			vertices[vtxOffset+1].tc = Vec2C( 1.0f,  0.0f);
			vertices[vtxOffset+2].tc = Vec2C( 1.0f,  1.0f);
			vertices[vtxOffset+3].tc = Vec2C( 0.0f,  1.0f);
		};

		Vec3 vecOffset = Vec3C(0.0f, 0.0f, 0.0f);

		// Vertices
		// Front
		vertices[ 0].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
		vertices[ 1].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[ 2].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[ 3].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
		finalizeFace( 0, vecOffset, Vec3C( 0.0f,  0.0f,  1.0f), color);

		// Back
		vertices[ 4].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
		vertices[ 5].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
		vertices[ 6].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
		vertices[ 7].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
		finalizeFace( 4, vecOffset, Vec3C( 0.0f,  0.0f, -1.0f), color);

		// Left
		vertices[ 8].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
		vertices[ 9].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
		vertices[10].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[11].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
		finalizeFace( 8, vecOffset, Vec3C(-1.0f,  0.0f,  0.0f), color);

		// Right
		vertices[12].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
		vertices[13].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
		vertices[14].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[15].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
		finalizeFace(12, vecOffset, Vec3C( 1.0f,  0.0f,  0.0f), color);

		// Top
		vertices[16].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
		vertices[17].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[18].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
		vertices[19].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
		finalizeFace(16, vecOffset, Vec3C( 0.0f,  1.0f,  0.0f), color);

		// Bottom
		vertices[20].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
		vertices[21].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
		vertices[22].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
		vertices[23].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
		finalizeFace(20, vecOffset, Vec3C( 0.0f, -1.0f,  0.0f), color);

		// Indices
		int vtxOffset;
		int idxOffset;

		// Front
		vtxOffset =  0;
		idxOffset =  0;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 1; indices[idxOffset+2] = vtxOffset + 2;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 2; indices[idxOffset+5] = vtxOffset + 3;

		// Back
		vtxOffset =  4;
		idxOffset =  6;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 2; indices[idxOffset+2] = vtxOffset + 1;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 3; indices[idxOffset+5] = vtxOffset + 2;

		// Left
		vtxOffset =  8;
		idxOffset = 12;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 2; indices[idxOffset+2] = vtxOffset + 1;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 3; indices[idxOffset+5] = vtxOffset + 2;

		// Right
		vtxOffset = 12;
		idxOffset = 18;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 1; indices[idxOffset+2] = vtxOffset + 2;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 2; indices[idxOffset+5] = vtxOffset + 3;

		// Top
		vtxOffset = 16;
		idxOffset = 24;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 2; indices[idxOffset+2] = vtxOffset + 1;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 3; indices[idxOffset+5] = vtxOffset + 2;

		// Bottom
		vtxOffset = 20;
		idxOffset = 30;
		indices[idxOffset+0] = vtxOffset + 0; indices[idxOffset+1] = vtxOffset + 1; indices[idxOffset+2] = vtxOffset + 2;
		indices[idxOffset+3] = vtxOffset + 0; indices[idxOffset+4] = vtxOffset + 2; indices[idxOffset+5] = vtxOffset + 3;

		m_mesh->updateVertexBuffer(vertices.data());
		m_mesh->updateIndexBuffer(indices.data());
	}
};

struct ExtDrawLine
{
	math::Vec3 p0;
	math::Vec3 p1;
	math::Vec4 c0;
	math::Vec4 c1;
};
struct ExtDrawPoint
{
	math::Vec3 p;
	math::Vec4 c;
};

std::vector<ExtDrawLine> g_simLines;
std::vector<ExtDrawPoint> g_simPoints;

void simResetDrawing()
{
	g_simLines.resize(0);
	g_simPoints.resize(0);
}
void simDrawLine(const math::Vec3 & p0, const math::Vec3 & p1, const math::Vec4 & c0, const math::Vec4 & c1)
{
	g_simLines.push_back({ p0, p1, c0, c1 });
}
void simDrawPoint(const math::Vec3 & p, const math::Vec4 & c)
{
	g_simPoints.push_back({ p, c });
}

void projPerspective(float fovRad, float aspect, float zNear, float zFar, float width, float height, math::Mat44 * projection)
{
	projection->zero();

	math::Vec2 scale;
	scale.x = width * 1.0f / tanf(fovRad / 2);
	scale.y = height * aspect * scale.x;

	float zDiff = zNear - zFar;
	float m[] = {
		scale.x, 0, 0, 0,
		0, scale.y, 0, 0,
		0, 0, (zFar + zNear) / zDiff, 2*zFar*zNear / zDiff,
		0, 0, -1, 0 
	};    
	memcpy(projection->m, m, sizeof(float)*16);
}

int main()
{
	using namespace windows;
	using namespace math;

	Timer perfTimer;

	Window window;
	window.setParameters(800, 600, Window::Kind::eWindowed);
	window.init();

	MSG msg;

	physics::g_simCallbacks.resetDrawingFn = simResetDrawing;
	physics::g_simCallbacks.drawLineFn = simDrawLine;
	physics::g_simCallbacks.drawPointFn = simDrawPoint;

	vulkan::Wrapper renderManager;
	renderManager.setDebugCallback(debugCallback);

	App sampleApp;
	sampleApp.setRenderManager(&renderManager);
	sampleApp.init(window.getHWnd(), window.getWidth(), window.getHeight());
	sampleApp.setWindowTitle(L"Vulkan app");

	CallbackData callbackData = {};
	callbackData.app = &sampleApp;
	callbackData.movementFlags = 0;
	callbackData.isActive = true;
	callbackData.isPaused = false;//true;
	callbackData.isAnimStep = false;
	callbackData.reqDropBox = false;
	getMouseCoordinates(&callbackData.mx, &callbackData.my);
	callbackData.dmx = 0;
	callbackData.dmy = 0;
	callbackData.dwheel = 0;
	callbackData.mouseMode = CallbackData::MouseMode::eStartingMode;
	callbackData.lmbState = 0;
	hideMouse();

	setUserDataPointer(&callbackData);
	setResizeCallback(resizeCallback);
	setChangeFocusCallback(chageFocusCallback);
	setChangeActiveCallback(chageActiveCallback);
	setKeyStateCallback(keyStateCallback);
	setMouseEventCallback(mouseEventCallback);

	Camera mainCamera;
	mainCamera.setPosition(Vec3C(0.0f, 2.0f, 10.0f));

#if 0
	const Vec3 basisOffset = Vec3C(1.0f, 0.0f, 0.0f);
	const Vec2 basisHalfSize = Vec2C(0.5f, 0.02f);
	CubeMesh basisMesh[3];
	basisMesh[0].init(renderManager.createMesh(), basisOffset+Vec3C(0.5f-basisHalfSize.y, 0.0f, 0.0f), Vec3C(basisHalfSize.x, basisHalfSize.y, basisHalfSize.y), Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
	basisMesh[1].init(renderManager.createMesh(), basisOffset+Vec3C(0.0f, 0.5f-basisHalfSize.y, 0.0f), Vec3C(basisHalfSize.y, basisHalfSize.x, basisHalfSize.y), Vec4C(0.0f, 1.0f, 0.0f, 1.0f));
	basisMesh[2].init(renderManager.createMesh(), basisOffset+Vec3C(0.0f, 0.0f, 0.5f-basisHalfSize.y), Vec3C(basisHalfSize.y, basisHalfSize.y, basisHalfSize.x), Vec4C(0.0f, 0.0f, 1.0f, 1.0f));

	const bool renderUnitCubeMesh = false;
	if (renderUnitCubeMesh)
	{
		CubeMesh unitCubeMesh;
		unitCubeMesh.init(renderManager.createMesh(), Vec3C(.0f, .0f, .0f), Vec3C(.4f, .4f, .4f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
	}

	const bool renderShadowMapBiasTestingMeshes = true;
	if (renderShadowMapBiasTestingMeshes)
	{
		CubeMesh shadowTestMesh01;
		shadowTestMesh01.init(renderManager.createMesh(), Vec3C(-2.5f, -0.5f, 0.0f), Vec3C(0.1f, 1.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
		CubeMesh shadowTestMesh02;
		shadowTestMesh02.init(renderManager.createMesh(), Vec3C( 0.0f, -0.5f, 2.0f), Vec3C(0.05f, 1.0f, 0.05f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
	}

	const Vec3 floorSize = Vec3C(10.0f, 0.5f, 10.0f);
	CubeMesh floorMesh;
	floorMesh.init(renderManager.createMesh(), Vec3C(0.0f, -2.0f, 0.0f), floorSize, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
#endif

	// Light setup
	renderManager.setLightPerspProjParams(PI * 0.5f, 1.0f, 0.1f, 100.0f, 1.0f, 1.0f);

#if (ENABLE_PHYSICS == 1)

	// FEM stress test similar visual stiffness: LCPCG 36ms, 40iter; PGS 100ms, 260iter;
	//physics::SolverPGS FEMTestSolver(20, 1e-10f);
	//physics::SolverPGSOrdered FEMTestSolver(20, 1e-10f);
	physics::SolverLCPCG FEMTestSolver(20, 1e-10f);

	FEMTestSolver.setInitialGuessCutFlag(false);
	FEMTestSolver.setInitialGuessCutThreshold(0.0f);

	float pickingDistance = 0.0f;
	physics::JointBase * pickingJoint = nullptr;

	physics::initPhysics();

	// Scene selection
#if 0
	sceneSample(FEMTestSolver);
#elif 0
	sceneSimpleBallsocket(FEMTestSolver);
#elif 0
	sceneBallSocketCoMTest(FEMTestSolver);
#elif 0
	sceneBoxContact(FEMTestSolver);
#elif 0
	sceneCompoundBodyTest(FEMTestSolver);
#elif 0
	sceneSpinningTest(FEMTestSolver);
#elif 0
	sceneGyro(FEMTestSolver);
#elif 1
	// Collision test
	sceneStackCollisionTest(FEMTestSolver);
#elif 0
	sceneStackCollisionTestMassive(FEMTestSolver);
#elif 0
	sceneShapesCollisionTest(FEMTestSolver);
#elif 0
	// For local mass scale options, check
	//	const bool applyLocalMassScale = true;
	//	const float localMassScaleRatio = 1.2f;
	// in the test setup function

	// For CG-based solver, increased number of iterations recommended (base iter 40+)

	sceneJointsTest(FEMTestSolver);
#elif 0
	sceneFEMBeam(FEMTestSolver);
#elif 0
	sceneFEMBeamSwirl(FEMTestSolver);
#elif 0
	sceneFEMChain(FEMTestSolver);
#elif 0
	sceneFEMPlayground(FEMTestSolver);
#endif

#endif

	vulkan::LinePoint auxP0, auxP1;
	auxP0.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);
	auxP1.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

	auto drawPoint = [&renderManager, &auxP0, &auxP1](const Vec3 & pos, const Vec4C & color = Vec4C(1.0f, 0.0f, 0.0f, 1.0f))
	{
		auxP0.col = color;
		auxP1.col = color;

		const Vec3 size = Vec3C(0.1f, 0.1f, 0.1f);
		auxP0.pos = pos - Vec3C(size.x, 0.0f, 0.0f);
		auxP1.pos = pos + Vec3C(size.x, 0.0f, 0.0f);
		renderManager.m_debugLines.push_back(auxP0);
		renderManager.m_debugLines.push_back(auxP1);
		auxP0.pos = pos - Vec3C(0.0f, size.y, 0.0f);
		auxP1.pos = pos + Vec3C(0.0f, size.y, 0.0f);
		renderManager.m_debugLines.push_back(auxP0);
		renderManager.m_debugLines.push_back(auxP1);
		auxP0.pos = pos - Vec3C(0.0f, 0.0f, size.z);
		auxP1.pos = pos + Vec3C(0.0f, 0.0f, size.z);
		renderManager.m_debugLines.push_back(auxP0);
		renderManager.m_debugLines.push_back(auxP1);
	};

	auto drawLine = [&renderManager, &auxP0, &auxP1](const Vec3 & pos0, const Vec3 & pos1, const Vec4 & color = Vec4C(1.0f, 1.0f, 0.0f, 1.0f))
	{
		auxP0.col = color;
		auxP1.col = color;

		auxP0.pos = pos0;
		auxP1.pos = pos1;
		renderManager.m_debugLines.push_back(auxP0);
		renderManager.m_debugLines.push_back(auxP1);
	};

	auto drawLineAux = [&renderManager](const vulkan::LinePoint & p0, const vulkan::LinePoint & p1)
	{
		renderManager.m_debugLines.push_back(p0);
		renderManager.m_debugLines.push_back(p1);
	};

	// Preparing rendering arrays
	// In this debug-oriented playground, the rendering is happening using the debug
	//	renderering mode, which is similar to immediate mode rendering in older graphics
	//	APIs, and this method is not exactly the best performance-wise.
	// TODO: use CubeMesh and the like to render physics objects
	std::vector<vulkan::Vertex> unitBoxVertices;
	std::vector<vulkan::Vertex> unitSphereVertices;
	std::vector<vulkan::Vertex> unitHemiSphereVertices;
	std::vector<vulkan::Vertex> unitCylinderVertices;
	std::vector<vulkan::Vertex> unitConeVertices;
	std::vector<vulkan::Vertex> unitDiskVertices;

	const Vec4 whiteColor = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);
	buildBoxVertices(&unitBoxVertices, Mat34().identity(), Vec3C(1.0f, 1.0f, 1.0f), whiteColor);
	buildSphereVertices(&unitSphereVertices, Mat34().identity(), 1.0f, whiteColor);
	buildHemiSphereVertices(&unitHemiSphereVertices, Mat34().identity(), 1.0f, whiteColor);
	buildCylinderVertices(&unitCylinderVertices, Mat34().identity(), 1.0f, 1.0f, whiteColor);
	buildConeVertices(&unitConeVertices, Mat34().identity(), 1.0f, 1.0f, whiteColor);
	buildDiskVertices(&unitDiskVertices, Mat34().identity(), 1.0f, whiteColor, true);

	// Scale could be incorporated into transform, but for the sake of convenience
	//	scale is passed as a separate argument
	auto drawVerticesScaled = [&renderManager](const std::vector<vulkan::Vertex> & inVertices, const Mat34 & transform, const Vec3 & scale)
	{
		const size_t vtxNum = inVertices.size();
		const vulkan::Vertex * vertices = inVertices.data();

		const size_t numStartVertices = renderManager.m_debugTris.size();
		renderManager.m_debugTris.resize(numStartVertices + inVertices.size());
		vulkan::Vertex * debugTrisVertices = &renderManager.m_debugTris[numStartVertices];

		vulkan::Vertex curVertex;
		for (size_t vi = 0; vi < vtxNum; ++vi)
		{
			curVertex = vertices[vi];
			curVertex.pos = transform * Vec3C(scale.x * curVertex.pos.x, scale.y * curVertex.pos.y, scale.z * curVertex.pos.z);
			curVertex.nrm = transform.rotateCopy(curVertex.nrm);
			debugTrisVertices[vi] = curVertex;
		}
	};

	double physicsTime = 0.0;
	double accumTimeMS = 0.0;
	int accumFrames = 0;
	perfTimer.start();
	double dtMS = 0.0;

	int internalMouseX = window.getWidth() / 2, internalMouseY = window.getHeight() / 2;

	bool isRunning = true;
	while (isRunning)
	{
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			if (msg.message == WM_QUIT)
			{
				isRunning = false;
				break;
			}
		}

		if (sampleApp.getIsExitting())
		{
			isRunning = false;
		}

		if (!isRunning)
		{
			break;
		}

		if (callbackData.isActive)
		{
			int nmx, nmy;
			getMouseCoordinates(&nmx, &nmy);
			setMouseCoordinates(callbackData.mx, callbackData.my);

			float sens = 0.01f;
			float movementSpeed = 0.005f;
			Vec3 posOffset = Vec3C(0.0f, 0.0f, 0.0f);

			const float movementAccelFactor = 5.0f;
			const float movementDeccelFactor = 0.2f;
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eAccel)
			{
				movementSpeed *= movementAccelFactor;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eDeccel)
			{
				movementSpeed *= movementDeccelFactor;
			}
		
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eForward)
			{
				posOffset.z -= movementSpeed;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eBackward)
			{
				posOffset.z += movementSpeed;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eLeft)
			{
				posOffset.x -= movementSpeed;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eRight)
			{
				posOffset.x += movementSpeed;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eDown)
			{
				posOffset.y -= movementSpeed;
			}
			if (callbackData.movementFlags & (uint32_t)CallbackData::MovementKindBits::eUp)
			{
				posOffset.y += movementSpeed;
			}

			if (callbackData.dwheel != 0)
			{
				float fovRad = renderManager.getViewFOV();
				fovRad += callbackData.dwheel * 0.1f;
				fovRad = clamp(fovRad, PI / 4.0f, PI / 1.5f);
				renderManager.setViewFOV(fovRad);
			}

#if (MOUSE_USE_RAWINPUT == 1)
			int dMouseX = callbackData.dmx;
			int dMouseY = callbackData.dmy;

			callbackData.dmx = 0;
			callbackData.dmy = 0;

			callbackData.dwheel = 0;
#else
			int dMouseX = nmx - callbackData.mx;
			int dMouseY = nmy - callbackData.my;
#endif

			if (callbackData.mouseMode == CallbackData::MouseMode::ePicking)
			{
				internalMouseX += dMouseX;
				if (internalMouseX < 0)
					internalMouseX = 0;
				if (internalMouseX > (int)window.getWidth()-1)
					internalMouseX = (int)window.getWidth()-1;
				internalMouseY -= dMouseY;
				if (internalMouseY < 0)
					internalMouseY = 0;
				if (internalMouseY > (int)window.getHeight()-1)
					internalMouseY = (int)window.getHeight()-1;

				// Mouse used for picking, do not propagate these values to the camera
				dMouseX = 0;
				dMouseY = 0;
			}
			mainCamera.update((float)dtMS * posOffset, sens*dMouseX, sens*dMouseY);
			mainCamera.fillMatrix(&renderManager.getViewMatrix());
		}

		{
			Mat44 lightView, lightProj;

			const Vec3 lightPos = Vec3C(10.0f * sinf((float)sampleApp.getElapsedTime() * 0.001f), 3.0f, 3.0f);
			const Vec3 lightDir = Vec3C(0.0f, 0.0f, 0.0f);

			Vec3 lightRight, lightUp, lightViewDir;
			lightViewDir = -(lightDir - lightPos).normalize();
			lightRight = Vec3C(0.0f, 1.0f, 0.0f).cross(lightViewDir).normalize();
			lightUp = lightViewDir.cross(lightRight).normalize();
			lightView.setBasis0(lightRight);
			lightView.setBasis1(lightUp);
			lightView.setBasis2(lightViewDir);
			lightView.setBasis3(lightPos);
			lightView = lightView.invertRTCopy();

			renderManager.setLightViewMatrix(lightView);

			const bool renderLightFrustum = true;
			if (renderLightFrustum)
			{
				float fovRad, aspect, zNear, zFar, width, height;
				renderManager.getLightPerspProjParams(&fovRad, &aspect, &zNear, &zFar, &width, &height);
				projPerspective(fovRad, aspect, zNear, zFar, width, height, &lightProj);

				Vec3 meshOBBMin = Vec3C(-1.0f, -1.0f, -1.0f), meshOBBMax = Vec3C(1.0f, 1.0f, 1.0f);

				Mat44 lightMatrixInv = lightProj * lightView;
				lightMatrixInv.invert();

				Vec3 meshOBB[8];
				meshOBB[0] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMin.x, meshOBBMin.y, meshOBBMin.z));
				meshOBB[1] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMax.x, meshOBBMin.y, meshOBBMin.z));
				meshOBB[2] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMax.x, meshOBBMax.y, meshOBBMin.z));
				meshOBB[3] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMin.x, meshOBBMax.y, meshOBBMin.z));
				meshOBB[4] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMin.x, meshOBBMin.y, meshOBBMax.z));
				meshOBB[5] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMax.x, meshOBBMin.y, meshOBBMax.z));
				meshOBB[6] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMax.x, meshOBBMax.y, meshOBBMax.z));
				meshOBB[7] = lightMatrixInv.homogTransformCopy(Vec3C(meshOBBMin.x, meshOBBMax.y, meshOBBMax.z));

				vulkan::LinePoint lp0, lp1;
				lp0.col = Vec4C(1.0f, 1.0f, 0.0f, 1.0f);
				lp1.col = Vec4C(1.0f, 1.0f, 0.0f, 1.0f);

				auto addLine = [&renderManager, &lp0, &lp1](const Vec3 & pos0, const Vec3 & pos1)
				{
					lp0.pos = pos0;
					lp1.pos = pos1;
					renderManager.m_debugLines.push_back(lp0);
					renderManager.m_debugLines.push_back(lp1);
				};

				{
					addLine(meshOBB[0], meshOBB[1]);
					addLine(meshOBB[1], meshOBB[2]);
					addLine(meshOBB[2], meshOBB[3]);
					addLine(meshOBB[3], meshOBB[0]);

					addLine(meshOBB[4], meshOBB[5]);
					addLine(meshOBB[5], meshOBB[6]);
					addLine(meshOBB[6], meshOBB[7]);
					addLine(meshOBB[7], meshOBB[4]);

					addLine(meshOBB[0], meshOBB[4]);
					addLine(meshOBB[1], meshOBB[5]);
					addLine(meshOBB[2], meshOBB[6]);
					addLine(meshOBB[3], meshOBB[7]);
				}
			}
		}

#if (ENABLE_PHYSICS == 1)
		// UPDATE
		const float solver_dt = 0.016f;
		if (physicsTime > solver_dt || callbackData.isAnimStep)
		{
			// The updateSim function includes calls to all the default high-level functions of the
			//	physics simulation code, such as collision detection, cntact cache matching, dynamics
			//	simulation, etc. They all operate on concepts defined in the Core physics framework,
			//	and could easily be modified to e.g. inject custom contacts, or broadphase candidates,
			//	or change the MLCP solve scheme.
			physics::updateSim(&FEMTestSolver, callbackData.isPaused ? 0.0f : solver_dt);
			// Do not accumulate time
			physicsTime = 0.0;
		}

		// DRAW
		// Current physics joint rendering code is weak - it uses lower level joint blocks, and hence
		//	cannot properly anchor hinge axis and stuff like that.
		// TODO: introduce higher level joints that wrap several low level blocks.
		for (size_t iJoint = 0, iJointEnd = physics::g_joints.size(); iJoint < iJointEnd; ++iJoint)
		{
			physics::JointBase * curJoint = physics::g_joints[iJoint];
			physics::JointBase::Type jointType = curJoint->getType();

			if (jointType == physics::JointBase::Type::eBallSocket)
			{
				physics::BallSocket * curBallSocket = static_cast<physics::BallSocket *>(curJoint);

				Mat34 rotMatrix;
				rotMatrix.identity();

				Vec3 b0_AnchorBS, b1_AnchorBS;

				Vec3 b0_comPos, b1_comPos;
				Vec3 b0_originPos, b1_originPos;
				Vec3 b0_AnchorWS, b1_AnchorWS;

				curBallSocket->getAnchorPoints_BS(b0_AnchorBS, b1_AnchorBS);

				b0_comPos = curBallSocket->getBody0L()->m_pos;

				physics::NodeRotational * body0Rot = curBallSocket->getBody0R();
				rotMatrix = math::Quat::toMatrix33(body0Rot->m_rot);

				b0_originPos = b0_comPos + rotMatrix * (-body0Rot->m_com);
				b0_AnchorWS = b0_comPos + rotMatrix * (b0_AnchorBS - body0Rot->m_com);

				drawLine(b0_originPos, b0_AnchorWS);
				drawPoint(b0_AnchorWS);

				physics::NodeTranslational * body1L = curBallSocket->getBody1L();
				physics::NodeRotational * body1Rot = curBallSocket->getBody1R();
				if (body1L)
				{
					b1_comPos = body1L->m_pos;
					b1_originPos = b1_comPos;
				}
				if (body1Rot)
				{
					rotMatrix = math::Quat::toMatrix33(body1Rot->m_rot);

					b1_originPos = b1_comPos + rotMatrix * (-body1Rot->m_com);
					b1_AnchorWS = b1_comPos + rotMatrix * (b1_AnchorBS - body1Rot->m_com);

					drawLine(b1_originPos, b1_AnchorWS);
					drawPoint(b1_AnchorWS);
				}
			}
			if (jointType == physics::JointBase::Type::eSlider)
			{
				physics::Slider * curSlider = static_cast<physics::Slider *>(curJoint);

				const float axisHalfLen = 1.0f;

				Vec3 wAxis0 = curSlider->getWorldAxis0();
				drawLine(curSlider->getBody0L()->m_pos - axisHalfLen*wAxis0, curSlider->getBody0L()->m_pos + axisHalfLen*wAxis0, Vec4C(0.0f, 1.0f, 1.0f, 1.0f));

				physics::NodeTranslational * body1L = curSlider->getBody1L();
				physics::NodeRotational * body1Rot = curSlider->getBody1R();
				if (body1L)
				{
					Vec3 wAxis1 = curSlider->getWorldAxis1();
					drawLine(body1L->m_pos - axisHalfLen*wAxis1, body1L->m_pos + axisHalfLen*wAxis1, Vec4C(0.0f, 1.0f, 1.0f, 1.0f));
				}
			}
			else if (jointType == physics::JointBase::Type::ePlane)
			{
				physics::PlaneConstraint * curPlaneConstraint = static_cast<physics::PlaneConstraint *>(curJoint);

				if (!curPlaneConstraint->m_bActive) continue;

				Vec3 NodePos = curPlaneConstraint->m_bodyL->m_pos;
				Vec3 plLambda = curPlaneConstraint->m_PlaneNormal * curPlaneConstraint->getLambda(0);

				//float color = fabsf(itPlaneJ->m_Violation);
				float color = fabsf(FEMTestSolver.m_resid[curPlaneConstraint->m_startIdx]) * 1000.0f;

				drawLine(NodePos, NodePos+plLambda, Vec4C(0.7f, color * 0.25f, color * 0.5f, 1.0f));
			}
			else if (jointType == physics::JointBase::Type::eFEM)
			{
				physics::FEMJoint * curFEMJoint = static_cast<physics::FEMJoint *>(curJoint);
#if 0
				drawLine(curFEMJoint->getNode0()->m_pos, curFEMJoint->getNode1()->m_pos, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
				drawLine(curFEMJoint->getNode0()->m_pos, curFEMJoint->getNode2()->m_pos, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
				drawLine(curFEMJoint->getNode0()->m_pos, curFEMJoint->getNode3()->m_pos, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
				//drawLine(curFEMJoint->getNode0()->m_pos, curFEMJoint->getNode1()->m_pos, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
#else
				vulkan::Vertex faceVtx;

				float FEstrain = curFEMJoint->m_strainNorm * 10.0f;
				faceVtx.col = Vec4C(
					clamp(1.0f + FEstrain, 0.0f, 1.0f),
					clamp(1.0f - FEstrain * 0.5f, 0.0f, 1.0f),
					clamp(1.0f - FEstrain * 0.5f, 0.0f, 1.0f),
					1.0f
					);

				physics::NodeTranslational * n0 = curFEMJoint->getNode0();
				physics::NodeTranslational * n1 = curFEMJoint->getNode1();
				physics::NodeTranslational * n2 = curFEMJoint->getNode2();
				physics::NodeTranslational * n3 = curFEMJoint->getNode3();

				auto addTetraFace = [&renderManager](
					vulkan::Vertex & faceVtx,
					const physics::NodeTranslational * n0,
					const physics::NodeTranslational * n1,
					const physics::NodeTranslational * n2
					)
				{
					Vec3 normal = (n1->m_pos - n0->m_pos).cross(n2->m_pos - n0->m_pos).getNormalized();

					faceVtx.tc = Vec2C(0.0f, 0.0f);
					faceVtx.pos = n0->m_pos;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
					faceVtx.tc = Vec2C(1.0f, 0.0f);
					faceVtx.pos = n1->m_pos;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
					faceVtx.tc = Vec2C(0.0f, 1.0f);
					faceVtx.pos = n2->m_pos;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
				};

				addTetraFace(faceVtx, n0, n2, n1);
				addTetraFace(faceVtx, n0, n1, n3);
				addTetraFace(faceVtx, n0, n3, n2);
				addTetraFace(faceVtx, n1, n2, n3);
#endif
			}
			else if (jointType == physics::JointBase::Type::eFEM_BallSocket)
			{
				physics::BallSocket_FEM * curBSFEMJoint = static_cast<physics::BallSocket_FEM *>(curJoint);

				Vec3 FEMAnchor = curBSFEMJoint->getFEMAnchor();
				drawLine(curBSFEMJoint->getNode0()->m_pos, FEMAnchor, Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
				drawLine(curBSFEMJoint->getNode1()->m_pos, FEMAnchor, Vec4C(0.0f, 1.0f, 0.0f, 1.0f));
				drawLine(curBSFEMJoint->getNode2()->m_pos, FEMAnchor, Vec4C(0.0f, 0.0f, 1.0f, 1.0f));

				Vec3 bodyAnchor = physics::transformOriginToWorld(curBSFEMJoint->getBodyL(), curBSFEMJoint->getBodyR(), curBSFEMJoint->getBodyAnchor());
				drawLine(curBSFEMJoint->getBodyL()->m_pos, bodyAnchor, Vec4C(1.0f, 1.0f, 0.0f, 1.0f));
			}
			else if (jointType == physics::JointBase::Type::eFEM_BallSocketSimple)
			{
				physics::BallSocket_FEM_Simple * curBSFEMJoint = static_cast<physics::BallSocket_FEM_Simple *>(curJoint);

				Vec3 FEMAnchor = curBSFEMJoint->getFEMAnchor();
				drawLine(curBSFEMJoint->getNode0()->m_pos, FEMAnchor, Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
				drawLine(curBSFEMJoint->getNode1()->m_pos, FEMAnchor, Vec4C(0.0f, 1.0f, 0.0f, 1.0f));
				drawLine(curBSFEMJoint->getNode2()->m_pos, FEMAnchor, Vec4C(0.0f, 0.0f, 1.0f, 1.0f));

				Vec3 bodyAnchor = physics::transformOriginToWorld(curBSFEMJoint->getBodyL(), curBSFEMJoint->getBodyR(), curBSFEMJoint->getBodyAnchor());
				if (curBSFEMJoint->getBodyL())
					drawLine(curBSFEMJoint->getBodyL()->m_pos, bodyAnchor, Vec4C(1.0f, 1.0f, 0.0f, 1.0f));
			}
		}

		// Draw bodies - just a simple RGB basis at the CoM of the body (linear node position)
		// Note: the CoM position may be different from the point where user originally placed
		//	rigid body, to transform between the world and original design spaces, use
		//	physics::transformWorldToOrigin/transformOriginToWorld
		Vec3 wsX, wsY, wsZ;
		for (size_t iNode = 0, iNodeEnd = physics::g_nodes.size() - 1; iNode < iNodeEnd; ++iNode)
		{
			// We need linear nodes, sequenced by rotational ones [full rigid body]
			if (physics::g_nodes[iNode]->getType() != physics::Node::Type::eTranslational || physics::g_nodes[iNode+1]->getType() != physics::Node::Type::eRotational)
				continue;

			physics::NodeTranslational * curNodeTrn = static_cast<physics::NodeTranslational *>(physics::g_nodes[iNode]);
			physics::NodeRotational * curNodeRot = static_cast<physics::NodeRotational *>(physics::g_nodes[iNode+1]);

			Mat34 Rotation;
			// NOTE: x y z wF
			Rotation = math::Quat::toMatrix33(curNodeRot->m_rot);

			// Get Basis
			wsX = Rotation.getBasis0();
			wsY = Rotation.getBasis1();
			wsZ = Rotation.getBasis2();

			drawLine(curNodeTrn->m_pos, curNodeTrn->m_pos + wsX, Vec4C(1.0f, 0.0f, 0.0f, 1.0f));
			drawLine(curNodeTrn->m_pos, curNodeTrn->m_pos + wsY, Vec4C(0.0f, 1.0f, 0.0f, 1.0f));
			drawLine(curNodeTrn->m_pos, curNodeTrn->m_pos + wsZ, Vec4C(0.0f, 0.0f, 1.0f, 1.0f));
		}

		// Draw geometries, geometries are included in the higher level rigid body concept
		//	(nodes do not have info about geometries)
		for (size_t irb = 0, irbEnd = physics::g_rigidBodies.size(); irb < irbEnd; ++irb)
		{
			physics::RigidBody * curBody = physics::g_rigidBodies[irb];

			Mat34 shapeTransform;
			Mat34 bodyTransform;
			Mat34 totalTransform;

			bodyTransform = Quat::toMatrix33(curBody->m_bodyR->m_rot);
			bodyTransform.fillTranslation(curBody->m_bodyL->m_pos);

			for (size_t gi = 0, giEnd = curBody->m_geometryShapes.size(); gi < giEnd; ++gi)
			{
				physics::GeometryShape * curShape = curBody->m_geometryShapes[gi];

				shapeTransform = curShape->m_rotation;
				shapeTransform.fillTranslation(curShape->m_origin - curBody->m_bodyR->m_com);

				totalTransform = bodyTransform * shapeTransform;

				if (curShape->getType() == physics::GeometryShape::Type::eGenericConvex)
				{
					physics::GeometryConvex * curConvex = static_cast<physics::GeometryConvex *>(curShape);
					if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eBox)
					{
						// Get box extents
						Vec3 extX = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(1.0f, 0.0f, 0.0f));
						Vec3 extY = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(0.0f, 1.0f, 0.0f));
						Vec3 extZ = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(0.0f, 0.0f, 1.0f));

						Vec3 halfExt = Vec3C(extX.x, extY.y, extZ.z);
						drawVerticesScaled(unitBoxVertices, totalTransform, halfExt);
					}
					else if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eSphere)
					{
						// Get sphere radius
						Vec3 radius = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(1.0f, 0.0f, 0.0f));
						drawVerticesScaled(unitSphereVertices, totalTransform, Vec3C(radius.x, radius.x, radius.x));
					}
					else if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eCapsule)
					{
						// Capsule is oriented vertically
						Vec3 radius = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(1.0f, 0.0f, 0.0f));
						Vec3 height = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(0.0f, 1.0f, 0.0f));

						height.y -= radius.x;

						drawVerticesScaled(unitCylinderVertices, totalTransform, Vec3C(radius.x, height.y, radius.x));

						Mat34 translation;
						translation.identity();

						translation.fillTranslation(height);
						drawVerticesScaled(unitHemiSphereVertices, totalTransform * translation, Vec3C(radius.x, radius.x, radius.x));

						translation.fillTranslation(-height);
						translation.fillRotation(Vec3C(1.0f, 0.0f, 0.0f), PI);
						drawVerticesScaled(unitHemiSphereVertices, totalTransform * translation, Vec3C(radius.x, radius.x, radius.x));
					}
					else if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eCylinder)
					{
						// Get cylinder extents
						Vec3 radius = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(1.0f, 0.0f, 0.0f));
						Vec3 height = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(0.0f, 1.0f, 0.0f));
						drawVerticesScaled(unitCylinderVertices, totalTransform, Vec3C(radius.x, height.y, radius.x));

						Mat34 translation;
						translation.identity();

						translation.fillTranslation(height);
						drawVerticesScaled(unitDiskVertices, totalTransform * translation, Vec3C(radius.x, 1.0f, radius.x));

						translation.fillTranslation(-height);
						translation.fillRotation(Vec3C(1.0f, 0.0f, 0.0f), PI);
						drawVerticesScaled(unitDiskVertices, totalTransform * translation, Vec3C(radius.x, 1.0f, radius.x));
					}
					else if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eCone)
					{
						// Get cone extents
						Vec3 radius = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(1.0f, 0.0f, 0.0f));
						Vec3 height = curConvex->m_convexHullShape->getSupportVertexLocal(Vec3C(0.0f, 1.0f, 0.0f));
						// Normals will be slightly off, as cylinder is scaled
						// TODO: introduce additional function that also scales normal
						drawVerticesScaled(unitConeVertices, totalTransform, Vec3C(radius.x, height.y, radius.x));

						Mat34 translation;
						translation.identity();

						translation.fillTranslation(-height);
						translation.fillRotation(Vec3C(1.0f, 0.0f, 0.0f), PI);
						drawVerticesScaled(unitDiskVertices, totalTransform * translation, Vec3C(radius.x, 1.0f, radius.x));
					}
					else if (curConvex->m_convexHullShape->getType() == ConvexHullShape::Type::eCustom)
					{
						ConvexCustom * curCustomConvex = static_cast<ConvexCustom *>(curConvex->m_convexHullShape);
						const size_t numTris = curCustomConvex->m_hullTriangles.size() / 3;
						for (size_t tri = 0; tri < numTris; ++tri)
						{
							const Vec3 renderOffset = Vec3C(0.0f, 0.0f, 0.0f);
							const Vec2 renderTCs = Vec2C(0.0f, 0.0f);
							const Vec4 renderColor = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

							const Vec3 & p0 = curCustomConvex->m_hullTriangles[tri*3+0];
							const Vec3 & p1 = curCustomConvex->m_hullTriangles[tri*3+1];
							const Vec3 & p2 = curCustomConvex->m_hullTriangles[tri*3+2];

							Vec3 p0w = totalTransform * p0;
							Vec3 p1w = totalTransform * p1;
							Vec3 p2w = totalTransform * p2;

							auto posToTexCoords = [](const Vec3 & pos) -> Vec2
							{
								float r = pos.len();
								float theta = atan2f(pos.x, pos.y);
								float phi = atan2f(pos.z, sqrtf(pos.x*pos.x + pos.y*pos.y));
								return Vec2C(theta, phi);
							};

							vulkan::Vertex renderV0, renderV1, renderV2;
							renderV0.pos = renderOffset + p0w;
							renderV0.col = renderColor;
							renderV0.tc = posToTexCoords(p0);
							renderV0.nrm = (p1w - p0w).cross(p2w - p0w).normalize();
							renderV1.pos = renderOffset + p1w;
							renderV1.col = renderColor;
							renderV1.tc = posToTexCoords(p1);
							renderV1.nrm = (p2w - p1w).cross(p0w - p1w).normalize();
							renderV2.pos = renderOffset + p2w;
							renderV2.col = renderColor;
							renderV2.tc = posToTexCoords(p2);
							renderV2.nrm = (p0w - p2w).cross(p1w - p2w).normalize();

							// Change winding for backface culling
							renderManager.m_debugTris.push_back(renderV0);
							renderManager.m_debugTris.push_back(renderV2);
							renderManager.m_debugTris.push_back(renderV1);
						}
					}
				}
			}
		}
#endif

#if 1
		Vec3 mouseWorldCoord = Vec3C();
		if (callbackData.mouseMode == CallbackData::MouseMode::ePicking)
		{
			// If mouse mode is picking - calculate mouse pointer coordinates in the world space
			float perspFOV;
			float perspAsp;
			float perspNear;
			float perspFar;
			float perspWidth;
			float perspHeight;
			renderManager.getViewPerspProjParams(&perspFOV, &perspAsp, &perspNear, &perspFar, &perspWidth, &perspHeight);

			Mat44 perspMatrix;
			projPerspective(perspFOV, perspAsp, perspNear, perspFar, perspWidth, perspHeight, &perspMatrix);

			float nrmX = internalMouseX / (float)window.getWidth() * 2.0f - 1.0f;
			float nrmY = internalMouseY / (float)window.getHeight() * 2.0f - 1.0f;
		
			Vec3 mouseCoord = Vec3C(nrmX, nrmY, 0.5f);
			Mat34 viewMat;
			mainCamera.fillMatrix(&viewMat);
			Mat44 mvp = perspMatrix * viewMat;
			Mat44 imvp = mvp;
			imvp.invert();
			mouseWorldCoord = imvp * mouseCoord;
			float ow = mouseCoord.x * imvp._30 + mouseCoord.y * imvp._31 + mouseCoord.z * imvp._32 + imvp._33;
			mouseWorldCoord /= ow;

			drawPoint(mouseWorldCoord, Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
		}
#endif

#if (ENABLE_PHYSICS == 1)
		if (callbackData.mouseMode == CallbackData::MouseMode::ePicking)
		{
			// Raycast and create joint on closest hit (if registered)
			static int lmbStatePrev = 0;
			if (callbackData.lmbState && !lmbStatePrev)
			{
				Vec3 rayO = mainCamera.getPosition();
				Vec3 rayD = mouseWorldCoord - mainCamera.getPosition();
				rayD.normalize();

				float minHitTime = FLT_MAX;
				physics::RigidBody * hitRigidBody = nullptr;

				for (size_t irb = 0, irbEnd = physics::g_rigidBodies.size(); irb < irbEnd; ++irb)
				{
					physics::RigidBody * curBody = physics::g_rigidBodies[irb];
					for (size_t gi = 0, giEnd = curBody->m_geometryShapes.size(); gi < giEnd; ++gi)
					{
						physics::GeometryShape * curShape = curBody->m_geometryShapes[gi];

						if (curShape->getType() != physics::GeometryShape::Type::eGenericConvex)
							continue;

						Mat34 shapeTransform;
						Mat34 bodyTransform;
						Mat34 totalTransform;

						shapeTransform = curShape->m_rotation;
						shapeTransform.fillTranslation(curShape->m_origin - curBody->m_bodyR->m_com);

						bodyTransform = Quat::toMatrix33(curBody->m_bodyR->m_rot);
						bodyTransform.fillTranslation(curBody->m_bodyL->m_pos);

						totalTransform = bodyTransform * shapeTransform;

						Vec3 hullPos = totalTransform.getBasis3();

						// Get closest point on a line
						float projTime = (hullPos - rayO).dot(rayD);
						Vec3 closestPointOnLine = rayO + projTime * rayD;

						Vec3 diffVec = closestPointOnLine - hullPos;

						Quat hullRot = Quat::fromMatrix33(totalTransform.getRotation33());
						hullRot.normalize();

						physics::GeometryConvex * curConvexShape = static_cast<physics::GeometryConvex *>(curShape);
						Vec3 supportToRay = curConvexShape->m_convexHullShape->getSupportVertex(diffVec, hullPos, hullRot);
						if (supportToRay.sqLen() < diffVec.sqLen())
						{
							continue;
						}

						float rayScalar = 0.0f;
						bool rayHit = raycastVsConvex(curConvexShape->m_convexHullShape, hullPos, hullRot, rayO, rayD, &rayScalar, 100.0f);
						if (rayHit)
						{
							if (minHitTime > rayScalar)
							{
								minHitTime = rayScalar;
								hitRigidBody = curBody;
							}
						}
					}
				}
				if (hitRigidBody)
				{
					Vec3 hitPoint = rayO + minHitTime * rayD;

					// Picking joint - should be relatively weak to avoid system instabilities due to
					//	sharp and high-amplitude user interactions
					physics::BallSocket * pickingBallsocket = new physics::BallSocket;
					pickingBallsocket->init(
						0.1f, 0.1f, hitPoint,
						hitRigidBody->m_bodyL,
						hitRigidBody->m_bodyR,
						nullptr,
						nullptr
						);

					physics::addTrackJoint(pickingBallsocket);
					pickingJoint = pickingBallsocket;
					pickingDistance = minHitTime;
				}
			}
			else if (!callbackData.lmbState && pickingJoint)
			{
				// Probably should go in reverse direction as picking joint most likely near to the end
				for (size_t ij = 0, ijEnd = physics::g_joints.size(); ij < ijEnd; ++ij)
				{
					physics::JointBase * curJoint = physics::g_joints[ij];
					if (curJoint == pickingJoint)
					{
						physics::g_joints.erase(physics::g_joints.begin() + ij);
						break;
					}
				}
				if (pickingJoint)
					delete pickingJoint;
				pickingJoint = nullptr;
			}
			else if (pickingJoint)
			{
				// Attachment to regular rigid body
				if (pickingJoint->getType() == physics::JointBase::Type::eBallSocket)
				{
					physics::BallSocket * pickingBallsocket = static_cast<physics::BallSocket *>(pickingJoint);
					Vec3 rayO = mainCamera.getPosition();
					Vec3 rayD = mouseWorldCoord - mainCamera.getPosition();
					rayD.normalize();
					Vec3 newPoint = rayO + pickingDistance * rayD;
					pickingBallsocket->setAnchorPoint1(newPoint);
				}
			}

			lmbStatePrev = callbackData.lmbState;
		}

		if (callbackData.reqDropBox)
		{
			callbackData.reqDropBox = false;

			const float skinWidth = 0.015f;

			physics::NodeTranslational * bodyL;
			const float boxMass = 1.0f;
			static int DBGnumBoxes = 0;

			Vec3 camPos = mainCamera.getPosition();
			Vec3 camView = mainCamera.getView().getNormalized();

			Vec3 camRight = mainCamera.getUp().cross(camView);
			camRight.normalize();

			Vec3 camUp = camView.cross(camRight);
			camUp.normalize();

			Mat33 boxOrient;
			boxOrient.setBasis0(camRight);
			boxOrient.setBasis1(camUp);
			boxOrient.setBasis2(camView);

			bodyL = physics::addTranslationalNode(1.0f / boxMass, camPos - camView, Vec3C());
			Quat identityQuat(1.0f, 0.0f, 0.0f, 0.0f);
			physics::NodeRotational * bodyR = physics::addRotationalNode(Mat33().identity(), identityQuat, Vec3C());
			bodyR->m_rot = Quat::fromMatrix33(boxOrient);
			bodyR->m_rot.normalize();
			Mat33 DBGmat = Quat::toMatrix33(bodyR->m_rot);

			bodyL->m_vel = -20.0f*camView;

			DBGnumBoxes++;

			bodyL->m_skinWidth = skinWidth;

			physics::RigidBody * body = physics::createRigidBody();
			body->m_bodyL = bodyL;
			body->m_bodyR = bodyR;

			ConvexBox * convexBox= new ConvexBox;
			convexBox->m_halfExtents = Vec3C(0.5f, 0.5f, 0.5f);
			convexBox->m_origin = Vec3C();
			convexBox->m_rotation.identity();

			physics::GeometryConvex * convex = new physics::GeometryConvex;
			convex->m_density = 1.0f;
			convex->m_origin = Vec3C(0.0f, 0.0f, 0.0f);
			convex->m_rotation.identity();
			convex->m_convexHullShape = convexBox;

			trackGeometryShape(convex);
			body->m_geometryShapes.push_back(convex);

			body->prepareGeometry();
		}
#endif

		for (const ExtDrawLine & simLine : g_simLines)
		{
			drawLine(simLine.p0, simLine.p1, simLine.c0);
		}
		for (const ExtDrawPoint & simPoint : g_simPoints)
		{
			drawPoint(simPoint.p, simPoint.c);
		}

#if 0
		{
			Vec3 cubeOffset = Vec3C(0.0f, 3.0f, 0.0f);
			Vec3 cubeSize = Vec3C(1.0f, 1.0f, 1.0f);
			Vec3 cubeNodes[8] =
				{
					Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z),//0
					Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z),//1
					Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z),//2
					Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z),//3
					Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z),//4
					Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z),//5
					Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z),//6
					Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z),//7
				};
			for (int i = 0; i < 8; ++i)
			{
				cubeNodes[i] += cubeOffset;
			}

			Vec3 tetraOffsets[5];
			for (int i = 0; i < 5; ++i)
			{
				tetraOffsets[i] = Vec3C();
			}

			auto drawWireTetra = [&drawLine](const Vec3 & v0, const Vec3 & v1, const Vec3 & v2, const Vec3 & v3, const Vec4C & color)
			{
				drawLine(v0, v1, color);
				drawLine(v0, v2, color);
				drawLine(v1, v2, color);
				drawLine(v1, v3, color);
				drawLine(v2, v3, color);
				drawLine(v3, v0, color);
			};

			auto drawTetra = [&renderManager](const Vec3 & v0, const Vec3 & v1, const Vec3 & v2, const Vec3 & v3, const Vec4C & color)
			{
				vulkan::Vertex faceVtx;
				faceVtx.col = color;

				auto addTetraFace = [&renderManager](
					vulkan::Vertex & faceVtx,
					const Vec3 & n0,
					const Vec3 & n1,
					const Vec3 & n2
					)
				{
					Vec3 normal = (n1 - n0).cross(n2 - n0).getNormalized();

					faceVtx.tc = Vec2C(0.0f, 0.0f);
					faceVtx.pos = n0;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
					faceVtx.tc = Vec2C(1.0f, 0.0f);
					faceVtx.pos = n1;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
					faceVtx.tc = Vec2C(0.0f, 1.0f);
					faceVtx.pos = n2;
					faceVtx.nrm = normal;
					renderManager.m_debugTris.push_back(faceVtx);
				};

				addTetraFace(faceVtx, v0, v2, v1);
				addTetraFace(faceVtx, v0, v1, v3);
				addTetraFace(faceVtx, v0, v3, v2);
				addTetraFace(faceVtx, v1, v2, v3);
			};

			const float translationMul = 0.4f * (sinf((float)sampleApp.getElapsedTime() * 0.001f) * 0.5f + 0.5f);
			const bool cornerTetraWire = true;
			const bool centerTetraWire = false;

			// Render Corner tetra 0
			tetraOffsets[0] = translationMul*Vec3C(-1.0f, -1.0f, -1.0f);
			int t0idx0 = 1, t0idx1 = 0, t0idx2 = 2, t0idx3 = 4;
			if (cornerTetraWire)
				drawWireTetra(tetraOffsets[0]+cubeNodes[t0idx0], tetraOffsets[0]+cubeNodes[t0idx1], tetraOffsets[0]+cubeNodes[t0idx2], tetraOffsets[0]+cubeNodes[t0idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			else
				drawTetra(tetraOffsets[0]+cubeNodes[t0idx0], tetraOffsets[0]+cubeNodes[t0idx1], tetraOffsets[0]+cubeNodes[t0idx2], tetraOffsets[0]+cubeNodes[t0idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));

			// Render Corner tetra 1
			tetraOffsets[1] = translationMul*Vec3C( 1.0f, -1.0f,  1.0f);
			int t1idx0 = 5, t1idx1 = 4, t1idx2 = 1, t1idx3 = 7;
			if (cornerTetraWire)
				drawWireTetra(tetraOffsets[1]+cubeNodes[t1idx0], tetraOffsets[1]+cubeNodes[t1idx1], tetraOffsets[1]+cubeNodes[t1idx2], tetraOffsets[1]+cubeNodes[t1idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			else
				drawTetra(tetraOffsets[1]+cubeNodes[t1idx0], tetraOffsets[1]+cubeNodes[t1idx1], tetraOffsets[1]+cubeNodes[t1idx2], tetraOffsets[1]+cubeNodes[t1idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));

			// Render Corner tetra 2
			tetraOffsets[2] = translationMul*Vec3C( 1.0f,  1.0f, -1.0f);
			int t2idx0 = 2, t2idx1 = 3, t2idx2 = 1, t2idx3 = 7;
			if (cornerTetraWire)
				drawWireTetra(tetraOffsets[2]+cubeNodes[t2idx0], tetraOffsets[2]+cubeNodes[t2idx1], tetraOffsets[2]+cubeNodes[t2idx2], tetraOffsets[2]+cubeNodes[t2idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			else
				drawTetra(tetraOffsets[2]+cubeNodes[t2idx0], tetraOffsets[2]+cubeNodes[t2idx1], tetraOffsets[2]+cubeNodes[t2idx2], tetraOffsets[2]+cubeNodes[t2idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));

			// Render Corner tetra 3
			tetraOffsets[3] = translationMul*Vec3C(-1.0f,  1.0f,  1.0f);
			int t3idx0 = 6, t3idx1 = 7, t3idx2 = 2, t3idx3 = 4;
			if (cornerTetraWire)
				drawWireTetra(tetraOffsets[3]+cubeNodes[t3idx0], tetraOffsets[3]+cubeNodes[t3idx1], tetraOffsets[3]+cubeNodes[t3idx2], tetraOffsets[3]+cubeNodes[t3idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			else
				drawTetra(tetraOffsets[3]+cubeNodes[t3idx0], tetraOffsets[3]+cubeNodes[t3idx1], tetraOffsets[3]+cubeNodes[t3idx2], tetraOffsets[3]+cubeNodes[t3idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));

			// Render Center tetra
			tetraOffsets[4] = translationMul*Vec3C( 0.0f,  0.0f,  0.0f);
			int t4idx0 = 2, t4idx1 = 7, t4idx2 = 1, t4idx3 = 4;
			if (centerTetraWire)
				drawWireTetra(tetraOffsets[4]+cubeNodes[t4idx0], tetraOffsets[4]+cubeNodes[t4idx1], tetraOffsets[4]+cubeNodes[t4idx2], tetraOffsets[4]+cubeNodes[t4idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
			else
				drawTetra(tetraOffsets[4]+cubeNodes[t4idx0], tetraOffsets[4]+cubeNodes[t4idx1], tetraOffsets[4]+cubeNodes[t4idx2], tetraOffsets[4]+cubeNodes[t4idx3], Vec4C(1.0f, 1.0f, 1.0f, 1.0f));
		}
#endif

		sampleApp.update(callbackData.isPaused ? 0.0 : dtMS);

		renderManager.beginFrame();
		renderManager.update();
		renderManager.render();

		dtMS = perfTimer.time();
		accumTimeMS += dtMS;
		physicsTime += dtMS * 0.001;
		++accumFrames;
		if (accumTimeMS > 500.0)
		{
			sampleApp.setDTime(accumTimeMS / (float)accumFrames);
			accumTimeMS = 0.0;
			accumFrames = 0;
		}
		perfTimer.start();

		if (callbackData.isAnimStep)
		{
			callbackData.isPaused = true;
			callbackData.isAnimStep = false;
		}
	}

#if (ENABLE_PHYSICS == 1)
	physics::cleanupPhysics();
#endif

	sampleApp.deinit();
	window.deinit();

	return 0;
}