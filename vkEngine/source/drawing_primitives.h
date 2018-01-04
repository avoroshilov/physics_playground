#pragma once

#include <vector>

#include <math/Mat34.h>
#include <math/Vec3.h>

static void buildBoxVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const math::Vec3 & cubeSize, const math::Vec4 & color)
{
	using namespace math;

	vulkan::Vertex faceVtx[6];
	auto finalizeFace = [&faceVtx, &outVertices](const Mat34 & modelMatrix, const Vec3 & normal, const Vec4 & faceColor)
	{
		faceVtx[0].tc = Vec2C( 0.0f,  0.0f);	// 0
		faceVtx[1].tc = Vec2C( 0.0f,  1.0f);	// 3
		faceVtx[2].tc = Vec2C( 1.0f,  1.0f);	// 2

		faceVtx[3].tc = Vec2C( 0.0f,  0.0f);	// 0
		faceVtx[4].tc = Vec2C( 1.0f,  1.0f);	// 2
		faceVtx[5].tc = Vec2C( 1.0f,  0.0f);	// 1

		for (int fi = 0; fi < 6; ++fi)
		{
			faceVtx[fi].pos = modelMatrix * faceVtx[fi].pos;
			faceVtx[fi].nrm = normal;
			faceVtx[fi].col = faceColor;

			outVertices->push_back(faceVtx[fi]);
		}
	};

	Vec3 vecOffset = Vec3C(0.0f, 0.0f, 0.0f);

	outVertices->reserve(36);
	outVertices->resize(0);

	// Vertices
	// Front
	faceVtx[0].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[1].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[2].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[3].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[4].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[5].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
	finalizeFace(modelMatrix, Vec3C( 0.0f,  0.0f,  1.0f), color);

	// Back
	faceVtx[0].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[1].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[2].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[3].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[4].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[5].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
	finalizeFace(modelMatrix, Vec3C( 0.0f,  0.0f, -1.0f), color);

	// Left
	faceVtx[0].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[1].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[2].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[3].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[4].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[5].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
	finalizeFace(modelMatrix, Vec3C(-1.0f,  0.0f,  0.0f), color);

	// Right
	faceVtx[0].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[1].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[2].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[3].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[4].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[5].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
	finalizeFace(modelMatrix, Vec3C( 1.0f,  0.0f,  0.0f), color);

	// Top
	faceVtx[0].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[1].pos = Vec3C( cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[2].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[3].pos = Vec3C(-cubeSize.x,  cubeSize.y, -cubeSize.z);
	faceVtx[4].pos = Vec3C( cubeSize.x,  cubeSize.y,  cubeSize.z);
	faceVtx[5].pos = Vec3C(-cubeSize.x,  cubeSize.y,  cubeSize.z);
	finalizeFace(modelMatrix, Vec3C( 0.0f,  1.0f,  0.0f), color);

	// Bottom
	faceVtx[0].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[1].pos = Vec3C(-cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[2].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[3].pos = Vec3C(-cubeSize.x, -cubeSize.y, -cubeSize.z);
	faceVtx[4].pos = Vec3C( cubeSize.x, -cubeSize.y,  cubeSize.z);
	faceVtx[5].pos = Vec3C( cubeSize.x, -cubeSize.y, -cubeSize.z);
	finalizeFace(modelMatrix, Vec3C( 0.0f, -1.0f,  0.0f), color);
}

static void buildSphereVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const float radius, const math::Vec4 & color)
{
	using namespace math;

	const uint32_t detI = 24;
	const uint32_t detJ = 24;

	std::vector<vulkan::Vertex> verticesStorage;
	verticesStorage.resize(detI*detJ);
	vulkan::Vertex * vertices = verticesStorage.data();

	for (uint32_t vj = 0; vj < detJ; ++vj)
	{
		const float nrm_vj = vj/(float)(detJ - 1);
		const float angJ = PI*nrm_vj - _PI2;

		for (uint32_t vi = 0; vi < detI; ++vi)
		{
			const float nrm_vi = vi/(float)(detI - 1);
			const float angI = _2PI*nrm_vi;

			vulkan::Vertex & curVertex = vertices[vi+vj*detI];
			Vec3 unitSphere = Vec3C(cosf(angJ)*cosf(angI), sinf(angJ), cosf(angJ)*sinf(angI));
			curVertex.pos = modelMatrix * (radius * unitSphere);

			curVertex.tc = Vec2C(nrm_vi, nrm_vj);
			curVertex.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

			curVertex.nrm = unitSphere;
		}
	}

	const uint32_t numQuadsI = detI - 1;
	const uint32_t numQuadsJ = detJ - 1;
	const uint32_t numTrisPerQuad = 2;
	const uint32_t numTris = numQuadsI*numQuadsJ*numTrisPerQuad;
	const uint32_t numIdxPerTri = 3;
	const size_t numIndices = numTris * numIdxPerTri;

	outVertices->reserve(numQuadsI*numQuadsJ*6);
	outVertices->resize(0);
	for (uint32_t qj = 0; qj < numQuadsJ; ++qj)
	{
		for (uint32_t qi = 0; qi < numQuadsI; ++qi)
		{
			outVertices->push_back(vertices[qi+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);

			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+(qj+1)*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);
		}
	}
}

static void buildHemiSphereVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const float radius, const math::Vec4 & color)
{
	using namespace math;

	const uint32_t detI = 24;
	const uint32_t detJ = 12;

	std::vector<vulkan::Vertex> verticesStorage;
	verticesStorage.resize(detI*detJ);
	vulkan::Vertex * vertices = verticesStorage.data();
	
	for (uint32_t vj = 0; vj < detJ; ++vj)
	{
		const float nrm_vj = vj/(float)(detJ - 1);
		const float angJ = PI*(nrm_vj*0.5f + 0.5f) - _PI2;

		for (uint32_t vi = 0; vi < detI; ++vi)
		{
			const float nrm_vi = vi/(float)(detI - 1);
			const float angI = _2PI*nrm_vi;

			vulkan::Vertex & curVertex = vertices[vi+vj*detI];
			Vec3 unitSphere = Vec3C(cosf(angJ)*cosf(angI), sinf(angJ), cosf(angJ)*sinf(angI));
			curVertex.pos = modelMatrix * (radius * unitSphere);

			curVertex.tc = Vec2C(nrm_vi, nrm_vj);
			curVertex.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

			curVertex.nrm = unitSphere;
		}
	}

	const uint32_t numQuadsI = detI - 1;
	const uint32_t numQuadsJ = detJ - 1;
	const uint32_t numTrisPerQuad = 2;
	const uint32_t numTris = numQuadsI*numQuadsJ*numTrisPerQuad;
	const uint32_t numIdxPerTri = 3;
	const size_t numIndices = numTris * numIdxPerTri;

	outVertices->reserve(numQuadsI*numQuadsJ*6);
	outVertices->resize(0);
	for (uint32_t qj = 0; qj < numQuadsJ; ++qj)
	{
		for (uint32_t qi = 0; qi < numQuadsI; ++qi)
		{
			outVertices->push_back(vertices[qi+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);

			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+(qj+1)*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);
		}
	}
}

static void buildCylinderVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const float height, const float radius, const math::Vec4 & color)
{
	using namespace math;

	const uint32_t detI = 24;
	const uint32_t detJ = 2;

	std::vector<vulkan::Vertex> verticesStorage;
	verticesStorage.resize(detI*detJ);
	vulkan::Vertex * vertices = verticesStorage.data();

	for (uint32_t vj = 0; vj < detJ; ++vj)
	{
		const float nrm_vj = vj/(float)(detJ - 1);

		for (uint32_t vi = 0; vi < detI; ++vi)
		{
			const float nrm_vi = vi/(float)(detI - 1);
			const float angI = _2PI*nrm_vi;

			vulkan::Vertex & curVertex = vertices[vi+vj*detI];
			Vec3 unitCircle = Vec3C(cosf(angI), 0.0f, sinf(angI));
			curVertex.pos = modelMatrix * (radius * unitCircle + Vec3C(0.0f, (2 * nrm_vj - 1.0f) * height, 0.0f));

			curVertex.tc = Vec2C(nrm_vi, nrm_vj);
			curVertex.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

			curVertex.nrm = unitCircle;
		}
	}

	const uint32_t numQuadsI = detI - 1;
	const uint32_t numQuadsJ = detJ - 1;
	const uint32_t numTrisPerQuad = 2;
	const uint32_t numTris = numQuadsI*numQuadsJ*numTrisPerQuad;
	const uint32_t numIdxPerTri = 3;
	const size_t numIndices = numTris * numIdxPerTri;

	outVertices->reserve(numQuadsI*numQuadsJ*6);
	outVertices->resize(0);
	for (uint32_t qj = 0; qj < numQuadsJ; ++qj)
	{
		for (uint32_t qi = 0; qi < numQuadsI; ++qi)
		{
			outVertices->push_back(vertices[qi+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);

			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+(qj+1)*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);
		}
	}
}

static void buildConeVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const float height, const float radius, const math::Vec4 & color)
{
	using namespace math;

	const uint32_t detI = 24;
	const uint32_t detJ = 2;

	std::vector<vulkan::Vertex> verticesStorage;
	verticesStorage.resize(detI*detJ);
	vulkan::Vertex * vertices = verticesStorage.data();

	for (uint32_t vj = 0; vj < detJ; ++vj)
	{
		const float nrm_vj = vj/(float)(detJ - 1);

		for (uint32_t vi = 0; vi < detI; ++vi)
		{
			const float nrm_vi = vi/(float)(detI - 1);
			const float angI = _2PI*nrm_vi;

			vulkan::Vertex & curVertex = vertices[vi+vj*detI];
			Vec3 unitCircle = Vec3C(cosf(angI), 0.0f, sinf(angI));
			curVertex.pos = modelMatrix * ((1.0f - nrm_vj)*radius * unitCircle + Vec3C(0.0f, (2 * nrm_vj - 1.0f) * height, 0.0f));

			curVertex.tc = Vec2C(nrm_vi, nrm_vj);
			curVertex.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

			// Handwavy normal approximation
			if (height != 0.0f)
			{
				curVertex.nrm = Vec3C(unitCircle.x, radius/height, unitCircle.y);
				curVertex.nrm.normalize();
			}
			else
			{
				curVertex.nrm = Vec3C(0.0f, 1.0f, 0.0f);
			}
		}
	}

	const uint32_t numQuadsI = detI - 1;
	const uint32_t numQuadsJ = detJ - 1;
	const uint32_t numTrisPerQuad = 2;
	const uint32_t numTris = numQuadsI*numQuadsJ*numTrisPerQuad;
	const uint32_t numIdxPerTri = 3;
	const size_t numIndices = numTris * numIdxPerTri;

	outVertices->reserve(numQuadsI*numQuadsJ*6);
	outVertices->resize(0);
	for (uint32_t qj = 0; qj < numQuadsJ; ++qj)
	{
		for (uint32_t qi = 0; qi < numQuadsI; ++qi)
		{
			outVertices->push_back(vertices[qi+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);

			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+(qj+1)*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);
		}
	}
}

static void buildDiskVertices(std::vector<vulkan::Vertex> * outVertices, const math::Mat34 & modelMatrix, const float radius, const math::Vec4 & color, bool radialTC = false)
{
	using namespace math;

	const uint32_t detI = 24;
	const uint32_t detJ = 2;

	std::vector<vulkan::Vertex> verticesStorage;
	verticesStorage.resize(detI*detJ);
	vulkan::Vertex * vertices = verticesStorage.data();

	for (uint32_t vj = 0; vj < detJ; ++vj)
	{
		const float nrm_vj = vj/(float)(detJ - 1);

		for (uint32_t vi = 0; vi < detI; ++vi)
		{
			const float nrm_vi = vi/(float)(detI - 1);
			const float angI = _2PI*nrm_vi;

			vulkan::Vertex & curVertex = vertices[vi+vj*detI];
			Vec3 unitCircle = Vec3C((1.0f - nrm_vj) * cosf(angI), 0.0f, (1.0f - nrm_vj) * sinf(angI));
			curVertex.pos = modelMatrix * (radius * unitCircle);

			if (radialTC)
				curVertex.tc = Vec2C(nrm_vi, nrm_vj);
			else
				curVertex.tc = Vec2C(unitCircle.x, unitCircle.z);
			curVertex.col = Vec4C(1.0f, 1.0f, 1.0f, 1.0f);

			curVertex.nrm = Vec3C(0.0f, 1.0f, 0.0f);
		}
	}

	const uint32_t numQuadsI = detI - 1;
	const uint32_t numQuadsJ = detJ - 1;
	const uint32_t numTrisPerQuad = 2;
	const uint32_t numTris = numQuadsI*numQuadsJ*numTrisPerQuad;
	const uint32_t numIdxPerTri = 3;
	const size_t numIndices = numTris * numIdxPerTri;

	outVertices->reserve(numQuadsI*numQuadsJ*6);
	outVertices->resize(0);
	for (uint32_t qj = 0; qj < numQuadsJ; ++qj)
	{
		for (uint32_t qi = 0; qi < numQuadsI; ++qi)
		{
			outVertices->push_back(vertices[qi+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);

			outVertices->push_back(vertices[(qi+1)+qj*detI]);
			outVertices->push_back(vertices[(qi+1)+(qj+1)*detI]);
			outVertices->push_back(vertices[qi+(qj+1)*detI]);
		}
	}
}
