#pragma once

#include "math\Vec2.h"
#include "math\Vec3.h"
#include "math\Vec4.h"

#include "math\Mat44.h"

namespace vulkan
{
	struct Vertex
	{
		math::Vec3 pos;
		math::Vec3 nrm;
		math::Vec4 col;
		math::Vec2 tc;
	};

	struct LinePoint
	{
		math::Vec3 pos;
		math::Vec4 col;
	};

	enum class BufferUsage
	{
		eStatic,
		eDynamic,

		eNUM_ENTRIES
	};

}