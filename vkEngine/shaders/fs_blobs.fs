#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "shader_bindings.h"

layout(location = 0) in vec4 in_color;
layout(location = 1) in vec2 in_texCoords;
layout(location = 2) in float in_time;
layout(location = 3) in vec2 blobCenters[5];

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = SH_BIND_GLOBAL_CONSTANTS) uniform UniformBufferObject
{
	float time;
} ubo;

layout(set = 0, binding = SH_BIND_TRANSFORM) uniform TransformUBO
{
	mat4 view;
	mat4 proj;
} transformUBO;

void main()
{
	const float width = 800.0;
	const float height = 600.0;
	const float aspect = width/height;
	
	const float radii[5] = { 0.4, 0.3, 0.2, 0.35, 0.25 };

	float scalar = 0.0;
	for (int i = 0; i < 5; ++i)
	{
		vec2 blobCenter = blobCenters[i];
		scalar += smoothstep(0.0, 1.0, clamp(1.0 - length(in_texCoords*vec2(aspect,1.0) - blobCenter) / radii[i], 0.0, 1.0));
	}
	
#define PI_DIV2 1.57079632679
		
	const float base = 0.2, range = 0.2;
	if (scalar > base && scalar < (base + range))
		scalar = sin(PI_DIV2 / (range/2.0) * (scalar - base));
	else if (scalar < base)
		scalar = 0.0;
	else
		scalar = sin(scalar - (base + range));

	outColor = vec4(scalar, scalar, scalar, 1.0);
}