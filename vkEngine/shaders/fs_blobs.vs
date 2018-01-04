#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "shader_bindings.h"

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec4 in_color;
layout(location = 3) in vec2 in_texCoords;

layout(set = 0, binding = SH_BIND_GLOBAL_CONSTANTS) uniform UniformBufferObject
{
	float time;
} ubo;

layout(set = 0, binding = SH_BIND_TRANSFORM) uniform TransformUBO
{
	mat4 view;
	mat4 proj;
} transformUBO;

layout(push_constant) uniform MeshPushConst
{
	mat4 model;
} meshPushConst;

out gl_PerVertex
{
	vec4 gl_Position;
};

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec2 out_texCoords;
layout(location = 2) out float out_time;
layout(location = 3) out vec2 blobCenters[5];

void main()
{
	gl_Position = vec4(in_position, 1.0);
	out_texCoords = in_texCoords;
	out_color = in_color;
	out_time = ubo.time;
	
	const float mulX[5] = { 0.5, 0.7, 0.8, 0.3, 0.6 };
	const float mulY[5] = { 0.8, 0.6, 0.5, 0.7, 0.4 };
	const float shiftX1[5] = { 0.1, 1.5, 3.5, 0.6, 1.9 };
	const float shiftY1[5] = { 0.6, 1.6, 2.8, 0.1, 1.3 };
	const float shiftX2[5] = { 0.9, 1.3, 2.9, 0.4, 2.3 };
	const float shiftY2[5] = { 0.3, 1.9, 3.2, 0.9, 2.4 };
	const float timeMulX1[5] = { 1.0, 1.2, 0.5, 0.6, 0.9 };
	const float timeMulY1[5] = { 1.0, 1.3, 0.8, 1.1, 1.3 };

	const float blobTime = ubo.time * 0.001;
	for (int i = 0; i < 5; ++i)
	{
		blobCenters[i] = vec2(
			mulX[i] * cos(timeMulX1[i] * blobTime + shiftX1[i]) * sin(blobTime + shiftX2[i]) + 0.5,
			mulY[i] * sin(timeMulY1[i] * blobTime + shiftY1[i]) * sin(blobTime + shiftY2[i]) + 0.25
			);
	}	
}