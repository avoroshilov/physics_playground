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

layout(set = 0, binding = SH_BIND_FWDSH_LIGHTMATRIX) uniform LightMatrixUBO
{
	mat4 view;
	mat4 proj;
} lightMatrixUBO;

layout(push_constant) uniform MeshPushConst
{
	mat4 model;
} meshPushConst;

out gl_PerVertex
{
	vec4 gl_Position;
};

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec3 out_normal_cam;
layout(location = 2) out vec2 out_texCoords;
layout(location = 3) out float out_time;
layout(location = 4) out vec3 out_L_cam;
layout(location = 5) out vec3 out_worldPos;

void main()
{
	vec4 vertexWorldPos = meshPushConst.model * vec4(in_position, 1.0);
	out_worldPos = vertexWorldPos.xyz / vertexWorldPos.w;
	gl_Position = transformUBO.proj * transformUBO.view * vertexWorldPos;
	out_texCoords = in_texCoords;
	out_color = in_color;
	// Suppose view is orthonormal
	out_normal_cam = mat3(transformUBO.view) * mat3(meshPushConst.model) * in_normal;
	out_time = ubo.time;
	
	//
	mat4 lightToWorld = inverse(lightMatrixUBO.view);
	vec3 light_pos = (lightToWorld[3]).xyz;//vec3(3.0, 3.0, 3.0);
	out_L_cam = normalize((transformUBO.view * (vec4(light_pos, 1.0) - vertexWorldPos)).xyz);
}