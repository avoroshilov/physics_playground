#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "shader_bindings.h"

layout(location = 0) in vec4 in_color;
layout(location = 1) in vec3 in_normal_cam;
layout(location = 2) in vec2 in_texCoords;
layout(location = 3) in float in_time;

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
}