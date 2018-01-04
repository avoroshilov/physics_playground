#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "shader_bindings.h"

layout(location = 0) in vec4 in_color;
layout(location = 1) in vec3 in_normal_cam;
layout(location = 2) in vec2 in_texCoords;
layout(location = 3) in float in_time;
layout(location = 4) in vec3 in_L_cam;
layout(location = 5) in vec3 in_worldPos;

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

layout(set = 0, binding = SH_BIND_FWDSH_ALBEDO_TEX) uniform sampler2D texSampler;
layout(set = 0, binding = SH_BIND_FWDSH_SHADOWMAP_TEX) uniform sampler2DShadow shadowmapSampler;
layout(set = 0, binding = SH_BIND_FWDSH_LIGHTPROJ_TEX) uniform sampler2D lightSampler;

layout(set = 0, binding = SH_BIND_FWDSH_LIGHTMATRIX) uniform LightMatrixUBO
{
	mat4 view;
	mat4 proj;
} lightMatrixUBO;

void main()
{
	mat4 shadowMatrix = lightMatrixUBO.proj * lightMatrixUBO.view;
	vec4 shadowmapPos = shadowMatrix * vec4(in_worldPos, 1.0);
	shadowmapPos /= shadowmapPos.w;

	vec4 lightProjColor = vec4(0.0, 0.0, 0.0, 0.0);
	vec4 shadowMaskColor = vec4(0.0, 0.0, 0.0, 0.0);

	if (shadowmapPos.x > -1.0 && shadowmapPos.x < 1.0 &&
		shadowmapPos.y > -1.0 && shadowmapPos.y < 1.0)
	{
		ivec2 shadowMapSize = textureSize(shadowmapSampler, 0);
		vec2 stepSize = vec2(1.0 / shadowMapSize.x, 1.0 / shadowMapSize.y);

		float depthBias = 0.0;
		
#define MANUAL_DEPTH_BIAS 0
#if (MANUAL_DEPTH_BIAS == 1)
		depthBias = 0.00017;
#endif

		shadowmapPos.x = 0.5 * shadowmapPos.x + 0.5;
		shadowmapPos.y = 0.5 * shadowmapPos.y + 0.5;
		shadowmapPos.z -= depthBias;
		// Central
		float shadowMapCmp = texture(shadowmapSampler, shadowmapPos.xyz);
		shadowMaskColor += vec4(shadowMapCmp, shadowMapCmp, shadowMapCmp, 1.0);

#define MORE_SHADOWMAP_FILTERING 1
#if (MORE_SHADOWMAP_FILTERING == 1)
		shadowMapCmp = texture(shadowmapSampler, vec3(-stepSize.x, 0.0, 0.0) + shadowmapPos.xyz);
		shadowMaskColor += vec4(shadowMapCmp, shadowMapCmp, shadowMapCmp, 1.0);
		shadowMapCmp = texture(shadowmapSampler, vec3( stepSize.x, 0.0, 0.0) + shadowmapPos.xyz);
		shadowMaskColor += vec4(shadowMapCmp, shadowMapCmp, shadowMapCmp, 1.0);
		shadowMapCmp = texture(shadowmapSampler, vec3(0.0, -stepSize.y, 0.0) + shadowmapPos.xyz);
		shadowMaskColor += vec4(shadowMapCmp, shadowMapCmp, shadowMapCmp, 1.0);
		shadowMapCmp = texture(shadowmapSampler, vec3(0.0,  stepSize.y, 0.0) + shadowmapPos.xyz);
		shadowMaskColor += vec4(shadowMapCmp, shadowMapCmp, shadowMapCmp, 1.0);

		shadowMaskColor /= 5.0;
#endif
		
		lightProjColor = texture(lightSampler, shadowmapPos.xy);
	}
	else
	{
		shadowMaskColor = vec4(1.0, 1.0, 1.0, 1.0);
	}

	shadowMaskColor = lightProjColor*shadowMaskColor;

	vec4 colorTex = texture(texSampler, in_texCoords);
#if 0
	//outColor = in_color * vec4(0.5 * in_normal + 0.5, 1.0);
	outColor = in_color * colorTex;
#else
	// Working in camera (rather than in world) space

	// Renormalize vectors after the interpolator
	vec3 normal_cam = normalize(in_normal_cam);
	vec3 L_cam = normalize(in_L_cam);

	float cosLightDir = dot(L_cam, normal_cam);

#define PI 3.14159265

	// Lambertian diffuse
	vec4 lightColor = vec4(0.9, 1.0, 0.85, 1.0);
	float lambertN = 1.0 / PI;
	// Two sided diffuse (abs instead of max(..., 0.0))
	vec4 diffuse = (lambertN * abs(cosLightDir)) * shadowMaskColor*lightColor;

	// Blinn-phong reflectance
	float shininess = 20.0;
	vec4 lightSpecularColor = vec4(0.8, 1.0, 0.6, 1.0);

	vec3 viewer_cam = vec3(0.0, 0.0, 1.0);

	vec4 specular;
	if (cosLightDir > 0.0)
	{
		vec3 halfVec_cam = L_cam + viewer_cam;
		halfVec_cam = normalize(halfVec_cam);
		float blinnPhongN = (shininess + 2) / (4 * PI * (2 - exp2(-shininess/2.0)));
		specular = blinnPhongN * vec4(pow( max(dot(halfVec_cam, normal_cam), 0.0), shininess )) * shadowMaskColor*lightSpecularColor;
	}
	else
	{
		specular = vec4(0.0, 0.0, 0.0, 0.0);
	}

	vec4 ambient = vec4(0.5, 0.5, 0.5, 0.5);
	outColor = in_color * ((diffuse + ambient) * colorTex + specular);
#endif
}