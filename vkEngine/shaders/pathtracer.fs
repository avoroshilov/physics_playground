#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "shader_bindings.h"

layout(location = 0) in vec4 in_color;
layout(location = 1) in vec2 in_texCoords;
layout(location = 2) in float in_time;

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

layout(set = 0, binding = SH_BIND_PATHTRACER_NOISE_TEX) uniform sampler2D texSampler;

float fakeRand(vec2 co, float time)
{
    return fract(sin(dot(time * co.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

#define PI		3.14159265358979323846
#define _2PI	6.28318530717958647692

vec3 randInUnitSphere(vec2 co, float time)
{
	float angTheta = _2PI*fakeRand(co, time);
	float angPhi = _2PI*fakeRand(12.3456*co, time);
	float rad = fakeRand(45.6789*co, time);

	return vec3(rad*sin(angTheta)*cos(angPhi), rad*sin(angTheta)*sin(angPhi), rad*cos(angTheta));
}
vec2 randOnDisk(vec2 co, float time)
{
	float ang = _2PI*fakeRand(co, time);
	float rad = fakeRand(34.5678*co, time);

	return vec2(rad*cos(ang), rad*sin(ang));
}

/* Ray */
struct Ray
{
	vec3 O;
	vec3 D;
};

Ray getRay(vec3 O, vec3 D)
{
	Ray r;
	r.O = O;
	r.D = D;
	return r;
}

vec3 getRayPoint(Ray ray, float t)
{
	return ray.O + t * ray.D;
}
/* End of Ray */

/* Hitting Routines */
struct HitData
{
	vec3 p;
	vec3 n;
	float t;
	int materialIndex;
};

bool hitSphere(vec3 center, float radius, int materialIndex, Ray r, float t_min, float t_max, out HitData hitData)
{
	/*

	Sphere eqn: dot( (p-c), (p-c) ) = rad*rad
		where p - point, c - sphere center, rad - sphere radius
	subs p for r(t)=r.O+r.D*t - ray:
		dot( (r(t)-c), (r(t)-c) ) = rad*rad
		dot( (r.O+r.D*t-c), (r.O+r.D*t-c) ) = rad*rad
	=>
		<r.O,r.O> + <r.O,r.D*t> - <r.O,c> + <r.D*t,r.O> + <r.D*t,r.D*t> - <r.D*t,c> - <c,r.O> - <c,r.D*t> + <c,c> = rad*rad
		<r.O,r.O> + t*<r.O,r.D> - <r.O,c> + t*<r.D,r.O> + t*<t*r.D,r.D> - t*<r.D,c> - <c,r.O> - t*<c,r.D> + <c,c> = rad*rad

		t*t*<r.D,r.D> + t*(<r.O,r.D> + <r.D,r.O> - <r.D,c> - <c,r.D>) + (<r.O,r.O> - <r.O,c> - <c,r.O> + <c,c>) = rad*rad
		t*t*<r.D,r.D> + t*(2*<r.O,r.D> - 2*<r.D,c>) + (<r.O,r.O> - 2*<r.O,c> + <c,c>) = rad*rad

	also,
		2*<r.O,r.D> - 2*<r.D,c> = 2*<r.D,r.O-c>
		<r.O-c,r.O-c> = <r.O,r.O> - <c,r.O> - <r.O,c> + <c,c> = <r.O,r.O> - 2*<c,r.O> + <c,c>

	and,
		rayO2sphC = r.O-c

	hence,
		t*t*<r.D,r.D> + t*(2*<r.D,r.O-c>) + (<r.O-c,r.O-c>) = rad*rad

	*/

	vec3 rayO2sphC = r.O - center;
	float a = dot(r.D, r.D);
	float b = 2.0 * dot(r.D, rayO2sphC);
	float c = dot(rayO2sphC, rayO2sphC) - radius*radius;
	float discriminant = b*b - 4*a*c;

	if (discriminant < 0)
	{
		return false;
	}

	// x0,x1 = [-b +- sqrt(D)] / [2*a]
	float sqrtD_div2a = sqrt(discriminant) / (2*a);
	float negB_div2a = -b / (2*a);

	float hitT1 = negB_div2a - sqrtD_div2a;
	if (hitT1 > t_min && hitT1 < t_max)
	{
		hitData.t = hitT1;
		hitData.p = getRayPoint(r, hitT1);
		hitData.n = (hitData.p - center) / radius;
		hitData.materialIndex = materialIndex;
		return true;
	}

	float hitT2 = negB_div2a + sqrtD_div2a;
	if (hitT2 > t_min && hitT2 < t_max)
	{
		hitData.t = hitT2;
		hitData.p = getRayPoint(r, hitT2);
		hitData.n = (hitData.p - center) / radius;
		hitData.materialIndex = materialIndex;
		return true;
	}

	return false;
}

bool refract(vec3 v, vec3 n, float ni_over_nt, out vec3 outV)
{
	// Snell's law of refraction
	// 	n1 * sin(theta1) = n2 * sin(theta2)
	//	where n - refractive index, theta - is the angle measured from the normal of the boundary

	vec3 v_nrm = normalize(v);
	float vdotn = dot(v_nrm, n);
	float discriminant = 1.0 - ni_over_nt*ni_over_nt * (1.0 - vdotn*vdotn);
	if (discriminant <= 0)
		return false;

	outV = ni_over_nt * (v_nrm - vdotn*n) - n*sqrt(discriminant);
	return true;
}

float schlick(float cosine, float refIdx)
{
	float r0 = (1.0 - refIdx) / (1.0 + refIdx);
	r0 = r0*r0;
	float one_minus_cos = 1.0 - cosine;
	// Cannot use pow() here as per spec, it's undefined for x < 0
	float one_minus_cos_sq = one_minus_cos*one_minus_cos;
	return r0 + (1.0 - r0) * one_minus_cos*one_minus_cos_sq*one_minus_cos_sq;
}

#define MaterialTypeLambert	1
#define MaterialTypeMetal	2
#define MaterialTypeGlass	3
struct Material
{
	int type;
	vec3 albedo;
	float roughness;
};

const int numMaterials = 5;
#define MaterialIdxBlueLambert		0
#define MaterialIdxGreyMetal		1
#define MaterialIdxOrangeMetal2		2
#define MaterialIdxGlass			3
#define MaterialIdxOrangeLambert	4

bool materialScatterRay(int materialIndex, Ray inR, HitData hitData, vec2 uv, float times, out vec3 attenuation, out Ray outR)
{
	Material materials[numMaterials];

	materials[MaterialIdxBlueLambert].type = MaterialTypeLambert;
	materials[MaterialIdxBlueLambert].albedo = vec3(0.1, 0.2, 0.4);
	materials[MaterialIdxBlueLambert].roughness = 0.0;

	materials[MaterialIdxGreyMetal].type = MaterialTypeMetal;
	materials[MaterialIdxGreyMetal].albedo = vec3(0.45, 0.45, 0.45);
	materials[MaterialIdxGreyMetal].roughness = 0.55;

	materials[MaterialIdxOrangeLambert].type = MaterialTypeLambert;
	materials[MaterialIdxOrangeLambert].albedo = vec3(0.5, 0.45, 0.05);
	materials[MaterialIdxOrangeLambert].roughness = 0.0;

	materials[MaterialIdxOrangeMetal2].type = MaterialTypeMetal;
	materials[MaterialIdxOrangeMetal2].albedo = vec3(0.75, 0.45, 0.05);
	materials[MaterialIdxOrangeMetal2].roughness = 0.1;

	materials[MaterialIdxGlass].type = MaterialTypeGlass;
	materials[MaterialIdxGlass].albedo = vec3(0.95, 0.95, 0.95);
	materials[MaterialIdxGlass].roughness = 0.0;

	if (materialIndex >= numMaterials || materialIndex < 0)
		return false;

	int materialType = materials[materialIndex].type;
	if (materialType == MaterialTypeLambert)
	{
		vec3 target = hitData.p + hitData.n + randInUnitSphere(uv, times);
		outR.O = hitData.p;
		outR.D = target - hitData.p;
		attenuation = materials[materialIndex].albedo;
		return true;
	}
	else if (materialType == MaterialTypeMetal)
	{
		vec3 reflected = reflect(normalize(inR.D), hitData.n);
		outR.O = hitData.p;
		outR.D = reflected + materials[materialIndex].roughness*randInUnitSphere(uv, times);
		attenuation = materials[materialIndex].albedo;
		return (dot(outR.D, hitData.n) > 0);
	}
	else if (materialType == MaterialTypeGlass)
	{
		vec3 rayD_nrm = normalize(inR.D);
		vec3 reflected = reflect(rayD_nrm, hitData.n);
		float ni_over_nt;
		attenuation = materials[materialIndex].albedo;

		const float refIdx = 1.5;

		vec3 outNormal;
		float cosine;
		if (dot(rayD_nrm, hitData.n) > 0)
		{
			outNormal = -hitData.n;
			ni_over_nt = refIdx;
			cosine = refIdx*dot(rayD_nrm, hitData.n);
		}
		else
		{
			outNormal = hitData.n;
			ni_over_nt = 1.0 / refIdx;
			cosine = -dot(rayD_nrm, hitData.n);
		}

		vec3 refracted;
		float reflProb;
		if (refract(rayD_nrm, outNormal, ni_over_nt, refracted))
		{
			reflProb = schlick(cosine, refIdx);
		}
		else
		{
			reflProb = 1.0;
		}

		if (fakeRand(uv, 23.45*times) > reflProb)
		{
			outR.O = hitData.p;
			outR.D = refracted + materials[materialIndex].roughness*randInUnitSphere(uv, times);
			return true;
		}
		else
		{
			outR.O = hitData.p;
			outR.D = reflected;
			return true;
		}
	}
	return false;
}

#define HitObjectSphere		1
struct HitObject
{
	int type;
	int materialIndex;
	vec4 param0;
	vec4 param1;
};

bool hitWorld(Ray r, out HitData hitData)
{
	// We need non-zero t_min, as sometimes due to FP errors, reflecting rays will hit the same
	//	surface they were reflected from, causing artifacts and inf-loops
	const float t_min = 0.00001;
	const float t_max = 10000.0;

#define MANY_OBJECTS 0

#if (MANY_OBJECTS == 1)
	const int numHitObjects = 106;
#else
	const int numHitObjects = 6;
#endif
	HitObject hitObjects[numHitObjects];

	hitObjects[0].type = HitObjectSphere;
	hitObjects[0].materialIndex = MaterialIdxBlueLambert;
	hitObjects[0].param0 = vec4(vec3(0.0, 0.0, -1.0), 0.5);

	hitObjects[1].type = HitObjectSphere;
	hitObjects[1].materialIndex = MaterialIdxOrangeLambert;
	hitObjects[1].param0 = vec4(vec3(0.0, -100.5, -1.0), 100.0);

	hitObjects[2].type = HitObjectSphere;
	hitObjects[2].materialIndex = MaterialIdxOrangeMetal2;
	hitObjects[2].param0 = vec4(vec3( 1.0, 0.0, -1.0), 0.5);

	hitObjects[3].type = HitObjectSphere;
	hitObjects[3].materialIndex = MaterialIdxGlass;
	hitObjects[3].param0 = vec4(vec3(-1.0, 0.0, -1.0), 0.5);

	hitObjects[4].type = HitObjectSphere;
	hitObjects[4].materialIndex = MaterialIdxGreyMetal;
	hitObjects[4].param0 = vec4(vec3( 0.0, -0.5, -2.0), 0.5);

	hitObjects[5].type = HitObjectSphere;
	hitObjects[5].materialIndex = MaterialIdxGreyMetal;
	hitObjects[5].param0 = vec4(vec3( 0.0, -0.5,  0.0), 0.5);
	
#if (MANY_OBJECTS == 1)
	int offsetCnt = 0;
	for (int i = 0; i < 10; ++i)
	{
		for (int j = 0; j < 10; ++j)
		{
			int objIndex = 4+i*10+j;
			int materialIndex = MaterialIdxGlass;

			materialIndex = ((i+j+1) & 3);

			++offsetCnt;
			if (offsetCnt > 5)
				offsetCnt = 0;

			vec3 offset;
			if (offsetCnt == 0)
			{
				offset = vec3(-0.25, 0.15, 0.88);
			}
			else if (offsetCnt == 1)
			{
				offset = vec3(0.12, -0.73, -0.45);
			}
			else if (offsetCnt == 2)
			{
				offset = vec3(-0.49, 0.33, -0.57);
			}
			else if (offsetCnt == 3)
			{
				offset = vec3(0.92, -0.15, 0.41);
			}
			else if (offsetCnt == 4)
			{
				offset = vec3(0.26, 0.53, 0.29);
			}
			else
			{
				offset = vec3(-0.39, -0.25, -0.72);
			}

			if (j < 5)
				offset.z -= 0.5;
			else
				offset.z += 0.5;

			hitObjects[objIndex].type = HitObjectSphere;
			hitObjects[objIndex].materialIndex = materialIndex;
			hitObjects[objIndex].param0 = vec4(vec3((i*0.1 - 0.5)*7.5, -0.3, (j*0.1 - 0.5)*7.5) + vec3(0.3, 0.1, 0.3)*offset, 0.1);
		}
	}
#endif

	bool anyHit = false;
	HitData hitDataTemp;
	float closestHit = t_max;

	for (int i = 0; i < numHitObjects; ++i)
	{
		if (hitObjects[i].type == HitObjectSphere)
		{
			bool isSphereHit = hitSphere(hitObjects[i].param0.xyz, hitObjects[i].param0.w, hitObjects[i].materialIndex, r, t_min, closestHit, hitDataTemp);
			if (isSphereHit && (hitDataTemp.t < closestHit))
			{
				anyHit = true;
				closestHit = hitDataTemp.t;
				hitData = hitDataTemp;
			}
		}
	}

	return anyHit;
}
/* End of Hitting Routines */

vec3 getColor(Ray r, vec2 uv, float times)
{
	Ray curRay = r;
	bool needRayCast = true;
	HitData hitData;
	float recastCount = 0.0;

	vec3 dissipation = vec3(1.0, 1.0, 1.0);

	while (needRayCast)
	{
		recastCount += 1.0;
		if (recastCount > 16.0)
			break;

		bool isAnythingHit = hitWorld(curRay, hitData);
		if (isAnythingHit)
		{
			Ray inRay = curRay;
			vec3 attenuation;
			needRayCast = materialScatterRay(hitData.materialIndex, inRay, hitData, uv, times + recastCount*1000, attenuation, curRay);
			dissipation *= attenuation;
		}
		else
		{
			break;
		}
	}

	vec3 nrmD = normalize(curRay.D);
	float t = clamp(0.5*(nrmD.y + 1.0), 0.0, 1.0);
	return dissipation*((1-t)*vec3(1.0, 1.0, 1.0) + t*vec3(0.5, 0.7, 1.0)) + vec3(0.1, 0.1, 0.1);
}

void main()
{
	const float fov = 90 * (PI / 180.0);
	const float width = 800.0;
	const float height = 600.0;
	const float aspect = width/height;

	const float aperture = 0.25;

	const vec2 rndShift = vec2(fract(ubo.time*0.001), fract(ubo.time*0.0013));
	const vec3 viewup = vec3(0.0, 1.0, 0.0);
	const vec3 viewpoint = vec3(-2.0 * sin(ubo.time*0.001), 1.0, 1.0);
	const vec3 viewtarget = vec3(0.0, 0.0, -1.0);

	vec3 origin = viewpoint;

	// Basis vectors
	vec3 basZ = normalize(viewpoint - viewtarget);
	vec3 basX = normalize(cross(viewup, basZ));
	vec3 basY = normalize(cross(basZ, basX));

	// Since range is [-1; 1]
	float fov_tan = tan(fov/2.0);
	vec3 axisX = aspect*fov_tan*basX;
	vec3 axisY = fov_tan*basY;

	float lensRad = aperture / 2.0;
	float focusDist = length(viewpoint - viewtarget);

	// Lower left corner
	vec3 llc = origin - 0.5*focusDist*axisX - 0.5*focusDist*axisY - focusDist*basZ;

	const int numSubSamples = 32;
	vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
	for (int i = 0; i < numSubSamples; ++i)
	{
		vec2 rndDisk = lensRad*randOnDisk(in_texCoords.xy+rndShift, 12.3*(i+23.4));
		vec3 offset = basX*rndDisk.x + basY*rndDisk.y;
		vec3 rayTarget = llc + focusDist*(in_texCoords.x + fakeRand(in_texCoords.xy+rndShift, i)/width) * axisX + focusDist*(in_texCoords.y + fakeRand(in_texCoords.xy+rndShift, i+numSubSamples)/height) * axisY;
		Ray r = getRay(origin + offset, rayTarget - origin - offset);
		color += vec4(getColor(r, in_texCoords.xy+rndShift, i), 1.0);
	}
	
#if 0
	vec4 colorTex = texture(texSampler, in_texCoords);
	
	float interp = 0.5;
	outColor = in_color * ((1.0 - interp) * (color / numSubSamples) + interp * colorTex);
#else
	outColor = in_color * (color / numSubSamples);
#endif
}