#pragma once

#include <math.h>
#include "math/AuxMath.h"
#include "math/Vec3.h"
#include "math/Mat34.h"

#include <stdio.h>

class Camera
{
protected:

	math::Vec3 m_pos = math::Vec3C(0.0f, 0.0f, 0.0f);
	math::Vec3 m_up = math::Vec3C(0.0f, 1.0f, 0.0f), m_view = math::Vec3C(0.0f, 0.0f, -1.0f);

	float m_angX = 0.0f, m_angY = 0.0f;

public:

	void setPosition(const math::Vec3 & pos) { m_pos = pos; }

	math::Vec3 getPosition() const { return m_pos; }
	math::Vec3 getUp() const { return m_up; }
	math::Vec3 getView() const { return m_view; }

	void update(const math::Vec3 & posOffset, float rotX, float rotY);
	void fillMatrix(math::Mat34 * mat) const;
};