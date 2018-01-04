#include "camera.h"

void Camera::update(const math::Vec3 & posOffset, float rotX, float rotY)
{
	using namespace math;

	m_angX += rotX;
	m_angY += rotY;

	const float angYLimit = 0.99f * _PI2;
	if (m_angY > angYLimit)
		m_angY = angYLimit;
	else if (m_angY < -angYLimit)
		m_angY = -angYLimit;

	float angX = m_angX;
	float angY = m_angY;

	const bool invert = false;
	if (!invert)
	{
		angX = -angX;
		angY = -angY;
	}
	angY += _PI2;

	m_view = Vec3C(sinf(angX)*sinf(angY), cosf(angY), cosf(angX)*sinf(angY));
	m_view.normalize();

	Vec3 right = m_up.cross(m_view);
	right.normalize();

	Vec3 up = m_view.cross(right);
	up.normalize();

	m_pos += right * posOffset.x + up * posOffset.y + m_view * posOffset.z;
}

void Camera::fillMatrix(math::Mat34 * mat) const
{
	if (!mat)
		return;

	using namespace math;

	Vec3 right = m_up.cross(m_view);
	right.normalize();

	Vec3 up = m_view.cross(right);
	up.normalize();

	mat->setBasis0(right);
	mat->setBasis1(up);
	mat->setBasis2(m_view);
	mat->setBasis3(m_pos);

	*mat = mat->invertRTCopy();
}
