#include "Quaternion.h"
#include <cmath>
#include <stdexcept>

glm::mat4 Quaternion::ToMatrix() const
{
	// Make sure we are working with a unit quaternion
	const Quaternion q = Normalize();
	const float s = q._s;
	const float x = q._x;
	const float y = q._y;
	const float z = q._z;

	glm::mat4 matrix{1.0f};

	// glm does column row indexing
	matrix[0][0] = 1.0f - 2.0f * (y*y + z*z);
	matrix[1][0] = 2.0f * (x*y - s*z);
	matrix[2][0] = 2.0f * (x*z + s*y);

	matrix[0][1] = 2.0f * (x*y + s*z);
	matrix[1][1] = 1.0f - 2.0f * (x*x + z*z);
	matrix[2][1] = 2.0f * (y*z - s*x);

	matrix[0][2] = 2.0f * (x*z - s*y);
	matrix[1][2] = 2.0f * (y*z + s*x);
	matrix[2][2] = 1.0f - 2.0f * (x*x + y*y);

	return matrix;
}

Quaternion Quaternion::Normalize() const
{
	float magnitude = std::sqrt(_s*_s + _x*_x + _y*_y + _z*_z);

	if (magnitude == 0.0f)
	{
		throw std::domain_error("Cannot normalize a zero quaternion.");
	}

	float s = _s / magnitude;
	glm::vec3 v{ _x / magnitude, _y / magnitude, _z / magnitude };

	return Quaternion{ s, v };
}

Quaternion Quaternion::Conjugate() const
{
	return Quaternion{ _s, -_x, -_y, -_z };
}

Quaternion Quaternion::Inverse() const
{
	this->Normalize();
	this->Conjugate();
	return Quaternion(*this);
}

Quaternion Quaternion::operator*(const Quaternion& q1) const
{
	float s1 = _s;
	float s2 = q1._s;
	glm::vec3 v1{_x,_y,_z};
	glm::vec3 v2{q1._x, q1._y, q1._z};

	float s = s1*s2 - glm::dot(v1, v2);
	glm::vec3 v = s1*v2 + s2*v1 + glm::cross(v1, v2);

	return Quaternion(s, v);
}

Quaternion Quaternion::operator*(const glm::vec3& r) const
{
	const Quaternion q = Normalize();
	glm::vec3 vector =
		(q._s*q._s - glm::dot(q._v, q._v)) * r
		+ 2.0f * glm::dot(q._v, r) * q._v
		+ 2.0f * q._s * glm::cross(q._v, r);
	return Quaternion(0.0f, vector);
}
