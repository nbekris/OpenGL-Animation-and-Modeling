#pragma once

#include <glm/glm.hpp>

class Quaternion
{
	public:

		float _s;
		glm::vec3 _v;

		float& _x = _v.x;
		float& _y = _v.y;
		float& _z = _v.z;

		Quaternion(float scaler, glm::vec3 vector) 
			: _s(scaler), _v(vector) {}

		Quaternion(float scaler, float x, float y, float z)
			: _s(scaler), _v(x, y, z) {}

		Quaternion(const Quaternion& other)
			: _s(other._s), _v(other._v) {}

		glm::mat4 ToMatrix() const;
		float Dot(Quaternion& q1, Quaternion& q2) const;
		Quaternion Normalize() const; // return unit quaternion
		Quaternion Conjugate() const;
		Quaternion Inverse() const;
		Quaternion Slerp(Quaternion q1, Quaternion q2, float t) const;
		// Spherical cubic Bezier: q0/q3 are endpoints, q1/q2 are controls; u is in [0, 1].
		Quaternion BezierDeCasteljau(const Quaternion& q0, const Quaternion& q1,
			const Quaternion& q2, const Quaternion& q3, float u) const;
		Quaternion ISlerpCheb(Quaternion q1, Quaternion q2, float n) const;

		Quaternion operator+(const Quaternion& q2) const;
		Quaternion operator*(const Quaternion& rhs) const;
		Quaternion operator*(const glm::vec3& r) const;
		Quaternion operator*(const float c) const;
		Quaternion& operator=(const Quaternion& other)
		{
			_s = other._s;
			_v = other._v;
			return *this;
		}

		friend Quaternion operator*(float c, const Quaternion& q)
		{
			return q * c;
		}
};

