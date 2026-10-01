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
		Quaternion Normalize() const; // return unit quaternion
		Quaternion Conjugate() const;
		Quaternion operator*(const Quaternion& rhs) const;
		Quaternion operator*(const glm::vec3& r) const;
		Quaternion& operator=(const Quaternion& other)
		{
			_s = other._s;
			_v = other._v;
			return *this;
		}
};

