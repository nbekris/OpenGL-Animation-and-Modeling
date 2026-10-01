#include "VQS.h"
#include "fwd.hpp"

VQS::VQS()
{}

VQS::VQS(const glm::vec3 & translation, const Quaternion& rotation, float scale)
{
	_translation = translation;
	_rotation = rotation;
	_scale = scale;
}

glm::mat4 VQS::ToMatrix() const
{
	return glm::mat4();
}

glm::vec3 VQS::TransformPoint(const glm::vec3 point) const
{
	Quaternion q = _rotation.Normalize();
	Quaternion qInverse = _rotation.Conjugate();
	glm::vec3 v = _translation;
	glm::vec3 r = point;
	float s = _scale;

	// r' = [v,q,s]r = q(sr)q-1 + v
	Quaternion rPrime = q * (s * r) * qInverse;

	return glm::vec3{ rPrime._x, rPrime._y, rPrime._z } + v;
}

VQS VQS::Inverse() const
{
	return VQS();
}

VQS VQS::operator*(const VQS& rhs) const
{
	//glm::vec3 u = rhs._translation;
	//Quaternion p = rhs._rotation;
	//float t = rhs._scale;

	//glm::vec3 v = this->_translation;
	//Quaternion q = this->_rotation;
	//float s = this->_scale;

	//Quaternion rPrime = 
	return VQS();
}

VQS VQS::Interpolate(const VQS& a, const VQS& b, float t)
{
	return VQS();
}
