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
	glm::mat4 matrix = _rotation.ToMatrix();
	matrix[3] = glm::vec4(_translation, 1.0f);
	matrix[0] *= _scale;
	matrix[1] *= _scale;
	matrix[2] *= _scale;
	return matrix;
}

glm::vec3 VQS::TransformPoint(const glm::vec3 point) const
{
	Quaternion q = _rotation.Normalize();
	Quaternion qInverse = q.Conjugate();
	glm::vec3 v = _translation;
	glm::vec3 r = point;
	float s = _scale;

	// r' = [v,q,s]r = q(sr)q^-1 + v
	Quaternion rPrime = q * (s * r)/* * qInverse*/;

	return glm::vec3{ rPrime._x, rPrime._y, rPrime._z } + v;
}

VQS VQS::Inverse() const
{
	return VQS();
}

VQS VQS::operator*(const VQS& rhs) const
{
	// [[u, p, t]v, pq, ts] - VQS concatenation
	// [u, p, t]v = q(sr)q^-1 + v so we can use TransformPoint()
	glm::vec3 uptv = TransformPoint(rhs._translation);
	Quaternion pq = _rotation * rhs._rotation;
	float ts = _scale * rhs._scale;

	return VQS(uptv, pq, ts);
}

VQS VQS::Interpolate(const VQS& a, const VQS& b, float t)
{
	return VQS();
}
