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

VQS VQS::Interpolate(const VQS& previous, const VQS& start,
    const VQS& end, const VQS& next, float t)
{
    return Interpolate(previous, start, end, next, t, t, t);
}

VQS VQS::Interpolate(const VQS& previous, const VQS& start,
    const VQS& end, const VQS& next, float translationT, float rotationT, float scaleT)
{
    const glm::vec3 translation = BezierTranslation(previous._translation,
        start._translation, end._translation, next._translation, translationT);

    // Interpolate the source keyframe rotations.
    Quaternion startRotation = start._rotation;
    Quaternion endRotation = end._rotation;
    const Quaternion rotation = start._rotation.Slerp(startRotation, endRotation, rotationT);

    const float scale = (1.0f - scaleT) * start._scale + scaleT * end._scale;
    return VQS(translation, rotation, scale);
}

glm::vec3 VQS::BezierTranslation(const glm::vec3& previous, const glm::vec3& start,
	const glm::vec3& end, const glm::vec3& next, float u)
{
	// Insert a_i and b_(i+1) using the neighboring-keyframe rule.
	const glm::vec3 a = start + (end - previous) * 0.5f;
	const glm::vec3 b = end - (next - start) * 0.5f;

	// Equation Q: interpolate the four Bezier controls.
	const glm::vec3 q0 = start + (a - start) * u;
	const glm::vec3 q1 = a + (b - a) * u;
	const glm::vec3 q2 = b + (end - b) * u;

	// Equation R: interpolate the three first-level results.
	const glm::vec3 r0 = q0 + (q1 - q0) * u;
	const glm::vec3 r1 = q1 + (q2 - q1) * u;

	// Equation S: return the position on the curve.
	return r0 + (r1 - r0) * u;
}
