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

float Quaternion::Dot(Quaternion& q1, Quaternion& q2) const
{

	return q1._s*q2._s + glm::dot(q1._v, q2._v);
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

QuatHelper Quaternion::PrepareSlerp(Quaternion q1, Quaternion q2) const
{
	q1 = q1.Normalize();
	q2 = q2.Normalize();

	float d = Dot(q1, q2);
	if (d < 0.0f)
	{
		// Flip q2 to take the shorter path.
		q2 = q2 * -1.0f;
		d = -d;
	}

	return {q1, q2, glm::clamp(d, 0.0f, 1.0f)};
}

Quaternion Quaternion::Slerp(Quaternion start, Quaternion end, float u) const
{
	auto [q1, q2, d] = PrepareSlerp(start, end);

	// Use normalized linear interpolation for nearly identical inputs.
	if (d > 0.9995f)
    {
        return ((1.0f - u) * q1 + u * q2).Normalize();
    }

    const float theta = glm::acos(d);
	return (glm::sin((1.0f - u) * theta) * q1
		+ glm::sin(u * theta) * q2) / glm::sin(theta);
}

Quaternion Quaternion::BezierDeCasteljau(const Quaternion& q0, const Quaternion& q1,
	const Quaternion& q2, const Quaternion& q3, float u) const
{
	Quaternion a = Slerp(q0, q1, u);
	Quaternion b = Slerp(q1, q2, u);
	Quaternion c = Slerp(q2, q3, u);

	Quaternion d = Slerp(a, b, u);
	Quaternion e = Slerp(b, c, u);

	return Slerp(d, e, u).Normalize();
}

Quaternion Quaternion::ISlerp(
    Quaternion start, Quaternion end, int n, int k, ISlerpState& state) const
{
    if (n < 1 || k < 0 || k > n)
    {
        throw std::invalid_argument("ISlerp requires n >= 1 and 0 <= k <= n.");
    }

    auto [q1, q2, d] = PrepareSlerp(start, end);
    if (k == 0)
    {
        state.initialized = false;
        return q1;
    }
    if (k == n)
    {
        return q2;
    }

    const float u = float(k) / float(n);
    if (d > 0.9995f)
    {
        return ((1.0f - u) * q1 + u * q2).Normalize();
    }

    // Initialize once, or restart when sampling backwards.
    if (!state.initialized || k < state.step)
    {
        const double a = std::acos(double(d));
        const double b = a / n;
        const double cosB = std::cos(b);
        const double sinB = std::sin(b);
        const double sinA = std::sin(a);
        state.coefficient = 2.0 * cosB; // Big A

		// save cacluations
        const double first[4] = {q1._s, q1._x, q1._y, q1._z};
        const double last[4] = {q2._s, q2._x, q2._y, q2._z};
        for (int i = 0; i < 4; ++i)
        {
            state.previous[i] = first[i];
            const double tangent = (last[i] - double(d) * first[i]) / sinA;
            state.current[i] = cosB * first[i] + sinB * tangent;
        }
        state.step = 1;
        state.initialized = true;
    }

    // Continue from the last computed sample.
    while (state.step < k)
    {
        for (int i = 0; i < 4; ++i)
        {
            const double next =
                state.coefficient * state.current[i] - state.previous[i];
            state.previous[i] = state.current[i];
            state.current[i] = next;
        }
        ++state.step;
    }

    return Quaternion(float(state.current[0]), float(state.current[1]),
        float(state.current[2]), float(state.current[3])).Normalize();
}

Quaternion Quaternion::operator+(const Quaternion& q2) const
{
	float scaler = _s + q2._s;
	glm::vec3 vector = _v + q2._v;
	return Quaternion(scaler, vector);
}

Quaternion Quaternion::operator-(const Quaternion& q2) const
{
	float scaler = _s - q2._s;
	glm::vec3 vector = _v - q2._v;
	return Quaternion(scaler, vector);
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

Quaternion Quaternion::operator*(const float c) const
{
	return Quaternion(c * _s, c * _v);
}

Quaternion Quaternion::operator/(const float c) const
{
	return Quaternion(_s / c, _v / c);
}
