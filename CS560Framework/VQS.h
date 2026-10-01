#pragma once

#include <glm/glm.hpp>
#include "Quaternion.h"

class VQS
{
	public:
		VQS();
		VQS(const glm::vec3& translation, const Quaternion& rotation, float scale);

		glm::mat4 ToMatrix() const;
		glm::vec3 TransformPoint(const glm::vec3 point) const;
		
		VQS Inverse() const;
		VQS operator*(const VQS& rhs) const;

		static VQS Interpolate(const VQS& a, const VQS& b, float t);

		glm::vec3 _translation{ 0.0f };
		Quaternion _rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
		float _scale{ 1.0f };
};

