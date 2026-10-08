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
		// Interpolates start to end; previous/next affect only the Bezier translation.
		// t is in [0, 1]; callers supply boundary neighbors as for BezierTranslation.
		static VQS Interpolate(const VQS& previous, const VQS& start,
			const VQS& end, const VQS& next, float t);
		// Independent factors for animation tracks with different key times.
		static VQS Interpolate(const VQS& previous, const VQS& start,
			const VQS& end, const VQS& next, float translationT, float rotationT, float scaleT);
		// Interpolates start to end using neighboring keyframe positions (u in [0, 1]).
		// Uses the handout's control offsets; C1 in time assumes equal segment durations.
		// Caller supplies neighbors, including an endpoint policy for missing keys.
		static glm::vec3 BezierTranslation(const glm::vec3& previous, const glm::vec3& start,
			const glm::vec3& end, const glm::vec3& next, float u);

		glm::vec3 _translation{ 0.0f };
		Quaternion _rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
		float _scale{ 1.0f };
};

