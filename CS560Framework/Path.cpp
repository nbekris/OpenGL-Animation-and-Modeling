#include "Path.h"

// Convert a position along the path into a 3D coordinate
glm::vec3 Path::Position(double pathParam) const
{
	const int numControlPoints = static_cast<int>(points.size());

	// Wrap so that model can loop the path
	double wrappedParam = std::fmod(pathParam, double(numControlPoints));
	if (wrappedParam < 0.0) wrappedParam += numControlPoints;

	const int segment = static_cast<int>(std::floor(wrappedParam));
	const float u = static_cast<float>(wrappedParam - segment);

	// helper for accessing control points with wrapping
	const auto point = [&](int index) -> const glm::vec3& {
		return points[(index + numControlPoints) % numControlPoints];
	};

	return EvaluateSegment(
		point(segment - 1), 
		point(segment), 
		point(segment + 1), 
		point(segment + 2), u);
}

// Calculate the position on the path segment based on path param u
glm::vec3 Path::EvaluateSegment(const glm::vec3& p0, 
	const glm::vec3& p1, const glm::vec3& p2, 
	const glm::vec3& p3, float u)
{
	// build cubic coefficients
	const glm::vec3 a = 2.0f * p1;
	const glm::vec3 b = -p0 + p2;
	const glm::vec3 c = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
	const glm::vec3 d = -p0 + 3.0f * p1 - 3.0f * p2 + p3;

	// 1/2(a + bu + cu^2 + du^3)
	return 0.5f * (a + b * u + c * u*u + d * u*u*u);
}
