#pragma once

#include <array>
#include <cmath>
#include <glm/glm.hpp>

class Path 
{
public:
	std::array<glm::vec3, 8> points;

	Path() 
	{
		// Create a simple path for testing purposes
		// 8 points in a zig-zag pattern, Z is 0 so elevation stays the same
		points[0] = glm::vec3(0.0f, 0.0f, 0.0f);
		points[1] = glm::vec3(6.0f, 0.0f, 0.0f);
		points[2] = glm::vec3(10.0f, 4.0f, 0.0f);
		points[3] = glm::vec3(8.0f, 10.0f, 0.0f);
		points[4] = glm::vec3(2.0f, 12.0f, 0.0f);
		points[5] = glm::vec3(-5.0f, 9.0f, 0.0f);
		points[6] = glm::vec3(-8.0f, 3.0f, 0.0f);
		points[7] = glm::vec3(-4.0f, -3.0f, 0.0f);
	}

	glm::vec3 Position(double pathParam) const;

private:
	static glm::vec3 EvaluateSegment(
		const glm::vec3& p0, 
		const glm::vec3& p1, 
		const glm::vec3& p2, 
		const glm::vec3& p3,
		float u);
};
