#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <glm/glm.hpp>

class Path
{
	struct ArcBuildSettings
	{
		float lengthTolerance = 1e-4f; // 0.0001 units of length
		float mappingTolerance = 1e-4f;
		float maxParameterSpan = 1.0f / 32.0f;
		int maxDepth = 20;
	};

	struct ArcEntry
	{
		float pathParam; // Local t or global q, depending on table
		float arcLength; // Cumulative distance; normalized after building
	};

	struct SegmentArcTable
	{
		std::vector<ArcEntry> entries;
		float worldLength = 0.0f;
	};

public:

	Path()
	{
		// Create a simple path for testing purposes
		// Eight control points define a closed path in the XY plane.
		points[0] = glm::vec3(0.0f, 0.0f, 0.0f);
		points[1] = glm::vec3(6.0f, 0.0f, 0.0f);
		points[2] = glm::vec3(10.0f, 4.0f, 0.0f);
		points[3] = glm::vec3(8.0f, 10.0f, 0.0f);
		points[4] = glm::vec3(2.0f, 12.0f, 0.0f);
		points[5] = glm::vec3(-5.0f, 9.0f, 0.0f);
		points[6] = glm::vec3(-8.0f, 3.0f, 0.0f);
		points[7] = glm::vec3(-4.0f, -3.0f, 0.0f);
		for (auto& point : points) point *= 4.5f;
		RebuildArcLengthTables();
	}

	glm::vec3 Position(double pathParam) const;

    bool closed = true;

    // Returns the number of spline segments for the current topology.
    int SegmentCount() const
    {
        return static_cast<int>(points.size()) - (closed ? 0 : 1);
    }

	// Call again after changing control points.
	void RebuildArcLengthTables();
	float TotalLength() const { return totalLength; }
	// Convert normalized distance [0,1] to evaluator parameter [0,SegmentCount()].
	float ParameterAtDistance(float distance) const;
	// Convert evaluator parameter [0,SegmentCount()] to normalized distance [0,1].
	float NormalizedDistanceAtParameter(double parameter) const;

private:

	void AppendAdaptiveArcTable(
		int segment, float a, float b,
		const glm::vec3& pa, const glm::vec3& pb, int depth,
		const ArcBuildSettings& settings, std::vector<ArcEntry>& table, float& cumulativeLength) const;

	static glm::vec3 EvaluateSegment(
		const glm::vec3& p0,
		const glm::vec3& p1,
		const glm::vec3& p2,
		const glm::vec3& p3,
		float u);

public:
	std::array<glm::vec3, 8> points;
private:
	std::vector<SegmentArcTable> segmentTables;
	std::vector<ArcEntry> pathTable;
	std::vector<ArcEntry>::const_iterator FirstEntryAfter(float value, float ArcEntry::*field) const;
	float totalLength = 0.0;
};
