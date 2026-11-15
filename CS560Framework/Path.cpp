#include "Path.h"
#include <algorithm>
#include <stdexcept>

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

// Build normalized segment tables and a length-weighted global table.
void Path::RebuildArcLengthTables()
{
	const int N = static_cast<int>(points.size());
	const ArcBuildSettings settings;

	// clear previous results
	segmentTables.clear();
	segmentTables.resize(N);
	pathTable.clear();
	totalLength = 0.0f;

	// Build and normalize each segments table
	for (int i = 0; i < N; ++i)
	{
		auto& segment = segmentTables[i];

		segment.entries.push_back({0.0f, 0.0f});
		segment.worldLength = 0.0f;

		// Calculate end points
		const glm::vec3 start(Position(double(i)));
		const glm::vec3 end(Position(double(i) + 1.0));

		// Pass segment index to adaptive helper function
		AppendAdaptiveArcTable(i, 0.0f, 1.0f, start, end, 0, settings, segment.entries, segment.worldLength);

		// Normalize segment distances while retaining the world-space length.
		if (segment.worldLength > 0.0f)
		{
			for (auto& entry : segment.entries)
			{
				entry.arcLength /= segment.worldLength;
			}
		}

		totalLength += segment.worldLength;
	}

	// If the entire path has zero length, there is nothing to traverse.
	// prevents division by zero during global normalization
	if (totalLength == 0.0f)
	{
		pathTable.push_back({ 0.0f, 0.0f });
		return;
	}

	// Concatenate the segment tables
	pathTable.push_back({ 0.0f, 0.0f });
	float prefixLength = 0.0f;

	for (int i = 0; i < N; ++i)
	{
		const auto& segment = segmentTables[i];

		// Convert each local entry into global entry
		for (std::size_t j = 1; j < segment.entries.size(); ++j)
		{
			const auto& entry = segment.entries[j];

			// Normalize the parameter across the entire path.
			const float q = (float(i) + entry.pathParam) / N;

			// convert local normalized distance to global
			const float s = (prefixLength + entry.arcLength * segment.worldLength) / totalLength;

			// store the pair entry
			pathTable.push_back({ q, s });
		}

		// move distance offset to next segment
		prefixLength += segment.worldLength;
	}

	pathTable.front() = {0.0f, 0.0f};
	pathTable.back() = {1.0f, 1.0f};
}

std::vector<Path::ArcEntry>::const_iterator Path::FirstEntryAfter(float value, float ArcEntry::*field) const
{
	std::size_t low = 0;
	std::size_t high = pathTable.size();

	while (low < high)
	{
		const std::size_t middle = low + (high - low) / 2;
		const float middleValue = pathTable[middle].*field;
		if (middleValue <= value)
		{
			low = middle + 1;
		}
		else
		{
			high = middle;
		}
	}

	return pathTable.begin() + low;
}

// Invert normalized cumulative distance by binary search and interpolation.
float Path::ParameterAtDistance(float distance) const
{
	// Check that a usable path exists
	if (pathTable.size() < 2 || totalLength <= 0.0f)
	{
		return 0.0f;
	}

	// restrict distance to [0,1]
	distance = std::max(0.0f, std::min(1.0f, distance));

	// Handle endpoints
	const float N = static_cast<float>(points.size());
	if (distance <= 0.0f) return 0.0f;
	if (distance >= 1.0f) return N;

	const auto hi = FirstEntryAfter(distance, &ArcEntry::arcLength);

	if (hi == pathTable.end())
	{
		return N;
	}

	const auto lo = hi - 1;
	const float span = hi->arcLength - lo->arcLength;

	// Calculate distance between the two entries
	if (span <= 0.0f)
	{
		return hi->pathParam * N;
	}

	// interpolate them
	const float alpha = (distance - lo->arcLength) / span;
	const float q = lo->pathParam + alpha * (hi->pathParam - lo->pathParam);

	return q * N;
}

float Path::NormalizedDistanceAtParameter(double parameter) const
{
	if (pathTable.size() < 2 || totalLength <= 0.0f)
		return 0.0f;

	const double N = static_cast<double>(points.size());
	if (parameter <= 0.0) return 0.0f;
	if (parameter >= N) return 1.0f;
	const float q = static_cast<float>(parameter / N);
	const auto hi = FirstEntryAfter(q, &ArcEntry::pathParam);
	if (hi == pathTable.end()) return 1.0f;
	const auto lo = hi - 1;
	const float alpha = (q - lo->pathParam) / (hi->pathParam - lo->pathParam);
	return lo->arcLength + alpha * (hi->arcLength - lo->arcLength);
}

void Path::AppendAdaptiveArcTable(int segment, float a, float b,
	const glm::vec3& pa, const glm::vec3& pb, int depth,
	const ArcBuildSettings& settings, std::vector<ArcEntry>& table, float& cumulativeLength) const
{
	// Evaluate the midpoint at (a + b) / 2 within this segment.
	const float mid = 0.5f * (a + b);
	const glm::vec3 pm(Position(double(segment) + mid));

	// estimate lengths by linear distances
	// P(um) - P(ua) etc.
	const float A = glm::length(pm - pa);
	const float B = glm::length(pb - pm);
	const float C = glm::length(pb - pa);

	// estimate if more subdivision is needed
	// d = A + B - C
	const float error = std::max(0.0f, A + B - C);
	const float span = b - a;

	// apply subdivision conditions, split if any condition is true
	const bool split = span > settings.maxParameterSpan || // too wide
		error > settings.lengthTolerance * span || // length error too large
		std::abs(A - B) > settings.mappingTolerance; // param halves cover very different distances,
													 // improves inverse lookup later


	if (split)
	{
		// if need be, recursively process both halves
		if (depth >= settings.maxDepth)
		{
			throw std::runtime_error("Arc-length table exceeded subdivision limit");
		}

		// distance matters here, left needs to be processed first so the distance is already accumulated
		AppendAdaptiveArcTable(segment, a, mid, pa, pm, depth + 1, settings, table, cumulativeLength);
		AppendAdaptiveArcTable(segment, mid, b, pm, pb, depth + 1, settings, table, cumulativeLength);

		return;
	}

	// accept approximation and append entries
	cumulativeLength += A;
	table.push_back({mid, cumulativeLength});

	cumulativeLength += B;
	table.push_back({ b, cumulativeLength });
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
