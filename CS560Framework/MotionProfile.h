#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

namespace MotionProfile
{
	struct Sample
	{
		double distance;
		double velocity;
	};

	// Convert normalized time [0,1] into a motion profile sample with distance and speed
	// tau = current normalized time in [0,1], a = end of acceleration, b = start of deceleration
	inline Sample Evaluate(double tau, double a, double b)
	{
		// just a check to make sure the inputs are valid
		assert(std::isfinite(tau));
		assert(0.0 < a && a < b && b < 1.0);

		// clamp time to [0,1]
		tau = (std::max)(0.0, (std::min)(1.0, tau));

		// calculate cruise speed
		// v0 is normalized speed during middle stage
		const double v0 = 2.0 / (1.0 - a + b); // v0 = 2 / (1 - a + b)

		// first branch, accel from 0 to a
		if (tau < a)
		{
			return
			{
				// distance comes from integrating velocity
				// v0/2t1*t^2
				v0* tau* tau / (2.0 * a),
				// v0t/t1
				v0* tau / a
			};
		}

		// second branch, cruise at v0 from a to b
		if (tau < b)
		{
			return
			{
				// v0(t - t1/2)
				v0 * (tau - 0.5 * a),
				v0
			};
		}

		const double remaining = 1.0 - tau;

		// third branch, decel to zero speed from time b to 1
		return
		{
			// 1 - v0(1 - t)^2 / (2(1 - t2))
			1.0 - v0 * remaining * remaining / (2.0 * (1.0 - b)),
			v0* remaining / (1.0 - b)
		};
	}

	// counts completed laps, returns total distance traveled, including completed laps
	inline double TotalDistance(double seconds, double duration,
		double length, double a, double b)
	{
		assert(seconds >= 0.0 && duration > 0.0 && length > 0.0);

		const double laps = seconds / duration;
		const double wholeLaps = std::floor(laps);
		const double tau = laps - wholeLaps;

		return length * (wholeLaps + Evaluate(tau, a, b).distance);
	}

	// reverses easing function
	inline double TimeAtDistance(double distance, double a, double b)
	{
		distance = (std::max)(0.0, (std::min)(1.0, distance));

		// handle the endpoints before binary search
		if (distance <= 0.0) return 0.0;
		if (distance >= 1.0) return 1.0;

		// use binary search to find the time that produces the requested distance
		double low = 0.0;
		double high = 1.0;

		for (int i = 0; i < 48; ++i)
		{
			const double middle = 0.5 * (low + high);

			if (Evaluate(middle, a, b).distance < distance)
			{
				low = middle;
			}
			else
			{
				high = middle;
			}
		}
		return 0.5 * (low + high);
	}
}
