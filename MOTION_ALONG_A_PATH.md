# Motion along a closed path

Each frame, the controller converts elapsed time into a fraction of the lap distance, looks up the corresponding spline parameter, and evaluates the character's position. A second sample farther along the path determines which way the character faces.

## Path interpolation

The path is a closed, uniform Catmull–Rom spline. For N control points, the integer part of the wrapped parameter selects segment i, and the fractional part u gives progress within it. Segment i runs from point P1 to P2. The preceding point P0 and following point P3 determine its endpoint tangents. All four indices wrap around the loop.

The segment is evaluated as a cubic polynomial in 3D:

~~~text
P(u) = 0.5 * (2 P1 + (-P0 + P2) u
              + (2 P0 - 5 P1 + 4 P2 - P3) u²
              + (-P0 + 3 P1 - 3 P2 + P3) u³)
~~~

At u = 0 the curve passes through P1, and at u = 1 it reaches P2. Adjacent cubics share the same tangent at their common point, including where the final segment rejoins the first. Equal increments in u can cover different world distances, even within a single segment. [Path::Position and EvaluateSegment](CS560Framework/Path.cpp)

The four-point cubic expands the Catmull–Rom basis on page 5. The project wraps the indices to close the curve. [Interpolation.pdf, p. 5](file:///G:/MastersHW/CS560/Interpolation.pdf#page=5)

## Arc length calculation

For each segment, the table builder samples the two endpoints and the parameter midpoint. Let A and B be the lengths of the two half-chords and C the length of the direct endpoint chord. It recursively halves the interval if any of three conditions holds:

- The parameter interval exceeds the maximum span.
- The excess length A + B − C exceeds the length tolerance times the interval span. This detects bending that a single chord would miss.
- The difference |A − B| exceeds the mapping tolerance. Even on a straight section, unequal half-chords mean the parameter midpoint is a poor estimate of the distance midpoint.

The midpoint chord test follows the adaptive subdivision algorithm. The project adds the maximum-span and half-chord imbalance checks. [ArcLength.pdf, pp. 5–6](file:///C:/Users/bekri/Downloads/ArcLength.pdf#page=5)

For an accepted interval, the builder adds A and then B to the running length, recording the cumulative distance at the midpoint and endpoint. Processing the left half first keeps the samples in traversal order. Each segment retains its estimated world length. The builder combines the segment tables into one table of normalized parameter and normalized distance, weighting each segment by its share of the total length.

To place the character at a requested distance fraction, ParameterAtDistance uses binary search to find the first entry above that distance. It linearly interpolates the neighboring parameter values and multiplies by N to recover the spline parameter. Thus halfway around the loop means half the estimated world distance, regardless of how the control points are spaced. [Arc table construction and lookup](CS560Framework/Path.cpp)

The inverse lookup follows the binary search and interpolation example. The length-weighted global table applies the method for concatenating curve segments. [ArcLength.pdf, pp. 4–5](file:///C:/Users/bekri/Downloads/ArcLength.pdf#page=4) · [SpeedControl.pdf, p. 17](file:///C:/Users/bekri/Downloads/SpeedControl.pdf#page=17)

## Speed and orientation control

### Speed profile

Let t be the fraction of the current lap's duration, a the end of acceleration, and b the start of deceleration. The normalized cruise speed is v = 2 / (1 − a + b), which makes the area under the velocity curve equal to one lap.

| Phase | Normalized velocity | Distance fraction |
| --- | --- | --- |
| Accelerate, 0 ≤ t < a | v t / a | v t² / (2a) |
| Cruise, a ≤ t < b | v | v (t − a/2) |
| Decelerate, b ≤ t ≤ 1 | v (1 − t) / (1 − b) | 1 − v (1 − t)² / (2(1 − b)) |

The table uses the piecewise-linear velocity profile and its integrated distance function. Here a and b correspond to the lecture parameters t1 and t2. [SpeedControl.pdf, pp. 9–11](file:///C:/Users/bekri/Downloads/SpeedControl.pdf#page=9)

The controller scales the distance fraction by path length and the normalized velocity by path length divided by lap duration. When the user seeks or changes the timing settings, it inverts the distance profile by bisection to find the time corresponding to the selected position. [MotionProfile](CS560Framework/MotionProfile.h)

The walk cycle advances by world distance traveled during the frame divided by world distance per stride. The distance difference includes completed laps, so a frame that crosses the loop boundary still advances the gait by the distance actually traveled. [LocomotionController::Update](CS560Framework/LocomotionController.cpp)

Advancing gait phase by distance divided by stride implements the relationship between travel speed, stride length, and animation-cycle frequency. [SpeedControl.pdf, p. 15](file:///C:/Users/bekri/Downloads/SpeedControl.pdf#page=15)

### Facing direction

The controller samples a target ahead of the current position by a configurable world distance capped at 5% of the loop length. The target distance wraps across the seam. Subtracting the current position from the target gives a forward vector, which the controller projects onto the XY plane. It computes heading in degrees with atan2(y, x), then subtracts the asset's forward-yaw offset to align the model with that heading. If the projected vector is too short to define a direction, the controller keeps its previous heading. This produces yaw about the Z axis. [LocomotionController::SamplePath](CS560Framework/LocomotionController.cpp)

The heading uses the forward center-of-interest method. The project chooses the target by world-space arc distance and converts the resulting direction to planar yaw with a model-specific offset. [OrientationControl.pdf, p. 3](file:///C:/Users/bekri/Downloads/OrientationControl.pdf#page=3)
