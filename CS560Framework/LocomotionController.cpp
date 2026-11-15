#include "LocomotionController.h"
#include "MotionProfile.h"
#include "Path.h"
#include <algorithm>
#include <cmath>

namespace
{
    // convert world distance to a position, wrapping at the end of the closed path
    glm::vec3 PointAtDistance(const Path& path, double distance)
    {
        const double length = path.TotalLength();
        double wrapped = std::fmod(distance, length);
        if (wrapped < 0.0)
            wrapped += length;
        return path.Position(path.ParameterAtDistance(static_cast<float>(wrapped / length)));
    }
}

void LocomotionController::SetPlaying(bool playing)
{
    m_playing = playing;
    if (!playing)
        m_sample.speed = 0.0;
}

bool LocomotionController::SetSettings(const LocomotionSettings& settings, const Path& path)
{
    if (!std::isfinite(settings.loopSeconds) || settings.loopSeconds <= 0.0 ||
        !(0.0 < settings.easeInEnd && settings.easeInEnd < settings.easeOutStart && settings.easeOutStart < 1.0) ||
        !std::isfinite(settings.strideWorld) || settings.strideWorld <= 0.0f ||
        !std::isfinite(settings.lookAheadWorld) || settings.lookAheadWorld <= 0.0f ||
        !std::isfinite(settings.assetForwardYawDegrees))
        return false;

    const bool profileChanged = settings.loopSeconds != m_settings.loopSeconds ||
        settings.easeInEnd != m_settings.easeInEnd || settings.easeOutStart != m_settings.easeOutStart;
    m_settings = settings;
    // changing pace or heading should not reset the gait phase
    if (profileChanged)
        RebaseTravelClock(path);
    Update(0.0, path);
    return true;
}

// Keep the current position when changing the duration or easing function
void LocomotionController::RebaseTravelClock(const Path& path)
{
    const double length = path.TotalLength();
    if (!std::isfinite(length) || length <= 1e-6)
    {
        m_travelSeconds = m_sample.distance = 0.0;
        return;
    }
    // t = T * S^-1(s), using binary search from MotionProfile
    m_travelSeconds = m_settings.loopSeconds * MotionProfile::TimeAtDistance(
        m_sample.distance / length, m_settings.easeInEnd, m_settings.easeOutStart);
}

void LocomotionController::SeekDistance(double distance, const Path& path)
{
    if (!std::isfinite(distance))
        return;
    m_sample.distance = (std::max)(0.0, (std::min)(double(path.TotalLength()), distance));
    RebaseTravelClock(path);
    // scrubbing chooses a pose, it should not count as distance traveled this frame
    m_sample.gaitPhase = m_sample.distance / m_settings.strideWorld;
    m_sample.gaitPhase -= std::floor(m_sample.gaitPhase);
    Update(0.0, path);
}

void LocomotionController::Restart(const Path& path)
{
    SeekDistance(0.0, path);
}

LocomotionSample LocomotionController::Update(double deltaSeconds, const Path& path)
{
    m_sample.speed = 0.0;
    const double length = path.TotalLength();
    if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0 ||
        !std::isfinite(length) || length <= 1e-6 ||
        !std::isfinite(m_travelSeconds + deltaSeconds))
        return m_sample;

    const double oldDistance = MotionProfile::TotalDistance(m_travelSeconds,
        m_settings.loopSeconds, length, m_settings.easeInEnd, m_settings.easeOutStart);
    if (m_playing)
        m_travelSeconds += deltaSeconds;
    const double newDistance = MotionProfile::TotalDistance(m_travelSeconds,
        m_settings.loopSeconds, length, m_settings.easeInEnd, m_settings.easeOutStart);

    // keep completed laps in the subtraction so crossing the seam is still forward motion
    const double deltaDistance = (std::max)(0.0, newDistance - oldDistance);
    const double laps = m_travelSeconds / m_settings.loopSeconds;
    const auto sample = MotionProfile::Evaluate(laps - std::floor(laps),
        m_settings.easeInEnd, m_settings.easeOutStart);
    // leave a scrub at distance 1 until time actually advances
    if (m_playing && deltaSeconds > 0.0)
        m_sample.distance = length * sample.distance;
    if (m_playing)
        m_sample.speed = length / m_settings.loopSeconds * sample.velocity;

    // v = P*n, so the number of cycles this frame is distance / stride
    // SpeedControl ties the feet's pace to movement along the path
    m_sample.gaitPhase += deltaDistance / m_settings.strideWorld;
    m_sample.gaitPhase -= std::floor(m_sample.gaitPhase);
    SamplePath(path);
    return m_sample;
}

void LocomotionController::SamplePath(const Path& path)
{
    const double length = path.TotalLength();
    // S(t) -> arc length lookup -> P(u), SpeedControl
    m_sample.pathParam = path.ParameterAtDistance(static_cast<float>(m_sample.distance / length));
    m_sample.position = path.Position(m_sample.pathParam);

    // Forward Center of Interest, OrientationControl
    // W = P(s + lookAhead) - P(s), using arc length instead of the spline parameter
    const double lookAhead = (std::min)(double(m_settings.lookAheadWorld), 0.05 * length);
    glm::vec3 forward = PointAtDistance(path, m_sample.distance + lookAhead) - m_sample.position;
    forward.z = 0.0f;
    if (glm::dot(forward, forward) > 1e-10f)
    {
        // our path is in XY, so use Z for up and atan2 for the heading
        constexpr double pi = 3.14159265358979323846;
        m_headingDegrees = static_cast<float>(std::atan2(forward.y, forward.x) * 180.0 / pi);
    }
    m_sample.yawDegrees = m_headingDegrees - m_settings.assetForwardYawDegrees;
}
