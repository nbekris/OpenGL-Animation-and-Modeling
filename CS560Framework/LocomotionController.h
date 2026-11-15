#pragma once

#include <glm/glm.hpp>

class Path;

struct LocomotionSettings
{
    double loopSeconds = 16.0;
    double easeInEnd = 0.2;
    double easeOutStart = 0.8;
    float strideWorld = 11.7f;
    float lookAheadWorld = 0.75f;
    float assetForwardYawDegrees = -90.0f;
};

struct LocomotionSample
{
    glm::vec3 position{ 0.0f };
    float yawDegrees = 0.0f;
    double gaitPhase = 0.0;
    double speed = 0.0;
    double distance = 0.0;
    double pathParam = 0.0;
};

// Controls path travel and gait timing without depending on the model or UI
class LocomotionController
{
public:
    const LocomotionSettings& GetSettings() const { return m_settings; }
    const LocomotionSample& GetSample() const { return m_sample; }
    bool IsPlaying() const { return m_playing; }
    void SetPlaying(bool playing);
    bool SetSettings(const LocomotionSettings& settings, const Path& path);
    // seeking changes the position and pose, but keeps the current play/pause state
    void SeekDistance(double distance, const Path& path);
    void Restart(const Path& path);
    LocomotionSample Update(double deltaSeconds, const Path& path);

private:
    void RebaseTravelClock(const Path& path);
    void SamplePath(const Path& path);

    LocomotionSettings m_settings;
    LocomotionSample m_sample;
    bool m_playing = true;
    double m_travelSeconds = 0.0;
    float m_headingDegrees = 0.0f;
};
