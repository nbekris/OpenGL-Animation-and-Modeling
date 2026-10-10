#pragma once
#include "AnimationData.h"
#include "InterpolationMode.h"
#include <map>
#include <string>

struct KeyIndices
{
	unsigned int previous;
	unsigned int next;
};

struct AnimationRange
{
    double startTicks = 0.0;
    double endTicks = 0.0; // Exclusive loop boundary.
};

// Borrows engine-owned animation data; reset before its owner replaces it.
// No rendering dependencies. Advancing time and evaluating a pose are separate.
class Animator
{
public:
    void Reset(const AnimationData* data = nullptr);
    void Update(double deltaSeconds);
    void Play() { m_playing = true; }
    void Pause() { m_playing = false; }
    bool IsPlaying() const { return m_playing; }
    bool SetAnimation(unsigned int index);
    bool SetPlaybackRange(AnimationRange range);
    bool SetPlaybackSpeed(double multiplier);
    double GetPlaybackSpeed() const { return m_playbackSpeed; }
    void SetInterpolationMode(InterpolationMode mode) { m_interpolationMode = mode; }
    InterpolationMode GetInterpolationMode() const { return m_interpolationMode; }

    unsigned int GetAnimationIndex() const { return m_animationIndex; }
    unsigned int GetAnimationCount() const;

    std::string GetAnimationName(unsigned int index) const;

    // Elapsed seconds within the selected playback range.
    double GetPlaybackSeconds() const { return m_playbackSeconds; }
    double GetSampleTimeTicks() const;
    double GetFirstKeyTimeTicks() const;
    void EvaluatePose(double timeTicks);

    const std::map<std::string, glm::mat4>& GetNodeTransforms() const { return m_nodeTransforms; }
private:
    double GetTicksPerSecond() const;
    AnimationRange GetPlaybackRange() const;
    const AnimationClip* GetAnimation() const;
    const AnimationChannel* FindAnimationChannel(const AnimationClip* animation, const std::string& nodeName) const;
    glm::mat4 SampleLocalTransform(const SkeletonNode* node, const AnimationChannel* channel, double animationTime) const;
    void EvaluateNode(const SkeletonNode* node, const glm::mat4& parentTransform, double timeTicks);

    const AnimationData* m_data = nullptr;
    unsigned int m_animationIndex = 0;
    double m_playbackSeconds = 0.0;
    double m_playbackSpeed = 1.0;
    InterpolationMode m_interpolationMode = InterpolationMode::Incremental;
    AnimationRange m_playbackRange;
    bool m_hasPlaybackRange = false;

    bool m_playing = false;

    std::map<std::string, glm::mat4> m_nodeTransforms;
};
