#pragma once
#include "AnimationData.h"
#include "InterpolationMode.h"
#include <map>
#include <string>
#include <set>
#include <unordered_map>

struct KeyIndices
{
	unsigned int previous;
	unsigned int next;
};

struct AnimationRange
{
    double startTicks = 0.0;
    double endTicks = 0.0;
};

// The caller tracks the segment identity separately from the recurrence state.
struct RotationInterpolationState
{
    Quaternion start{1, 0, 0, 0}, end{1, 0, 0, 0};
    int n = 0;
    ISlerpState interpolation;
};

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
    const std::vector<glm::vec3>& GetSkeletonEndpoints() const { return m_skeletonEndpoints; }
private:
    double GetTicksPerSecond() const;
    AnimationRange GetPlaybackRange() const;
    const AnimationClip* GetAnimation() const;
    void CacheAnimationChannels();
    const AnimationChannel* FindAnimationChannel(const std::string& nodeName) const;
    glm::mat4 SampleLocalTransform(const SkeletonNode* node, const AnimationChannel* channel, double animationTime);
    void EvaluateNode(const SkeletonNode* node, const glm::mat4& parentTransform, double timeTicks,
        bool hasParentBone, const glm::vec3& parentBonePosition);

    std::unordered_map<const AnimationChannel*, RotationInterpolationState> m_rotationSamplers;

    const AnimationData* m_data = nullptr;
    unsigned int m_animationIndex = 0;
    double m_playbackSeconds = 0.0;
    double m_playbackSpeed = 1.0;
    InterpolationMode m_interpolationMode = InterpolationMode::Incremental;
    AnimationRange m_playbackRange;
    bool m_hasPlaybackRange = false;

    bool m_playing = false;

    std::map<std::string, glm::mat4> m_nodeTransforms;
    std::set<std::string> m_boneNames;
    std::vector<glm::vec3> m_skeletonEndpoints;
    // References the selected clip in the AnimationData supplied to Reset.
    std::unordered_map<std::string, const AnimationChannel*> m_animationChannels;
};
