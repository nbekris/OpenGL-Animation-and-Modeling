#pragma once
#include "AnimationData.h"
#include <map>
#include <string>

struct KeyIndices
{
	unsigned int previous;
	unsigned int next;
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

    unsigned int GetAnimationIndex() const { return m_animationIndex; }
    unsigned int GetAnimationCount() const;

    std::string GetAnimationName(unsigned int index) const;

    // Current position within the clip, wrapped to its duration.
    double GetPlaybackSeconds() const { return m_playbackSeconds; }
    double GetSampleTimeTicks() const;
    double GetFirstKeyTimeTicks() const;
    void EvaluatePose(double timeTicks);

    const std::map<std::string, glm::mat4>& GetNodeTransforms() const { return m_nodeTransforms; }
private:
    const AnimationClip* GetAnimation() const;
    const AnimationChannel* FindAnimationChannel(const AnimationClip* animation, const std::string& nodeName) const;
    glm::mat4 SampleLocalTransform(const SkeletonNode* node, const AnimationChannel* channel, double animationTime) const;
    void EvaluateNode(const SkeletonNode* node, const glm::mat4& parentTransform, double timeTicks);

    const AnimationData* m_data = nullptr;
    unsigned int m_animationIndex = 0;
    double m_playbackSeconds = 0.0;

    bool m_playing = false;

    std::map<std::string, glm::mat4> m_nodeTransforms;
};
