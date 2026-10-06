#pragma once
#include <assimp/scene.h>
#include <map>
#include <string>

// Borrows the scene; reset before its owning importer reloads.
// No rendering dependencies. Advancing time and evaluating a pose are separate.
class Animator
{
public:
    void Reset(const aiScene* scene = nullptr);
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
    const std::map<std::string, aiMatrix4x4>& GetNodeTransforms() const { return m_nodeTransforms; }
private:
    const aiAnimation* GetAnimation() const;
    const aiNodeAnim* FindAnimationChannel(const aiAnimation* animation, const std::string& nodeName) const;
    aiMatrix4x4 SampleLocalTransform(const aiNode* node, const aiNodeAnim* channel, double animationTime) const;
    void EvaluateNode(const aiNode* node, const aiMatrix4x4& parentTransform, double timeTicks);
    const aiScene* m_scene = nullptr;
    unsigned int m_animationIndex = 0;
    double m_playbackSeconds = 0.0;
    bool m_playing = false;
    aiMatrix4x4 m_inverseRoot;
    std::map<std::string, aiMatrix4x4> m_nodeTransforms;
};
