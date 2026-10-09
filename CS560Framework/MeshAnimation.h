#pragma once
#include <map>
#include <string>
#include <vector>
#include <assimp/scene.h>
#include "VQS.h"

// CPU skeleton pose evaluation. The borrowed scene must outlive this component.
class MeshAnimation
{
public:
    void Initialize(const aiScene* scene, const std::map<std::string, int>& boneIndices,
        const std::vector<aiMatrix4x4>& offsets, int animationIndex = 0);
    void UpdateAnimation(int anim_num, double elapsedSeconds, bool loop = true);
    unsigned int GetAnimationCount() const;
    std::string GetAnimationName(unsigned int index) const;
    double GetAnimationDurationSeconds(unsigned int index) const;
    const std::vector<glm::vec3>& GetSkeletonEndpoints() const { return m_endpoints; }
private:
    void BuildBindPoseSkeleton();
    void BuildFirstAnimationFrameSkeleton(int anim_num);
    void CollectBindPoseLines(const aiNode* node, const aiMatrix4x4& parentTransform,
        bool hasParentBone, const std::string& parentBoneName,
        std::vector<glm::vec3>& endpoints) const;
    glm::vec3 GetBindPosePosition(const std::string& boneName) const;
    const aiNodeAnim* FindAnimationChannel(const aiAnimation* animation,
        const std::string& nodeName) const;
    VQS SampleLocalTransform(const aiNode* node,
        const aiNodeAnim* channel, double animationTime) const;
    void CollectAnimatedPoseLines(const aiNode* node, const aiAnimation* animation,
        double animationTime, const VQS& parentTransform,
        bool hasParentBone, const glm::vec3& parentBonePosition,
        std::vector<glm::vec3>& endpoints) const;

    const aiScene* m_scene = nullptr;
    std::map<std::string, int> m_name_index;
    std::vector<aiMatrix4x4> m_offsets;
    VQS m_inverseRootVQS;
    std::vector<glm::vec3> m_endpoints;
};
