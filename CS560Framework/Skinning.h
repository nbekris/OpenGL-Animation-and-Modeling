#pragma once
#include "AnimationData.h"
#include <map>
#include <cmath>
#include <stdexcept>

constexpr int MAX_NUM_BONES_PER_VERTEX = 4;
constexpr unsigned int MAX_SKIN_BONES = 128;

struct BoneData
{
    int ids[MAX_NUM_BONES_PER_VERTEX] = {};
    float w[MAX_NUM_BONES_PER_VERTEX] = {};

    void AddBoneData(int id, float weight)
    {
        if (id < 0 || id >= static_cast<int>(MAX_SKIN_BONES) ||
            !std::isfinite(weight) || weight < 0.0f)
        {
            throw std::runtime_error("Invalid imported bone influence");
        }
        if (weight == 0.0f)
        {
            return;
        }
        for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; ++i)
        {
            if (w[i] == 0.0f)
            {
                ids[i] = id;
                w[i] = weight;
                return;
            }
        }
        throw std::runtime_error("Assimp bone influence limit exceeded");
    }

    void Normalize()
    {
        double sum = 0.0;
        for (float weight : w)
        {
            sum += weight;
        }
        if (sum > 0.0)
        {
            for (float &weight : w)
            {
                weight = static_cast<float>(weight / sum);
            }
        }
    }
};

inline void BuildSkinPalette(const std::vector<SkeletonBone>& bindings,
    const std::map<std::string, glm::mat4>& pose, std::vector<glm::mat4>& palette)
{
    if (bindings.size() > MAX_SKIN_BONES)
    {
        throw std::runtime_error("Submesh exceeds the 128-bone skinning palette");
    }
    palette.resize(bindings.size());
    for (size_t i = 0; i < bindings.size(); ++i)
    {
        const auto node = pose.find(bindings[i].name);
        if (node == pose.end())
        {
            throw std::runtime_error("Missing skinning node: " + bindings[i].name);
        }
        // Animator already applies the inverse scene root.
        // rightmost matrix acts first
        palette[i] = node->second * bindings[i].offsetMatrix;
    }
}

inline glm::mat4 SkinInstanceTransform(const glm::mat4& referenceNode,
    const glm::mat4& instanceNode)
{
    if (std::abs(glm::determinant(referenceNode)) < 1e-12f)
    {
        throw std::runtime_error("Cannot instance a skinned mesh with a singular reference node");
    }
    return instanceNode * glm::inverse(referenceNode);
}
