#include "AnimationData.h"
#include <assimp/scene.h>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <set>

namespace
{
glm::vec3 ImportVector(const aiVector3D& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

Quaternion ImportQuaternion(const aiQuaternion& q)
{
    return Quaternion(q.w, q.x, q.y, q.z);
}

glm::mat4 ImportMatrix(const aiMatrix4x4& m)
{
    // GLM lists columns; Assimp members are rows.
    return glm::mat4(m.a1, m.b1, m.c1, m.d1,
                     m.a2, m.b2, m.c2, m.d2,
                     m.a3, m.b3, m.c3, m.d3,
                     m.a4, m.b4, m.c4, m.d4);
}

float ImportUniformScale(const aiVector3D& scale, const std::string& context)
{
    const bool hasInvalidComponent =
        !std::isfinite(scale.x) || !std::isfinite(scale.y) || !std::isfinite(scale.z);

    // Allow small roundoff differences from imported matrix decomposition.
    constexpr float relativeTolerance = 0.00001f;
    const float largestMagnitude = std::max({
        1.0f, std::abs(scale.x), std::abs(scale.y), std::abs(scale.z) });
    const float tolerance = relativeTolerance * largestMagnitude;

    const bool hasNonuniformScale =
        std::abs(scale.x - scale.y) > tolerance ||
        std::abs(scale.x - scale.z) > tolerance;

    if (hasInvalidComponent || hasNonuniformScale)
    {
        throw std::runtime_error("Unsupported nonuniform or invalid scale in " + context);
    }
    return scale.x;
}

SkeletonNode ImportNode(const aiNode& source, const std::set<std::string>& animatedNodes)
{
    SkeletonNode node;

    node.name = source.mName.C_Str();
    node.bindMatrix = ImportMatrix(source.mTransformation);

    aiVector3D position, scale;
    aiQuaternion rotation;
    source.mTransformation.Decompose(scale, rotation, position);

    node.bindVQS = VQS(ImportVector(position), ImportQuaternion(rotation),
        animatedNodes.count(node.name)
            ? ImportUniformScale(scale, "bind transform of node '" + node.name + "'")
            : 1.0f);
    node.children.reserve(source.mNumChildren);

    for (unsigned int i = 0; i < source.mNumChildren; ++i)
    {
        node.children.push_back(ImportNode(*source.mChildren[i], animatedNodes));
    }

    return node;
}

void ImportMeshInstances(const aiNode& node, unsigned int meshCount,
    std::vector<MeshInstance>& instances, std::map<std::string, unsigned int>& names)
{
    ++names[node.mName.C_Str()];
    for (unsigned int i = 0; i < node.mNumMeshes; ++i)
    {
        if (node.mMeshes[i] >= meshCount)
        {
            throw std::runtime_error("Invalid mesh index in node hierarchy");
        }
        instances.push_back({node.mMeshes[i], node.mName.C_Str()});
    }
    for (unsigned int i = 0; i < node.mNumChildren; ++i)
    {
        ImportMeshInstances(*node.mChildren[i], meshCount, instances, names);
    }
}
}

AnimationData ImportAnimationData(const aiScene* scene)
{
    AnimationData data;
    if (!scene)
    {
        return data;
    }

    if (scene->mRootNode)
    {
        data.hasRoot = true;
        std::set<std::string> animatedNodes;
        for (unsigned int i = 0; i < scene->mNumAnimations; ++i)
        {
            for (unsigned int j = 0; j < scene->mAnimations[i]->mNumChannels; ++j)
            {
                if (scene->mAnimations[i]->mChannels[j])
                {
                    animatedNodes.insert(scene->mAnimations[i]->mChannels[j]->mNodeName.C_Str());
                }
            }
        }
        // Static nodes use their exact bind matrix, including nonuniform scale.
        data.root = ImportNode(*scene->mRootNode, animatedNodes);
        // Match the existing Animator's inversion at the import boundary.
        data.inverseRootMatrix =
            ImportMatrix(aiMatrix4x4(scene->mRootNode->mTransformation).Inverse());
    }

    data.clips.reserve(scene->mNumAnimations);
    for (unsigned int i = 0; i < scene->mNumAnimations; ++i)
    {
        const aiAnimation& source = *scene->mAnimations[i];
        AnimationClip clip;

        clip.name = source.mName.C_Str();
        clip.durationTicks = source.mDuration;
        clip.ticksPerSecond = source.mTicksPerSecond;
        clip.channels.reserve(source.mNumChannels);

        for (unsigned int j = 0; j < source.mNumChannels; ++j)
        {
            const aiNodeAnim* imported = source.mChannels[j];
            if (!imported)
            {
                continue;
            }
            AnimationChannel channel;
            channel.nodeName = imported->mNodeName.C_Str();
            channel.positions.reserve(imported->mNumPositionKeys);
            for (unsigned int k = 0; k < imported->mNumPositionKeys; ++k)
            {
                const auto& key = imported->mPositionKeys[k];
                channel.positions.push_back({key.mTime, ImportVector(key.mValue)});
            }
            channel.rotations.reserve(imported->mNumRotationKeys);
            for (unsigned int k = 0; k < imported->mNumRotationKeys; ++k)
            {
                const auto& key = imported->mRotationKeys[k];
                channel.rotations.push_back({key.mTime, ImportQuaternion(key.mValue)});
            }
            channel.scales.reserve(imported->mNumScalingKeys);
            for (unsigned int k = 0; k < imported->mNumScalingKeys; ++k)
            {
                const auto& key = imported->mScalingKeys[k];
                channel.scales.push_back({key.mTime, ImportUniformScale(key.mValue,
                    "scale key at tick " + std::to_string(key.mTime) + " of node '" + channel.nodeName + "'")});
            }
            clip.channels.push_back(std::move(channel));
        }
        data.clips.push_back(std::move(clip));
    }
    data.meshBones.resize(scene->mNumMeshes);
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
    {
        const aiMesh& mesh = *scene->mMeshes[i];
        auto& bones = data.meshBones[i];
        bones.reserve(mesh.mNumBones);
        for (unsigned int j = 0; j < mesh.mNumBones; ++j)
        {
            const aiBone& bone = *mesh.mBones[j];
            bones.push_back({bone.mName.C_Str(), ImportMatrix(bone.mOffsetMatrix)});
        }
    }
    std::map<std::string, unsigned int> nodeNames;
    if (scene->mRootNode)
    {
        ImportMeshInstances(*scene->mRootNode, scene->mNumMeshes, data.meshInstances, nodeNames);
    }
    auto requireUniqueNode = [&nodeNames](const std::string& name)
    {
        if (nodeNames[name] != 1)
        {
            throw std::runtime_error("Missing or ambiguous animation/mesh node: " + name);
        }
    };
    for (const auto& instance : data.meshInstances)
    {
        requireUniqueNode(instance.nodeName);
    }
    for (const auto& bones : data.meshBones)
    {
        for (const auto &bone : bones)
        {
            requireUniqueNode(bone.name);
        }
    }
    for (const auto& clip : data.clips)
    {
        for (const auto &channel : clip.channels)
        {
            if (nodeNames[channel.nodeName] > 1)
            {
                throw std::runtime_error("Ambiguous animation node: " + channel.nodeName);
            }
        }
    }
    return data;
}
