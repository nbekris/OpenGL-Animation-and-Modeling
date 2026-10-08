#include "AnimationData.h"
#include <assimp/scene.h>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <algorithm>

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
    // Allow small roundoff differences from imported matrix decomposition.
    const float tolerance = 1e-5f * std::max(1.0f,
        std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z))));
    if (!std::isfinite(scale.x) || !std::isfinite(scale.y) || !std::isfinite(scale.z) ||
        std::abs(scale.x - scale.y) > tolerance || std::abs(scale.x - scale.z) > tolerance)
    {
        throw std::runtime_error("Unsupported nonuniform or invalid scale in " + context);
    }
    return scale.x;
}
SkeletonNode ImportNode(const aiNode& source)
{
    SkeletonNode node;

    node.name = source.mName.C_Str();
    node.bindMatrix = ImportMatrix(source.mTransformation);

    aiVector3D position, scale;
    aiQuaternion rotation;
    source.mTransformation.Decompose(scale, rotation, position);

    node.bindVQS = VQS(ImportVector(position), ImportQuaternion(rotation),
        ImportUniformScale(scale, "bind transform of node '" + node.name + "'"));
    node.children.reserve(source.mNumChildren);

    for (unsigned int i = 0; i < source.mNumChildren; ++i)
    {
        node.children.push_back(ImportNode(*source.mChildren[i]));
    }

    return node;
}
}

AnimationData ImportAnimationData(const aiScene* scene)
{
    AnimationData data;
    if (!scene) return data;

    if (scene->mRootNode)
    {
        data.hasRoot = true;
        data.root = ImportNode(*scene->mRootNode);
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
            if (!imported) continue;
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
    return data;
}
