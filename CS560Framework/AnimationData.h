#pragma once
#include "VQS.h"
#include <string>
#include <vector>
struct aiScene;

template <typename Value>
struct AnimationKey { double timeTicks; Value value; };

struct AnimationChannel
{
    std::string nodeName;
    std::vector<AnimationKey<glm::vec3>> positions;
    std::vector<AnimationKey<Quaternion>> rotations;
    std::vector<AnimationKey<float>> scales;
};

struct AnimationClip
{
    std::string name;
    double durationTicks = 0.0;
    // Preserve zero; Animator applies its existing tick-rate fallback.
    double ticksPerSecond = 0.0;
    std::vector<AnimationChannel> channels;
};

struct SkeletonNode
{
    std::string name;
    std::vector<SkeletonNode> children;
    // Exact transform for nodes without animation channels.
    glm::mat4 bindMatrix{1.0f};
    // Decomposed defaults for animated nodes; static nodes use bindMatrix.
    VQS bindVQS;
};

struct SkeletonBone
{
    std::string name;
    glm::mat4 offsetMatrix{1.0f};
};

struct MeshInstance
{
    unsigned int meshIndex;
    std::string nodeName;
};

// Owns all values, with no pointers into the Assimp importer.
struct AnimationData
{
    bool hasRoot = false;
    SkeletonNode root;
    glm::mat4 inverseRootMatrix{1.0f};
    std::vector<AnimationClip> clips;
    // Mesh order, then bone order; preserve mesh-specific offsets for shared names.
    std::vector<std::vector<SkeletonBone>> meshBones;
    std::vector<MeshInstance> meshInstances;
};

// Null produces empty data. Animated nonuniform scale is unsupported;
// static nodes retain their exact bind matrix.
AnimationData ImportAnimationData(const aiScene* scene);
