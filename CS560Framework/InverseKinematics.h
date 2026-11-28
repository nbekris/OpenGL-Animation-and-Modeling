#pragma once
#include <array>
#include <string>
#include <vector>
#include <glm/glm.hpp>

class Animator;

struct IKChain
{
    std::vector<int> joints;
    std::string label;
};

struct CCDJointSettings
{
    // Local XYZ offsets from the sampled pose; zero stays inside every range.
    glm::vec3 minDegrees{-180.0f};
    glm::vec3 maxDegrees{180.0f};
    float weight = 1.0f;
    int priority = 0; // Lower values are visited first; ties use tip-to-root order.
    int hingeAxis = -1; // -1: XYZ rotation, 0/1/2: local X/Y/Z hinge.
};

struct CCDSettings
{
    int maxIterations = 48;
    float toleranceWorld = 0.05f;
    float maxStepRadians = 0.15f;
    float stagnationWorld = 0.0001f;
    std::array<CCDJointSettings, 16> joints;
};

struct CCDResult
{
    bool valid = false;
    bool reached = false;
    bool stagnated = false;
    int iterations = 0;
    float errorWorld = 0.0f;
};

namespace InverseKinematics
{
    // Finds the complete right-hand branch from the skeletal root.
    IKChain FindRightHandChain(const Animator& animator);
    // Creates local limits and arm-first priorities for the right-hand branch.
    CCDSettings CreateSettings(const Animator& animator, const IKChain& chain);
    // Solves and blends local corrections into the supplied animation pose.
    CCDResult SolveCCD(Animator& animator, const IKChain& chain, const glm::vec3& targetWorld,
        const glm::mat4& modelTransform, const CCDSettings& settings, float weight = 1.0f);
    // Builds highlighted chain segments from the final model-space pose.
    std::vector<glm::vec3> GetEndpoints(const Animator& animator, const IKChain& chain);
}
