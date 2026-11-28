#define GLM_FORCE_RADIANS
#include "InverseKinematics.h"
#include "Animator.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace
{
    // Builds the constrained rotation using XYZ Euler offsets or one hinge axis.
    glm::quat JointRotation(const glm::vec3& angles, int hingeAxis)
    {
        if (hingeAxis < 0) return glm::normalize(glm::quat(angles));
        glm::vec3 axis(0.0f);
        axis[hingeAxis] = 1.0f;
        return glm::angleAxis(angles[hingeAxis], axis);
    }
}

IKChain InverseKinematics::FindRightHandChain(const Animator& animator)
{
    const auto& nodes = animator.GetPoseNodes();
    // Search in reverse to preserve the previous preference for the last match.
    for (int endpoint = static_cast<int>(nodes.size()) - 1; endpoint >= 0; --endpoint)
    {
        if (!nodes[endpoint].bone) continue;
        std::string name = nodes[endpoint].source->name;
        for (auto& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (name.find("righthand") == std::string::npos) continue;

        IKChain chain;
        for (int joint = endpoint; joint >= 0; joint = nodes[joint].parentBone)
            chain.joints.push_back(joint);
        if (chain.joints.size() < 2 || chain.joints.size() > 17) continue;

        std::reverse(chain.joints.begin(), chain.joints.end());
        chain.label = "Right hand: ";
        for (std::size_t i = 0; i < chain.joints.size(); ++i)
        {
            if (i) chain.label += " -> ";
            chain.label += nodes[chain.joints[i]].source->name;
        }
        return chain;
    }
    return {};
}

// Produces highlighted chain segments using the final pose, including IK blending.
std::vector<glm::vec3> InverseKinematics::GetEndpoints(const Animator& animator, const IKChain& chain)
{
    const auto& nodes = animator.GetPoseNodes();
    std::vector<glm::vec3> endpoints;
    for (int i = 0; i < static_cast<int>(chain.joints.size()); ++i)
        if (chain.joints[i] < 0 || chain.joints[i] >= static_cast<int>(nodes.size()))
            return endpoints;

    for (int i = 0; i < static_cast<int>(chain.joints.size()) - 1; ++i)
    {
        endpoints.push_back(glm::vec3(animator.GetInverseRootMatrix() * nodes[chain.joints[i]].global[3]));
        endpoints.push_back(glm::vec3(animator.GetInverseRootMatrix() * nodes[chain.joints[i + 1]].global[3]));
    }
    return endpoints;
}

// Assigns conservative torso limits and stronger arm participation in local axes.
CCDSettings InverseKinematics::CreateSettings(const Animator& animator, const IKChain& chain)
{
    CCDSettings settings;
    const auto& nodes = animator.GetPoseNodes();
    std::vector<glm::mat4> bindGlobal(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i)
        bindGlobal[i] = (nodes[i].parent < 0 ? glm::mat4(1) : bindGlobal[nodes[i].parent]) * nodes[i].source->bindMatrix;
    for (std::size_t i = 0; i + 1 < chain.joints.size() && i < settings.joints.size(); ++i)
    {
        const int index = chain.joints[i];
        if (index < 0 || index >= static_cast<int>(nodes.size())) continue;
        std::string name = nodes[index].source->name;
        for (auto& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        auto& joint = settings.joints[i];
        joint.minDegrees = glm::vec3(-45);
        joint.maxDegrees = glm::vec3(45);
        joint.weight = 0.35f;
        joint.priority = 3;
        if (name.find("hips") != std::string::npos || name.find("pelvis") != std::string::npos)
        {
            joint.minDegrees = glm::vec3(-12, -15, -12);
            joint.maxDegrees = -joint.minDegrees;
            joint.weight = 0.15f;
            joint.priority = 5;
        }
        else if (name.find("spine") != std::string::npos)
        {
            joint.minDegrees = glm::vec3(-20, -25, -20);
            joint.maxDegrees = -joint.minDegrees;
            joint.weight = 0.25f;
            joint.priority = 4;
        }
        else if (name.find("forearm") != std::string::npos)
        {
            joint.weight = 1.0f;
            joint.priority = 0;
            // These assets face -Z before their scene rotation. Estimate the elbow
            // hinge from the bind-pose link and that forward direction.
            const int next = chain.joints[i + 1];
            if (next < 0 || next >= static_cast<int>(nodes.size())) continue;
            const glm::mat4 inverseBind = glm::inverse(bindGlobal[index]);
            const glm::vec3 link = glm::vec3(inverseBind * bindGlobal[next][3]);
            const glm::vec3 forward = glm::vec3(inverseBind * glm::vec4(0, 0, -1, 0));
            const glm::vec3 axis = glm::cross(link, forward);
            int dominant = 0;
            for (int a = 1; a < 3; ++a)
                if (std::abs(axis[a]) > std::abs(axis[dominant])) dominant = a;
            joint.hingeAxis = dominant;
            joint.minDegrees = joint.maxDegrees = glm::vec3(0);
            if (axis[dominant] >= 0) joint.maxDegrees[dominant] = 145;
            else joint.minDegrees[dominant] = -145;
        }
        else if (name.find("arm") != std::string::npos || name.find("shoulder") != std::string::npos)
        {
            joint.minDegrees = glm::vec3(-90, -80, -120);
            joint.maxDegrees = glm::vec3(90, 80, 120);
            joint.weight = 0.9f;
            joint.priority = 1;
        }
    }
    return settings;
}

// Runs weighted, ordered CCD with local joint limits and sweep-level stagnation detection.
CCDResult InverseKinematics::SolveCCD(Animator& animator, const IKChain& chain, const glm::vec3& targetWorld,
    const glm::mat4& modelTransform, const CCDSettings& settings, float weight)
{
    const auto& nodes = animator.GetPoseNodes();
    CCDResult result;

    // Calculate how many joints will revieve corrections
    const int count = static_cast<int>(chain.joints.size()) - 1;

    // Validate Inputs
    if (nodes.empty() || count < 1 || count > static_cast<int>(settings.joints.size()) ||
        !std::isfinite(weight) || settings.maxIterations < 1 ||
        !std::isfinite(settings.toleranceWorld) || settings.toleranceWorld <= 0 ||
        !std::isfinite(settings.stagnationWorld) || settings.stagnationWorld < 0 ||
        !std::isfinite(settings.maxStepRadians) || settings.maxStepRadians <= 0)
    {
        return result;
    }

    // check target coords
    for (int a = 0; a < 3; ++a)
    {
        if (!std::isfinite(targetWorld[a])) return result;
    }

    // Validate chain and joint settings
    for (int i = 0; i <= count; ++i)
    {
        const int index = chain.joints[i];
        if (index < 0 || index >= static_cast<int>(nodes.size()) || !nodes[index].bone ||
            (i && nodes[index].parentBone != chain.joints[i - 1])) return result;
        if (i == count) continue;
        const auto& joint = settings.joints[i];
        if (!std::isfinite(joint.weight) || joint.weight < 0 || joint.weight > 1 ||
            joint.hingeAxis < -1 || joint.hingeAxis > 2) return result;
        // Angle Limits
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite(joint.minDegrees[a]) || !std::isfinite(joint.maxDegrees[a]) ||
                joint.minDegrees[a] < -180 || joint.maxDegrees[a] > 180 ||
                joint.minDegrees[a] > 0 || joint.maxDegrees[a] < 0) return result;
    }

    // Build transform in world space to convert node skeleton position
    const glm::mat4 worldBasis = modelTransform * animator.GetInverseRootMatrix();
    if (!std::isfinite(glm::determinant(worldBasis)) || std::abs(glm::determinant(worldBasis)) < 1e-12f)
        return result;

    // Store starting pose and correction state
    std::vector<glm::mat4> base(count);
    std::vector<glm::vec3> angles(count, glm::vec3(0));

    // List of joints in order to be adjusted
    std::vector<int> order;
    for (int i = count - 1; i >= 0; --i)
    {
        base[i] = nodes[chain.joints[i]].local;
        order.push_back(i);
    }

    // Equal priorities retain tip-to-root order.
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return settings.joints[a].priority < settings.joints[b].priority;
    });

    // Measures the endpoint in world space, also used for the notes Pv comparison.
    const auto endpoint = [&]() {
        return glm::vec3(worldBasis * nodes[chain.joints.back()].global[3]);
    };

    // Applies offsets in the sampled joint's local axes without changing its translation.
    const auto apply = [&](int i, const glm::vec3& offset) {
        animator.SetLocalTransform(chain.joints[i], base[i] *
            glm::mat4_cast(JointRotation(offset, settings.joints[i].hingeAxis)));
    };

    // Solving Sweeps, continues as long as enpoint is further from target
    for (int iteration = 0; iteration < settings.maxIterations &&
        glm::length(targetWorld - endpoint()) > settings.toleranceWorld; ++iteration)
    {
        const glm::vec3 previousEndpoint = endpoint();
        result.iterations = iteration + 1;

        // Visit each rotating joint
        for (int i : order)
        {
            if (glm::length(targetWorld - endpoint()) <= settings.toleranceWorld) break;

            // Gets settings and skips joints configured to make no contribution
            const auto& joint = settings.joints[i];
            if (joint.weight == 0) continue;

            // Get current joints pose node
            const auto& node = nodes[chain.joints[i]];

            // Construct the joints reference frame
            const glm::mat4 referenceWorld = worldBasis *
                (node.parent < 0 ? glm::mat4(1) : nodes[node.parent].global) * base[i];

            // Skip joint if transform is nearly signular
            if (std::abs(glm::determinant(referenceWorld)) < 1e-12f) continue;

            // Build transform from world space into joint reference frame
            const glm::mat4 inverseReference = glm::inverse(referenceWorld);

            // Express endpoint and target relative to the joint
            glm::vec3 from = glm::vec3(inverseReference * glm::vec4(endpoint(), 1));
            glm::vec3 to = glm::vec3(inverseReference * glm::vec4(targetWorld, 1));
            glm::vec3 candidate = angles[i];

            // Handle hinge joint
            if (joint.hingeAxis >= 0)
            {
                glm::vec3 axis(0);
                axis[joint.hingeAxis] = 1;

                // Remove components parallel to the hinge axis
                from -= axis * glm::dot(from, axis);
                to -= axis * glm::dot(to, axis);

                // Skip if projected direction is too small to define angle
                if (glm::length(from) < 1e-6f || glm::length(to) < 1e-6f) continue;

                from = glm::normalize(from);
                to = glm::normalize(to);

                // Compute signed angle from the endpoint direction to target direction around hinge axis
                const float angle = std::atan2(glm::dot(axis, glm::cross(from, to)), glm::dot(from, to));
                candidate[joint.hingeAxis] += glm::clamp(angle, -settings.maxStepRadians,
                    settings.maxStepRadians) * joint.weight;
            }
            else
            {
                // Handle joint that can rotate around XYZ
                if (glm::length(from) < 1e-6f || glm::length(to) < 1e-6f) continue;

                from = glm::normalize(from);
                to = glm::normalize(to);

                const float cosine = glm::clamp(glm::dot(from, to), -1.0f, 1.0f);

                if (cosine > 0.999999f) continue;

                glm::vec3 axis = glm::cross(from, to);

                // Case for when directions are nearly opposite
                if (glm::length(axis) < 1e-6f)
                {
                    axis = glm::cross(from, std::abs(from.x) < 0.8f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0));
                }

                // Compute the required angle, caps the step size, and scales by joint weights
                const float angle = (std::min)(std::acos(cosine), settings.maxStepRadians) * joint.weight;

                // Build updated correction
                candidate = glm::eulerAngles(glm::normalize(glm::angleAxis(angle, glm::normalize(axis)) *
                    JointRotation(angles[i], -1)));
            }

            // Enforce joint limits and update pose
            angles[i] = glm::clamp(candidate, glm::radians(joint.minDegrees), glm::radians(joint.maxDegrees));
            apply(i, angles[i]);
            animator.RebuildGlobalPose();
        }

        // Check whether sweep should end solving
        if (glm::length(targetWorld - endpoint()) <= settings.toleranceWorld) break;
        if (glm::length(endpoint() - previousEndpoint) <= settings.stagnationWorld)

        {
            result.stagnated = true;
            break;
        }
    }

    // Blend solved correction with original animation
    weight = glm::clamp(weight, 0.0f, 1.0f);

    // Blend bounded local angles; zero is inside every range, so limits remain satisfied.
    for (int i = 0; i < count; ++i) apply(i, angles[i] * weight);

    // Update global pose
    animator.RebuildGlobalPose();

    // Report final result
    result.valid = true;
    result.errorWorld = glm::length(targetWorld - endpoint());
    result.reached = result.errorWorld <= settings.toleranceWorld;
    return result;
}

