#define GLM_FORCE_RADIANS
#include "Animator.h"
#include "InverseKinematics.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>

// Stops the regression run when an IK invariant fails.
void Check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

// Builds a five-link skeleton with an imported scene root and a helper node.
AnimationData MakeSkeleton()
{
    AnimationData data;
    data.hasRoot = true;
    data.root.name = "Scene";
    data.root.bindMatrix = glm::translate(glm::mat4(1), glm::vec3(4, 3, 2));
    data.inverseRootMatrix = glm::inverse(data.root.bindMatrix);
    data.root.children.push_back({});
    auto* node = &data.root.children.back();
    data.meshBones.resize(1);
    for (int i = 0; i < 6; ++i)
    {
        node->name = i == 0 ? "Pelvis" : i == 5 ? "RightHand" : "joint" + std::to_string(i);
        if (i) node->bindMatrix = glm::translate(glm::mat4(1), glm::vec3(1, 0, 0));
        data.meshBones[0].push_back({node->name, glm::mat4(1)});
        if (i == 5) break;
        node->children.push_back({});
        node = &node->children.back();
        if (i == 2)
        {
            node->name = "helper";
            node->children.push_back({});
            node = &node->children.back();
        }
    }
    return data;
}

// Checks fixed root position and all world-space link lengths.
void CheckGeometry(const std::vector<glm::vec3>& before, const std::vector<glm::vec3>& after,
    const glm::mat4& world)
{
    Check(before.size() >= 2 && before.size() == after.size(), "highlighted chain segments");
    Check(glm::length(before[0] - after[0]) < 1e-4f, "root position changed");
    for (int i = 0; i < static_cast<int>(before.size()); i += 2)
    {
        const float a = glm::length(glm::vec3(world * glm::vec4(before[i+1] - before[i], 0)));
        const float b = glm::length(glm::vec3(world * glm::vec4(after[i+1] - after[i], 0)));
        Check(std::isfinite(b) && std::abs(a-b) < 0.002f, "link length changed");
    }
}

// Checks local hinge bounds, stagnation, weights, blending, and priority independently.
void TestConstrainedControls()
{
    auto data = MakeSkeleton();
    Animator animator;
    animator.Reset(&data);
    const auto full = InverseKinematics::FindRightHandChain(animator);
    IKChain one{{full.joints[0], full.joints[1]}, "single hinge"};
    CCDSettings settings;
    settings.maxStepRadians = 2.0f;
    auto& joint = settings.joints[0];
    joint.hingeAxis = 2;
    joint.minDegrees = glm::vec3(0);
    joint.maxDegrees = glm::vec3(0, 0, 30);
    animator.EvaluatePose(0);
    auto result = InverseKinematics::SolveCCD(animator, one, glm::vec3(0,1,0), glm::mat4(1), settings);
    auto end = InverseKinematics::GetEndpoints(animator, one).back();
    Check(std::abs(end.y - 0.5f) < 1e-4f, "hinge upper limit");
    Check(result.stagnated && !result.reached && result.iterations < settings.maxIterations,
        "blocked target did not stop on stagnation");
    animator.EvaluatePose(0);
    result = InverseKinematics::SolveCCD(animator, one, glm::vec3(0,-1,0), glm::mat4(1), settings);
    Check(result.stagnated && std::abs(InverseKinematics::GetEndpoints(animator, one).back().y) < 1e-5f,
        "one-sided hinge bent backward");
    animator.EvaluatePose(0);
    InverseKinematics::SolveCCD(animator, one, glm::vec3(0,1,0), glm::mat4(1), settings, 0.5f);
    Check(std::abs(InverseKinematics::GetEndpoints(animator, one).back().y - std::sin(glm::radians(15.0f))) < 1e-4f,
        "blended hinge violated local bounds");
    settings.maxIterations = 1;
    settings.maxStepRadians = 0.2f;
    joint.weight = 0.5f;
    animator.EvaluatePose(0);
    InverseKinematics::SolveCCD(animator, one, glm::vec3(0,1,0), glm::mat4(1), settings);
    Check(std::abs(InverseKinematics::GetEndpoints(animator, one).back().y - std::sin(0.1f)) < 1e-4f,
        "joint weight did not scale its angle");
    joint.weight = 0;
    animator.EvaluatePose(0);
    result = InverseKinematics::SolveCCD(animator, one, glm::vec3(0,1,0), glm::mat4(1), settings);
    Check(result.stagnated && glm::length(InverseKinematics::GetEndpoints(animator, one).back() - glm::vec3(1,0,0)) < 1e-5f,
        "zero joint weight did not freeze the joint");

    IKChain two{{full.joints[0], full.joints[1], full.joints[2]}, "two joints"};
    settings = CCDSettings{};
    settings.maxIterations = 1;
    settings.maxStepRadians = 3.14f;
    settings.joints[0].priority = 0;
    settings.joints[1].priority = 1;
    animator.EvaluatePose(0);
    InverseKinematics::SolveCCD(animator, two, glm::vec3(1,1,0), glm::mat4(1), settings);
    const float rootTurn = animator.GetNodeTransforms().at("Pelvis")[0].y;
    Check(rootTurn > 0.5f, "root-first priority was ignored");
    settings.joints[1].priority = settings.joints[0].priority;
    animator.EvaluatePose(0);
    InverseKinematics::SolveCCD(animator, two, glm::vec3(1,1,0), glm::mat4(1), settings);
    Check(std::abs(animator.GetNodeTransforms().at("Pelvis")[0].y) < 1e-5f,
        "standard tip-to-root order was not restored");

    // A rotated reference pose must rotate the hinge axis with the joint.
    data.root.children[0].bindMatrix = glm::rotate(glm::mat4(1), glm::radians(90.0f), glm::vec3(1,0,0));
    animator.Reset(&data);
    animator.EvaluatePose(0);
    settings = CCDSettings{};
    settings.joints[0].hingeAxis = 2;
    settings.joints[0].minDegrees = glm::vec3(0);
    settings.joints[0].maxDegrees = glm::vec3(0,0,90);
    result = InverseKinematics::SolveCCD(animator, one, glm::vec3(0,0,1), glm::mat4(1), settings);
    Check(result.reached, "hinge axis was not evaluated in local space");
    settings.joints[0].minDegrees.z = 10;
    Check(!InverseKinematics::SolveCCD(animator, one, glm::vec3(0), glm::mat4(1), settings).valid,
        "invalid local interval accepted");
}

// Exercises convergence, limits, degeneracies, and both production model hierarchies.
int main(int argc, char** argv)
{
    try
    {
        TestConstrainedControls();
        auto data = MakeSkeleton();
        Animator animator;
        animator.Reset(&data);
        const auto chain = InverseKinematics::FindRightHandChain(animator);
        Check(chain.joints.size() == 6, "right-hand chain must include all five links");
        CCDSettings settings;
        settings.maxIterations = 128;
        settings.toleranceWorld = 0.01f;
        settings.joints.fill(CCDJointSettings{});
        const glm::mat4 world = glm::translate(glm::mat4(1), glm::vec3(10, -4, 3)) *
            glm::rotate(glm::mat4(1), 0.6f, glm::vec3(0,0,1)) * glm::scale(glm::mat4(1), glm::vec3(0.7f));
        for (const glm::vec3 target : {glm::vec3(3,2,0), glm::vec3(-3,0,0), glm::vec3(20,1,0), glm::vec3(0)})
        {
            animator.EvaluatePose(0);
            const auto before = InverseKinematics::GetEndpoints(animator, chain);
            const auto result = InverseKinematics::SolveCCD(animator, chain, glm::vec3(world * glm::vec4(target,1)), world, settings);
            Check(result.valid && std::isfinite(result.errorWorld), "invalid CCD result");
            CheckGeometry(before, InverseKinematics::GetEndpoints(animator, chain), world);
            if (target.y == 2 || target.x == -3) Check(result.reached, "reachable target did not converge");
            if (target.x == 20) Check(!result.reached, "unreachable target reported success");
        }
        animator.EvaluatePose(0);
        const auto before = InverseKinematics::GetEndpoints(animator, chain);
        for (auto& joint : settings.joints) joint.minDegrees = joint.maxDegrees = glm::vec3(0);
        InverseKinematics::SolveCCD(animator, chain, glm::vec3(0,3,0), glm::mat4(1), settings);
        Check(glm::length(InverseKinematics::GetEndpoints(animator, chain).back() - before.back()) < 1e-4f, "zero limits ignored");
        settings.joints.fill(CCDJointSettings{});
        InverseKinematics::SolveCCD(animator, chain, glm::vec3(0,3,0), glm::mat4(1), settings, 0);
        Check(glm::length(InverseKinematics::GetEndpoints(animator, chain).back() - before.back()) < 1e-4f, "zero IK blend changed pose");
        for (auto& joint : settings.joints) { joint.minDegrees = glm::vec3(-15); joint.maxDegrees = glm::vec3(15); }
        InverseKinematics::SolveCCD(animator, chain, glm::vec3(-3,2,0), glm::mat4(1), settings);
        const auto& limited = animator.GetNodeTransforms();
        for (int i = 0; i < 5; ++i)
        {
            const std::string name = i == 0 ? "Pelvis" : i == 5 ? "RightHand" : "joint" + std::to_string(i);
            const std::string parent = i == 1 ? "Pelvis" : "joint" + std::to_string(i-1);
            const glm::mat4 local = i == 0 ? limited.at(name) : glm::inverse(limited.at(parent)) * limited.at(name);
            const glm::quat rotation = glm::normalize(glm::quat_cast(glm::mat3(local)));
            const glm::vec3 angle = glm::degrees(glm::eulerAngles(rotation));
            for (int axis = 0; axis < 3; ++axis) Check(std::abs(angle[axis]) <= 15.01f, "local Euler limit exceeded");
        }
        settings.joints.fill(CCDJointSettings{});
        auto invalid = chain;
        invalid.joints[2] = invalid.joints[0];
        Check(!InverseKinematics::SolveCCD(animator, invalid, glm::vec3(0), world, settings).valid, "invalid chain accepted");
        Check(!InverseKinematics::SolveCCD(animator, chain, glm::vec3(std::numeric_limits<float>::quiet_NaN()), world, settings).valid,
            "nonfinite target accepted");

        for (int asset = 1; asset < argc; ++asset)
        {
            Assimp::Importer importer;
            const aiScene* scene = importer.ReadFile(argv[asset], 0);
            Check(scene != nullptr, "asset load failed");
            auto imported = ImportAnimationData(scene);
            Animator model;
            model.Reset(&imported);
            const auto candidate = InverseKinematics::FindRightHandChain(model);
            Check(candidate.joints.size() == 8, "production right-hand chain must include all seven links");
            std::cout << argv[asset] << ": " << candidate.label << '\n';
            model.EvaluatePose(0);
            const auto original = InverseKinematics::GetEndpoints(model, candidate);
            const auto defaults = InverseKinematics::CreateSettings(model, candidate);
            const int elbow = static_cast<int>(candidate.joints.size()) - 2;
            Check(defaults.joints[elbow].hingeAxis >= 0, "elbow hinge preset missing");
            Check(defaults.joints[elbow].weight > defaults.joints[0].weight &&
                defaults.joints[elbow].priority < defaults.joints[0].priority, "arm preference missing");
            const auto constrained = InverseKinematics::SolveCCD(model, candidate,
                original.back() + glm::vec3(2,2,1), glm::mat4(1), defaults);
            Check(constrained.valid && constrained.errorWorld < 3.0f, "default arm solve made no progress");
            CheckGeometry(original, InverseKinematics::GetEndpoints(model, candidate), glm::mat4(1));
        }
        std::cout << "PASS: CCD convergence, geometry, local limits, hinges, stagnation, weights, priorities, blend, invalid input, and asset chains\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
