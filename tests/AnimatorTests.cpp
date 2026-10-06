#include "Animator.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void Check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
void Near(double actual, double expected, const char* message)
{
    Check(std::abs(actual - expected) < 1e-5, message);
}

int main()
{
    try
    {
        aiScene scene;
        scene.mRootNode = new aiNode("root");
        scene.mRootNode->mTransformation.a4 = 10.0f;
        scene.mRootNode->mNumChildren = 1;
        scene.mRootNode->mChildren = new aiNode*[1]{new aiNode("joint")};
        aiNode* joint = scene.mRootNode->mChildren[0];
        joint->mParent = scene.mRootNode;
        joint->mTransformation.b4 = 3.0f;
        scene.mNumAnimations = 3;
        scene.mAnimations = new aiAnimation*[3]{new aiAnimation(), new aiAnimation(), new aiAnimation()};
        scene.mAnimations[0]->mDuration = 100.0;
        scene.mAnimations[0]->mTicksPerSecond = 50.0;
        scene.mAnimations[1]->mDuration = 50.0;
        scene.mAnimations[1]->mTicksPerSecond = 0.0;
        scene.mAnimations[2]->mDuration = 0.0;
        aiAnimation* clip = scene.mAnimations[0];
        clip->mNumChannels = 1;
        clip->mChannels = new aiNodeAnim*[1]{new aiNodeAnim()};
        aiNodeAnim* channel = clip->mChannels[0];
        channel->mNodeName = aiString("joint");
        channel->mNumPositionKeys = 2;
        channel->mPositionKeys = new aiVectorKey[2]{
            aiVectorKey(0.0, aiVector3D(0, 3, 0)), aiVectorKey(100.0, aiVector3D(4, 3, 0))};

        Animator animator;
        animator.Update(1.0);
        Near(animator.GetSampleTimeTicks(), 0, "empty animator");
        animator.Reset(&scene);
        animator.Update(1.0);
        Near(animator.GetPlaybackSeconds(), 0, "initially paused");
        animator.Play();
        animator.Update(0.5);
        Near(animator.GetSampleTimeTicks(), 25, "seconds to ticks");
        animator.Pause();
        animator.Update(20.0);
        Near(animator.GetPlaybackSeconds(), 0.5, "paused clock");
        animator.Play();
        animator.Update(0.25);
        Near(animator.GetPlaybackSeconds(), 0.75, "resume");
        animator.Update(5.5);
        Near(animator.GetPlaybackSeconds(), 0.25, "multi-loop update");
        animator.Update(-1.0);
        animator.Update(std::numeric_limits<double>::quiet_NaN());
        Near(animator.GetPlaybackSeconds(), 0.25, "invalid frame durations");
        Check(!animator.SetAnimation(99), "reject invalid clip");
        Near(animator.GetPlaybackSeconds(), 0.25, "invalid selection preserves clock");
        animator.EvaluatePose(50.0);
        const auto& pose = animator.GetNodeTransforms();
        Near(pose.at("joint").a4, 2, "position interpolation and inverse root");
        Near(pose.at("joint").b4, 3, "missing scale/rotation preserve defaults");
        Near(pose.at("root").a4, 0, "root coordinate system");
        Check(animator.SetAnimation(1), "select clip");
        Near(animator.GetPlaybackSeconds(), 0, "clip selection resets clock");
        Check(animator.IsPlaying(), "selection preserves playing state");
        animator.Update(0.5);
        Near(animator.GetSampleTimeTicks(), 12.5, "unspecified tick rate fallback");
        animator.Update(1.5);
        Near(animator.GetSampleTimeTicks(), 0, "exact duration wraps");
        animator.SetAnimation(2);
        animator.Update(1.0);
        Near(animator.GetSampleTimeTicks(), 0, "zero duration");
        animator.Reset();
        Check(animator.GetNodeTransforms().empty() && !animator.IsPlaying(), "reset detaches scene");
        Near(animator.GetPlaybackSeconds(), 0, "reset clock");
        std::cout << "Animator clock and pose tests passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
