#include "Animator.h"
#include "AnimationPresets.h"
#include <assimp/scene.h>
#include "AnimationData.h"
#include "VQS.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/config.h>
#include <fstream>
#include <iomanip>
#include "FrameLimiter.h"
#include "TimingSettings.h"

void Check(bool value, const char* message)
{
    if (!value)
    {
        throw std::runtime_error(message);
    }

}
void Near(double actual, double expected, const char* message)
{
    Check(std::abs(actual - expected) < 1e-5, message);
}

// A synthetic channel with deliberately different key counts and time intervals.
void TestVQSSampling()
{
    aiScene scene;
    scene.mRootNode = new aiNode("root");
    scene.mRootNode->mTransformation.a4 = 10.0f;
    scene.mRootNode->mNumChildren = 1;
    scene.mRootNode->mChildren = new aiNode*[1]{new aiNode("joint")};
    aiNode* joint = scene.mRootNode->mChildren[0];
    joint->mParent = scene.mRootNode;
    scene.mNumAnimations = 1;
    scene.mAnimations = new aiAnimation*[1]{new aiAnimation()};
    aiAnimation* clip = scene.mAnimations[0];
    clip->mDuration = 60;
    clip->mTicksPerSecond = 144; // Exact landmark samples on the 144 Hz grid.
    clip->mNumChannels = 1;
    clip->mChannels = new aiNodeAnim*[1]{new aiNodeAnim()};
    aiNodeAnim* channel = clip->mChannels[0];
    channel->mNodeName = aiString("joint");
    channel->mNumPositionKeys = 4;
    channel->mPositionKeys = new aiVectorKey[4]{
        aiVectorKey(0, aiVector3D(-2,-2,1)), aiVectorKey(10, aiVector3D(0,0,2)),
        aiVectorKey(20, aiVector3D(4,0,4)), aiVectorKey(40, aiVector3D(6,-2,5))};
    const float half = std::sqrt(0.5f);
    channel->mNumRotationKeys = 2;
    channel->mRotationKeys = new aiQuatKey[2]{
        aiQuatKey(0, aiQuaternion(2,0,0,0)), aiQuatKey(60, aiQuaternion(-half,0,0,-half))};
    channel->mNumScalingKeys = 2;
    channel->mScalingKeys = new aiVectorKey[2]{
        aiVectorKey(5, aiVector3D(1,1,1)), aiVectorKey(45, aiVector3D(5,5,5))};
    Animator animator;
    AnimationData animationData = ImportAnimationData(&scene);
    animator.Reset(&animationData);
    auto sample = [&](double time, InterpolationMode mode = InterpolationMode::Incremental) {
        // Re-import after each fixture mutation, just as a file reload would.
        animationData = ImportAnimationData(&scene);
        animator.Reset(&animationData);
        animator.SetInterpolationMode(mode);
        animator.EvaluatePose(time);
        return animator.GetNodeTransforms().at("joint");
    };
    glm::mat4 m = sample(15);
    Near(m[3][0],2,"Bezier midpoint x"); Near(m[3][1],0,"linear midpoint y");
    Near(m[3][2],3,"Bezier midpoint z");
    const float angle = 3.14159265358979323846f / 8.0f;
    Near(m[0][0],std::pow(5.0f,0.25f)*std::cos(angle),"independent rotation and scale timing");
    Near(m[0][1],std::pow(5.0f,0.25f)*std::sin(angle),"rotation matrix column mapping");
    Near(m[1][0],-std::pow(5.0f,0.25f)*std::sin(angle),"rotation matrix row mapping");
    Near(m[2][2],std::pow(5.0f,0.25f),"uniform scale"); Near(m[3][3],1,"homogeneous matrix");
    Near(channel->mRotationKeys[0].mValue.w,2,"source rotation not normalized in place");
    Near(channel->mRotationKeys[1].mValue.w,-half,"source rotation not negated in place");
    Check(animator.GetInterpolationMode() == InterpolationMode::Incremental, "default incremental mode");
    animator.Play();
    animator.SetPlaybackSpeed(2.0);
    Check(animator.SetPlaybackRange({5, 45}), "mode fixture range");
    animator.Update(10.0 / 288.0);
    animator.Pause();
    const double sampleTime = animator.GetSampleTimeTicks();
    Near(sampleTime, 15, "mode fixture sample time");
    animator.SetInterpolationMode(InterpolationMode::NonIncremental);
    Check(!animator.IsPlaying(), "mode switch preserves pause");
    Near(animator.GetPlaybackSpeed(), 2, "mode switch preserves speed");
    Near(animator.GetSampleTimeTicks(), sampleTime, "mode switch preserves clock and range");
    animator.EvaluatePose(sampleTime);
    const VQS expected = VQS::Interpolate(
        VQS(glm::vec3(-2,-2,1),Quaternion(1,0,0,0),1),
        VQS(glm::vec3(0,0,2),Quaternion(2,0,0,0),1),
        VQS(glm::vec3(4,0,4),Quaternion(-half,0,0,-half),5),
        VQS(glm::vec3(6,-2,5),Quaternion(1,0,0,0),1),
        0.5f, 0.25f, 0.25f);
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            Near(animator.GetNodeTransforms().at("joint")[column][row],
                expected.ToMatrix()[column][row], "non-incremental independent track sampling");
    Near(animator.GetNodeTransforms().at("joint")[3][1], 0.75, "non-incremental Bezier neighbors");
    animator.EvaluatePose(-1);
    Near(animator.GetNodeTransforms().at("joint")[3][0], -2, "non-incremental first boundary");
    animator.EvaluatePose(100);
    Near(animator.GetNodeTransforms().at("joint")[3][0], 6, "non-incremental last boundary");
    animator.SetInterpolationMode(InterpolationMode::Incremental);
    animator.EvaluatePose(sampleTime);
    Near(animator.GetNodeTransforms().at("joint")[3][1], 0, "switch back restores incremental pose");
    animator.Play();
    animator.SetInterpolationMode(InterpolationMode::NonIncremental);
    Check(animator.IsPlaying(), "mode switch preserves play");
    Check(animator.SetAnimation(0), "same clip selection succeeds");
    Check(animator.GetInterpolationMode() == InterpolationMode::NonIncremental, "clip selection preserves mode");
    m=sample(10); Near(m[3][0],0,"exact interior key x"); Near(m[3][2],2,"exact interior key z");
    m=sample(20); Near(m[3][0],4,"exact second interior key");
    m=sample(30); Near(m[3][0],5,"last segment linear x");
    Near(m[3][1],-1,"last segment linear y"); Near(m[3][2],4.5,"last segment z");
    m=sample(-1); Near(m[3][0],-2,"before first position"); Near(m[2][2],1,"before first scale");
    m=sample(100); Near(m[3][0],6,"after last position"); Near(m[2][2],5,"after last scale");

    // Missing tracks retain the node's decomposed defaults.
    channel->mNumScalingKeys=0; channel->mNumRotationKeys=0;
    // Explicit T * R * S: 90 degrees about Z, uniform scale 2.
    joint->mTransformation=aiMatrix4x4(0,-2,0,7, 2,0,0,8, 0,0,2,9, 0,0,0,1);
    m=sample(15); Near(m[0][1],2,"missing rotation keeps default");
    Near(m[1][0],-2,"missing uniform scale keeps default"); Near(m[2][2],2,"default z scale");
    Near(m[3][1],0,"position track still sampled");
    m=sample(15, InterpolationMode::NonIncremental);
    Near(m[0][1],2,"non-incremental missing rotation default");
    Near(m[2][2],2,"non-incremental missing scale default");
    channel->mNumPositionKeys=0;
    m=sample(15); Near(m[3][0],7,"empty position track default x"); Near(m[3][1],8,"empty position default y");
    m=sample(15, InterpolationMode::NonIncremental);
    Near(m[3][0],7,"non-incremental empty position default");
    channel->mNumPositionKeys=1;
    m=sample(15); Near(m[3][0],-2,"single position key held"); Near(m[3][2],1,"single position z held");
    channel->mNumScalingKeys=1; channel->mNumRotationKeys=1;
    m=sample(15); Near(m[0][0],1,"single rotation and scale key"); Near(m[1][1],1,"single uniform scale");
    m=sample(15, InterpolationMode::NonIncremental);
    Near(m[3][0],-2,"non-incremental single position held");
    Near(m[0][0],1,"non-incremental single rotation and scale held");

    // No animation channel preserves the original matrix.
    channel->mNodeName=aiString("different-node");
    joint->mTransformation=aiMatrix4x4(); joint->mTransformation.a4=7;
    m=sample(15); Near(m[0][0],1,"unanimated node keeps full matrix"); Near(m[3][0],7,"unanimated translation");
    Check(animator.GetInterpolationMode() == InterpolationMode::Incremental, "reset restores interpolation default");
}

void TestVQSFactors()
{
    const Quaternion q0(1,0,0,0), q1(0,0,0,1);
    for (int k : {0, 1, 30, 60})
    {
        const float u = float(k) / 60;
        const Quaternion incremental = q0.ISlerp(q0,q1,60,k);
        const Quaternion expected = q0.Slerp(q0,q1,u);
        Near(incremental._s,expected._s,"incremental rotation scalar");
        Near(incremental._z,expected._z,"incremental rotation vector");
        Near(VQS::ILerp(glm::vec3(0),glm::vec3(6),60,k).x,6*u,"incremental position");
        Near(VQS::IELerp(1,4,60,k),std::pow(4.0f,u),"incremental scale");
    }
    Near(VQS::IELerp(0,4,60,30),2,"zero scale fallback");
    bool rejected = false;
    try { VQS::IELerp(1,4,0,0); }
    catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected,"invalid subdivision count rejected");
    const Quaternion identity(1,0,0,0);
    const VQS previous(glm::vec3(-2,-2,0),identity,1);
    const VQS start(glm::vec3(0),identity,1);
    const VQS end(glm::vec3(4,0,0),Quaternion(0,0,0,1),5);
    const VQS next(glm::vec3(6,-2,0),identity,1);
    const VQS result=VQS::Interpolate(previous,start,end,next,0.5f,0.0f,0.25f);
    Near(result._translation.y,0.75,"translation factor");
    Near(result._rotation._s,1,"rotation factor"); Near(result._scale,std::pow(5.0f,0.25f),"scale factor");
    const VQS common=VQS::Interpolate(previous,start,end,next,0.5f);
    Near(common._scale,std::sqrt(5.0f),"single-factor overload");
    Near(common._rotation._s,std::sqrt(0.5f),"single-factor rotation");
}

void TestAnimationImport()
{
    Check(!ImportAnimationData(nullptr).hasRoot, "null import is empty");
    AnimationData data;
    {
        aiScene scene;
        scene.mRootNode = new aiNode("root");
        scene.mRootNode->mTransformation.a4 = 10;
        scene.mRootNode->mNumChildren = 2;
        scene.mRootNode->mChildren = new aiNode*[2]{new aiNode("scaled"), new aiNode("other")};
        auto* scaled = scene.mRootNode->mChildren[0];
        scaled->mTransformation.a1 = 2;
        scaled->mTransformation.b2 = 2;
        scaled->mTransformation.c3 = 2;
        scene.mNumAnimations = 1;
        scene.mAnimations = new aiAnimation*[1]{new aiAnimation()};
        auto* clip = scene.mAnimations[0];
        clip->mName = aiString("clip");
        clip->mDuration = 75;
        clip->mTicksPerSecond = 0;
        clip->mNumChannels = 2;
        clip->mChannels = new aiNodeAnim*[2]{new aiNodeAnim(), new aiNodeAnim()};
        auto* channel = clip->mChannels[0];
        channel->mNodeName = aiString("scaled");
        channel->mNumPositionKeys = 2;
        channel->mPositionKeys = new aiVectorKey[2]{aiVectorKey(3, aiVector3D(1,2,3)), aiVectorKey(17, aiVector3D(4,5,6))};
        channel->mNumRotationKeys = 1;
        channel->mRotationKeys = new aiQuatKey[1]{aiQuatKey(11, aiQuaternion(0.5f,0.25f,-0.5f,0.75f))};
        channel->mNumScalingKeys = 1;
        channel->mScalingKeys = new aiVectorKey[1]{aiVectorKey(23, aiVector3D(2,2,2))};
        scene.mNumMeshes = 2;
        scene.mMeshes = new aiMesh*[2]{new aiMesh(), new aiMesh()};
        for (unsigned int i = 0; i < 2; ++i)
        {
            auto* mesh = scene.mMeshes[i];
            mesh->mNumBones = 1;
            mesh->mBones = new aiBone*[1]{new aiBone()};
            mesh->mBones[0]->mName = aiString("scaled");
            mesh->mBones[0]->mOffsetMatrix.a4 = 5.0f + i;
            mesh->mBones[0]->mOffsetMatrix.b1 = 0.125f;
        }
        data = ImportAnimationData(&scene);
        channel->mPositionKeys[0].mValue.x = 99;
        scene.mRootNode->mName = aiString("changed");
    } // Imported data must survive destruction of the Assimp scene.
    Check(data.hasRoot && data.root.name == "root", "import owns node names");
    Check(data.root.children.size() == 2, "hierarchy preserved");
    Near(data.root.bindVQS._translation.x, 10, "bind translation");
    Near(data.inverseRootMatrix[3][0], -10, "inverse root matrix convention");
    const auto& scaled = data.root.children[0];
    Near(scaled.bindVQS._scale, 2, "uniform bind scale" );
    Near(scaled.bindMatrix[2][2], 2, "exact bind matrix" );
    const auto& clip = data.clips.at(0);
    Check(clip.name == "clip", "clip name");
    Near(clip.durationTicks, 75, "clip duration");
    Near(clip.ticksPerSecond, 0, "unspecified rate preserved");
    const auto& channel = clip.channels.at(0);
    Check(channel.nodeName == "scaled" && channel.positions.size() == 2, "channel and key count");
    Near(channel.positions[0].value.x, 1, "import owns key values");
    Near(channel.positions[1].timeTicks, 17, "position timestamp");
    Near(channel.rotations[0].timeTicks, 11, "independent rotation timestamp");
    Near(channel.rotations[0].value._s, 0.5, "quaternion scalar order");
    Near(channel.rotations[0].value._x, 0.25, "quaternion x order and copy references");
    Near(channel.rotations[0].value._y, -0.5, "quaternion y order");
    Near(channel.rotations[0].value._z, 0.75, "quaternion z order");
    Near(channel.scales[0].timeTicks, 23, "independent scale timestamp");
    Near(channel.scales[0].value, 2, "scalar scale key");
    Check(clip.channels[1].rotations.empty(), "missing tracks remain empty");
    Near(data.meshBones[0][0].offsetMatrix[0][1], 0.125, "offset matrix convention");
    Near(data.meshBones[1][0].offsetMatrix[3][0], 6, "shared name preserves mesh offset");
    Animator animator;
    animator.Reset(&data);
    Check(animator.GetAnimationName(0) == "clip", "custom clip name");
    Check(animator.GetAnimationName(99).empty(), "invalid clip name");
    Near(animator.GetFirstKeyTimeTicks(), 3, "earliest independent key time");
    animator.Play();
    animator.Update(0.5);
    Near(animator.GetSampleTimeTicks(), 12.5, "clock after source scene destruction");
    animator.EvaluatePose(17);
    Near(animator.GetNodeTransforms().at("scaled")[3][0], 4, "pose after source scene destruction");
    Near(animator.GetNodeTransforms().at("scaled")[3][1], 5, "imported position used");
    Near(channel.rotations[0].value._s, 0.5, "sampling preserves custom quaternion");
    Near(channel.rotations[0].value._z, 0.75, "sampling preserves quaternion vector");
    animator.Reset();
    data = ImportAnimationData(nullptr);
    Check(!data.hasRoot && data.clips.empty() && data.meshBones.empty(), "replacement clears old data");
}
void TestNonuniformScaleRejected()
{
    aiScene scene;
    scene.mRootNode = new aiNode("scaled-node");
    const auto expectRejected = [&](const char* context) {
        bool rejected = false;
        try { ImportAnimationData(&scene); }
        catch (const std::runtime_error& error)
        {
            const std::string message = error.what();
            rejected = message.find("scaled-node") != std::string::npos &&
                message.find(context) != std::string::npos;
        }
        Check(rejected, "nonuniform scale rejected with node and context");
    };
    scene.mRootNode->mTransformation.b2 = 2;
    expectRejected("bind transform");
    scene.mRootNode->mTransformation = aiMatrix4x4();
    scene.mNumAnimations = 1;
    scene.mAnimations = new aiAnimation*[1]{new aiAnimation()};
    auto* clip = scene.mAnimations[0];
    clip->mNumChannels = 1;
    clip->mChannels = new aiNodeAnim*[1]{new aiNodeAnim()};
    auto* channel = clip->mChannels[0];
    channel->mNodeName = aiString("scaled-node");
    channel->mNumScalingKeys = 2;
    channel->mScalingKeys = new aiVectorKey[2]{
        aiVectorKey(0, aiVector3D(1,1,1)), aiVectorKey(10, aiVector3D(2,3,2))};
    expectRejected("tick 10");
    channel->mScalingKeys[1].mValue = aiVector3D(2,2,3);
    expectRejected("tick 10");
    channel->mScalingKeys[1].mValue = aiVector3D(2,2.000001f,2);
    scene.mRootNode->mTransformation.b2 = 1.000001f;
    const AnimationData data = ImportAnimationData(&scene);
    Near(data.clips[0].channels[0].scales[1].value, 2, "roundoff accepted as uniform");
}
void TestModelAssets(const char* snapshotPath)
{
    std::ofstream snapshot(snapshotPath);
    Check(snapshot.good(), "open asset pose snapshot");
    snapshot << std::setprecision(9);
    for (const char* name : {"roman_D", "viking_C", "egyptian_B"})
    {
        Assimp::Importer importer;
        importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
        const std::string path = std::string("CS560Framework/fbx/fbx/") + name + ".fbx";
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate |
            aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices);
        Check(scene != nullptr, "load model asset");
        AnimationData data;
        try { data = ImportAnimationData(scene); }
        catch (const std::runtime_error& error)
        {
            if (std::string(name) != "viking_C" || std::string(error.what()).find("'shield'") == std::string::npos) throw;
            std::cout << "Rejected " << name << ": " << error.what() << '\n';
            snapshot << "Rejected " << name << ": " << error.what() << '\n';
            continue;
        }
        std::vector<std::vector<aiMatrix4x4>> referenceOffsets(scene->mNumMeshes);
        for (unsigned int mesh = 0; mesh < scene->mNumMeshes; ++mesh)
            for (unsigned int bone = 0; bone < scene->mMeshes[mesh]->mNumBones; ++bone)
                referenceOffsets[mesh].push_back(scene->mMeshes[mesh]->mBones[bone]->mOffsetMatrix);
        importer.FreeScene();
        Animator animator;
        animator.Reset(&data);
        Check(data.hasRoot && !data.clips.empty(), "model has animation");
        for (unsigned int clip = 0; clip < data.clips.size(); ++clip)
        {
            animator.SetAnimation(clip);
            const double duration = data.clips[clip].durationTicks;
            for (double time : {animator.GetFirstKeyTimeTicks(), duration * 0.25, duration * 0.5, duration})
            {
                animator.EvaluatePose(time);
                // Verify the GLM skinning product against Assimp's matrix multiplication
                // and the original imported offsets. Production uses no Assimp matrices.
                for (unsigned int mesh = 0; mesh < data.meshBones.size(); ++mesh)
                    for (unsigned int bone = 0; bone < data.meshBones[mesh].size(); ++bone)
                    {
                        const auto& importedBone = data.meshBones[mesh][bone];
                        const auto& nodeMatrix = animator.GetNodeTransforms().at(importedBone.name);
                        const glm::mat4 finalMatrix = nodeMatrix * importedBone.offsetMatrix;
                        aiMatrix4x4 referenceNode;
                        for (int row = 0; row < 4; ++row)
                            for (int column = 0; column < 4; ++column)
                                referenceNode[row][column] = nodeMatrix[column][row];
                        const aiMatrix4x4 referenceFinal = referenceNode * referenceOffsets[mesh][bone];
                        for (int row = 0; row < 4; ++row)
                            for (int column = 0; column < 4; ++column)
                            {
                                const float actual = finalMatrix[column][row];
                                const float expected = referenceFinal[row][column];
                                Check(std::isfinite(actual) && std::abs(actual - expected) <=
                                    1e-5f * (1.0f + std::abs(expected)), "asset final bone matrix convention");
                            }
                    }
                for (const auto& node : animator.GetNodeTransforms())
                {
                    snapshot << name << ' ' << clip << ' ' << time << ' ' << node.first;
                    for (int row = 0; row < 4; ++row)
                        for (int column = 0; column < 4; ++column)
                        {
                            const float value = node.second[column][row];
                            Check(std::isfinite(value), "finite asset pose matrix");
                            snapshot << ' ' << value;
                        }
                    snapshot << '\n';
                }
            }
        }
        if (std::string(name) == "roman_D")
        {
            animator.SetAnimation(0);
            for (const AnimationPreset& preset : AnimationPresets)
            {
                Check(animator.SetPlaybackRange(preset.range), "preset fits Roman source clip");
                animator.EvaluatePose(animator.GetSampleTimeTicks());
                Check(!animator.GetNodeTransforms().empty(), "preset evaluates asset pose");
            }
        }
        std::cout << "Verified " << name << ": " << data.clips.size() << " clips\n";
    }
}
void TestPlaybackRanges()
{
    AnimationData data;
    data.clips.resize(2);
    data.clips[0].durationTicks = 600.0;
    data.clips[0].ticksPerSecond = 10.0;
    data.clips[1].durationTicks = 600.0; // Unspecified rate uses 25 ticks/sec.
    Animator animator;
    Check(!animator.SetPlaybackRange({340.0, 363.0}), "range requires clip");
    animator.Reset(&data);
    Check(animator.SetPlaybackRange({340.0, 363.0}), "select run range");
    Near(animator.GetSampleTimeTicks(), 340.0, "range starts at absolute tick");
    animator.Play();
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 350.0, "range seconds to ticks");
    animator.Pause();
    animator.Update(5.0);
    Near(animator.GetSampleTimeTicks(), 350.0, "paused range holds tick");
    for (AnimationRange invalid : {AnimationRange{-1.0, 363.0},
         AnimationRange{363.0, 363.0}, AnimationRange{400.0, 363.0},
         AnimationRange{340.0, 601.0},
         AnimationRange{0.0, std::numeric_limits<double>::infinity()},
         AnimationRange{std::numeric_limits<double>::quiet_NaN(), 363.0}})
        Check(!animator.SetPlaybackRange(invalid), "reject invalid range");
    Near(animator.GetSampleTimeTicks(), 350.0, "invalid range preserves clock");
    animator.Play();
    animator.Update(1.3);
    Near(animator.GetSampleTimeTicks(), 340.0, "run wraps at 363");
    animator.Update(7.4);
    Near(animator.GetSampleTimeTicks(), 345.0, "large delta wraps multiple ranges");
    Check(animator.SetPlaybackRange({443.0, 515.0}), "switch to attack");
    Near(animator.GetSampleTimeTicks(), 443.0, "selection restarts range");
    animator.Update(7.2);
    Near(animator.GetSampleTimeTicks(), 443.0, "attack wraps at 515");
    Check(animator.SetPlaybackRange({0.0, 515.0}), "select cycle");
    animator.Update(51.5);
    Near(animator.GetSampleTimeTicks(), 0.0, "cycle wraps at 515 before source end");
    animator.SetAnimation(1);
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 25.0, "clip change clears range and uses fallback");
    animator.SetPlaybackRange({364.0, 442.0});
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 389.0, "range uses fallback tick rate");
    animator.Reset(&data);
    Near(animator.GetSampleTimeTicks(), 0.0, "reset clears range");
}

void TestFrameRateIndependentPlayback()
{
    AnimationData data;
    data.clips.resize(1);
    data.clips[0].durationTicks = 600.0;
    data.clips[0].ticksPerSecond = 25.0;
    for (double speed : {0.5, 1.0, 3.0})
    {
        Animator at60, at144;
        for (Animator* animator : {&at60, &at144})
        {
            animator->Reset(&data);
            animator->SetPlaybackRange({340.0, 363.0});
            animator->SetPlaybackSpeed(speed);
            animator->Play();
        }
        for (int frame = 0; frame < 60 * 5; ++frame)
            at60.Update(1.0 / 60.0);
        for (int frame = 0; frame < 144 * 5; ++frame)
            at144.Update(1.0 / 144.0);
        Near(at144.GetSampleTimeTicks(), at60.GetSampleTimeTicks(),
            "render rate does not change animation playback");
    }
}

void TestPlaybackSpeed()
{
    AnimationData data;
    data.clips.resize(2);
    for (AnimationClip& clip : data.clips)
    {
        clip.durationTicks = 600.0;
        clip.ticksPerSecond = 10.0;
    }
    Animator animator;
    animator.Reset(&data);
    Near(animator.GetPlaybackSpeed(), 1.0, "default speed");
    animator.SetPlaybackRange({340.0, 363.0});
    animator.Play();
    Check(animator.SetPlaybackSpeed(0.5), "select half speed");
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 345.0, "half speed advances five ticks");
    Check(animator.SetPlaybackSpeed(2.0), "select double speed");
    Near(animator.GetSampleTimeTicks(), 345.0, "speed change preserves position");
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 342.0, "double speed wraps selected range");
    animator.Pause();
    animator.SetPlaybackSpeed(3.0);
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 342.0, "speed change does not resume playback");
    animator.Play();
    animator.Update(0.5);
    Near(animator.GetSampleTimeTicks(), 357.0, "resume uses new speed");
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::quiet_NaN()})
        Check(!animator.SetPlaybackSpeed(invalid), "reject invalid speed");
    Near(animator.GetPlaybackSpeed(), 3.0, "invalid speed preserves multiplier");
    animator.Update(std::numeric_limits<double>::max());
    Near(animator.GetSampleTimeTicks(), 357.0, "scaled delta overflow preserves position");
    animator.SetPlaybackRange({443.0, 515.0});
    Near(animator.GetPlaybackSpeed(), 3.0, "range change preserves speed");
    animator.SetAnimation(1);
    Near(animator.GetPlaybackSpeed(), 3.0, "clip change preserves speed");
    animator.SetPlaybackSpeed(1.0);
    animator.Update(1.0);
    Near(animator.GetSampleTimeTicks(), 10.0, "normal speed restored");
    animator.SetPlaybackSpeed(2.0);
    animator.Reset(&data);
    Near(animator.GetPlaybackSpeed(), 1.0, "reset restores normal speed");
}

void MeasureFramePacing()
{
    const int frames = 120;
    const auto frameDuration = std::chrono::duration<double>(
        1.0 / TimingSettings::TargetFramesPerSecond);
    auto start = FrameLimiter::Clock::now();
    for (int frame = 0; frame < frames; ++frame)
        std::this_thread::sleep_until(FrameLimiter::Clock::now() + frameDuration);
    const double oldSeconds = std::chrono::duration<double>(
        FrameLimiter::Clock::now() - start).count();

    FrameLimiter limiter(TimingSettings::TargetFramesPerSecond);
    start = FrameLimiter::Clock::now();
    for (int frame = 0; frame < frames; ++frame)
        limiter.Wait(FrameLimiter::Clock::now());
    const double newSeconds = std::chrono::duration<double>(
        FrameLimiter::Clock::now() - start).count();
    std::cout << "Frame pacing only (no rendering): old sleep = " << frames / oldSeconds
              << " FPS, high-resolution limiter = " << frames / newSeconds << " FPS\n";
}

int main(int argc, char** argv)
{
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--frame-pacing")
        {
            MeasureFramePacing();
            return 0;
        }
        if (argc == 2) TestModelAssets(argv[1]);
        TestNonuniformScaleRejected();
        TestAnimationImport();
        TestVQSSampling();
        TestVQSFactors();
        TestPlaybackRanges();
        TestPlaybackSpeed();
        TestFrameRateIndependentPlayback();
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
        AnimationData animationData = ImportAnimationData(&scene);
        animator.Reset(&animationData);
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
        animator.EvaluatePose(animator.GetSampleTimeTicks());
        const glm::mat4 movingPose = animator.GetNodeTransforms().at("joint");
        animator.Pause();
        animator.Update(1.0);
        animator.EvaluatePose(animator.GetSampleTimeTicks());
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                Near(animator.GetNodeTransforms().at("joint")[column][row],
                    movingPose[column][row], "paused pose remains frozen");
        animator.Play();
        animator.Update(0.25);
        animator.EvaluatePose(animator.GetSampleTimeTicks());
        Check(std::abs(animator.GetNodeTransforms().at("joint")[3][0] - movingPose[3][0]) > 1e-5,
            "resumed playback changes joint pose");
        animator.Update(1.75);
        animator.EvaluatePose(animator.GetSampleTimeTicks());
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                Near(animator.GetNodeTransforms().at("joint")[column][row],
                    movingPose[column][row], "loop returns to the same joint pose");
        animator.EvaluatePose(25.0);
        Near(animator.GetNodeTransforms().at("joint")[3][0],1,"two-key linear quarter");
        animator.EvaluatePose(25.1);
        Near(animator.GetNodeTransforms().at("joint")[3][0],1,"pose holds within fixed step");
        animator.EvaluatePose(25.9);
        Near(animator.GetNodeTransforms().at("joint")[3][0],37.0/36.0,"pose advances at 144 Hz");
        animator.EvaluatePose(50.0);
        const auto& pose = animator.GetNodeTransforms();
        Near(pose.at("joint")[3][0], 2, "position interpolation and inverse root");
        Near(pose.at("joint")[3][1], 3, "missing scale/rotation preserve defaults");
        Near(pose.at("root")[3][0], 0, "root coordinate system");
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
        Check(animator.GetNodeTransforms().empty() && !animator.IsPlaying(), "reset detaches animation data");
        Near(animator.GetPlaybackSeconds(), 0, "reset clock");
        std::cout << "Animator clock, VQS sampling, and pose tests passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
