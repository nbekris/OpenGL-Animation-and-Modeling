
#include "math.h"
#include <iostream>
#include <stdlib.h>
#include <memory>

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "framework.h"
#include "AnimationPresets.h"
#include "TimingSettings.h"
#include <cmath>
//#include "shapes.h"
#include "texture.h"
#include "transform.h"

const float PI = 3.14159f;
const float rad = PI/180.0f;

namespace
{
    constexpr int RunPresetIndex = 2;

    const char* ModelNames[] = { "Roman", "Viking" };
    const char* ModelPaths[] = { "fbx/fbx/roman_D.fbx", "fbx/fbx/viking_C.fbx" };
}

bool Scene::SelectModel(int index)
{
    try
    {
        if (index < 0 || index >= static_cast<int>(loadedModels.size()) || !loadedModels[index])
        {
            throw std::runtime_error("Model is not loaded");
        }

        Mesh* replacement = loadedModels[index].get();
        replacement->GetAnimator().Reset(&replacement->GetAnimationData());

        if (replacement->SetAnimation(0))
        {
            Animator& animator = replacement->GetAnimator();
            if (animator.SetPlaybackRange(AnimationPresets[0].range))
            {
                animator.Play();
            }
        }
        model = replacement;
        selectedModel = index;
        selectedAnimationPreset = 0;
        modelLoadError.clear();
        if (synchronizeLocomotion && !ConfigureLocomotion())
        {
            synchronizeLocomotion = false;
            model->GetAnimator().Pause();
        }
        return true;
    }
    catch (const std::exception& error)
    {
        modelLoadError = error.what();
        return false;
    }
}

bool Scene::ConfigureLocomotion()
{
    if (!model)
        return false;

    Animator& animator = model->GetAnimator();
    if (!animator.SetPlaybackRange(AnimationPresets[RunPresetIndex].range))
    {
        locomotion.SetPlaying(false);
        modelLoadError = "The Run range is invalid for this model";
        return false;
    }

    selectedAnimationPreset = RunPresetIndex;
    // distance controls the animation clock while following the path
    animator.SetPlaybackSpeed(1.0);
    animator.Pause();
    auto settings = locomotion.GetSettings();
    settings.strideWorld = strideWorld[selectedModel];
    settings.assetForwardYawDegrees = assetForwardYawDegrees[selectedModel];
    locomotion.SetSettings(settings, travelPath);
    locomotion.SeekDistance(locomotion.GetSample().distance, travelPath);
    animator.SetCyclePhase(locomotion.GetSample().gaitPhase);
    modelLoadError.clear();
    return true;
}

void Scene::DrawLocomotionMenu()
{
    auto settings = locomotion.GetSettings();
    double distance = locomotion.GetSample().distance;
    bool playing = locomotion.IsPlaying();
    const bool scrubbed = pathDebugger.DrawMenu(
        travelPath, distance, settings.loopSeconds, playing);

    float a = static_cast<float>(settings.easeInEnd);
    float b = static_cast<float>(settings.easeOutStart);
    if (ImGui::SliderFloat("Ease-in end", &a, 0.05f, b - 0.05f, "%.2f", ImGuiSliderFlags_AlwaysClamp))
        settings.easeInEnd = a;
    if (ImGui::SliderFloat("Ease-out start", &b, a + 0.05f, 0.95f, "%.2f", ImGuiSliderFlags_AlwaysClamp))
        settings.easeOutStart = b;

    ImGui::SliderFloat("Stride per cycle", &strideWorld[selectedModel],
        0.05f, 20.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Look-ahead distance", &settings.lookAheadWorld,
        0.01f, 3.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Asset forward heading", &assetForwardYawDegrees[selectedModel],
        -180.0f, 180.0f, "%.1f deg", ImGuiSliderFlags_AlwaysClamp);

    // the menu edits settings, the controller handles clocks and pose changes
    settings.strideWorld = strideWorld[selectedModel];
    settings.assetForwardYawDegrees = assetForwardYawDegrees[selectedModel];
    locomotion.SetPlaying(playing);
    locomotion.SetSettings(settings, travelPath);
    if (scrubbed)
        locomotion.SeekDistance(distance, travelPath);
    if (ImGui::Button("Restart locomotion"))
        locomotion.Restart(travelPath);

    const auto& sample = locomotion.GetSample();
    ImGui::Text("Travel speed: %.3f units/s", sample.speed);
    ImGui::Text("Gait rate: %.3f cycles/s", sample.speed / locomotion.GetSettings().strideWorld);
}

void Scene::ReleaseModels()
{
    // Release debug buffers while the OpenGL context is still alive.
    pathDebugger.Release();
    model = nullptr;
    for (auto& cached : loadedModels)
    {
        cached.reset();
    }
}

glm::mat4 Identity;

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line scene.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

void Scene::InitCamera()
{
    spin = 0.0;
    tilt = 15.0;
    eye = glm::vec3(0.0, -20.0, 10.0);
    speed = 300.0 / 30.0;
    last_time = glfwGetTime();
    tr = glm::vec3(0.0, -3.0, 25.0);

    ry = 0.4f;
    front = 0.5;
    back = 5000.0;
}

void Scene::InitializeScene()
{
    std::cout << "INIT\n";
    glEnable(GL_DEPTH_TEST);
    CHECKERROR;

    glfwGetFramebufferSize(window, &width, &height);

    lightSpin = 90.0;
    lightTilt = -45.0;
    lightDist = 50.0;
    lightCol = { 3.0f, 3.0f, 3.0f };

    key = 0;
    InitCamera();

    glGenBuffers(1, &id);
    bindpoint = 1;
    glBindBufferBase(GL_UNIFORM_BUFFER, bindpoint, id);
    CHECKERROR;

    glEnable(GL_DEPTH_TEST);

    //FBO
    gBuffer.Init(width, height);
    CHECKERROR;

    lightingProgram = new ShaderProgram();
    lightingProgram->AddShader("shaders/lightingPhong.vert", GL_VERTEX_SHADER);
    lightingProgram->AddShader("shaders/lightingPhong.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lightingProgram->programId, 0, "vertex");
    glBindAttribLocation(lightingProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(lightingProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(lightingProgram->programId, 3, "vertexTangent");
    lightingProgram->LinkProgram();

    gBufferProgram = new ShaderProgram();
    gBufferProgram->AddShader("shaders/gBuffer.vert", GL_VERTEX_SHADER);
    gBufferProgram->AddShader("shaders/gBuffer.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(gBufferProgram->programId, 0, "vertex");
    glBindAttribLocation(gBufferProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(gBufferProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(gBufferProgram->programId, 3, "vertexTangent");
    glBindAttribLocation(gBufferProgram->programId, 4, "boneIds");
    glBindAttribLocation(gBufferProgram->programId, 5, "weight");
    gBufferProgram->LinkProgram();

    basicShader = new ShaderProgram();
    basicShader->AddShader("shaders/vertexShader.vert", GL_VERTEX_SHADER);
    basicShader->AddShader("shaders/fragmentShader.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(basicShader->programId, 0, "vertex");
    glBindAttribLocation(basicShader->programId, 1, "vertexNormal");
    glBindAttribLocation(basicShader->programId, 2, "vertexTexture");
    glBindAttribLocation(basicShader->programId, 3, "vertexTangent");
    basicShader->LinkProgram();

    quad = new Quad();
    floor.Initialize();
    pathDebugger.Initialize(travelPath);

    for (int index = 0; index < IM_ARRAYSIZE(ModelPaths); ++index)
    {
        auto cached = std::make_unique<Mesh>();
        cached->LoadMesh(ModelPaths[index]);
        if (!cached->GetAnimationData().hasRoot)
        {
            throw std::runtime_error(std::string("Could not load model: ") + ModelPaths[index]);
        }
        loadedModels[index] = std::move(cached);
    }
    SelectModel(0);

    previousFrameTime = glfwGetTime();
}

void Scene::DrawMenu()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Sample UI Box");
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    if (ImGui::BeginCombo("Model", ModelNames[selectedModel]))
    {
        for (int n = 0; n < IM_ARRAYSIZE(ModelNames); n++)
        {
            bool isSelected = (selectedModel == n);
            if (ImGui::Selectable(ModelNames[n], isSelected) && !isSelected)
            {
                SelectModel(n);
            }

            if (isSelected)
            {
                ImGui::SetItemDefaultFocus();
            }

        }

        ImGui::EndCombo();
    }

    if (!modelLoadError.empty())
    {
        ImGui::TextWrapped("Model load failed: %s", modelLoadError.c_str());
    }

    Animator& animator = model->GetAnimator();

    if (ImGui::Checkbox("Synchronize path and gait", &synchronizeLocomotion))
    {
        if (synchronizeLocomotion)
        {
            if (!ConfigureLocomotion())
                synchronizeLocomotion = false;
        }
        else
        {
            locomotion.SetPlaying(false);
            animator.SetPlaybackSpeed(1.0);
            animator.Play();
        }
    }

    // Imgui logic for handling selection between incremental and non incremental interpolation algorithms
    int interpolationIndex = animator.GetInterpolationMode() == InterpolationMode::Incremental ? 1 : 0;
    const char* interpolationNames[] = { "Slerp", "iSlerp" };
    if (ImGui::Combo("Interpolation", &interpolationIndex, interpolationNames, IM_ARRAYSIZE(interpolationNames)))
    {
        animator.SetInterpolationMode(interpolationIndex == 0 ? InterpolationMode::NonIncremental
                                                              : InterpolationMode::Incremental);
    }

    if (!synchronizeLocomotion)
    {
        if (ImGui::Button(animator.IsPlaying() ? "Pause" : "Play"))
        {
            if (animator.IsPlaying())
            {
                animator.Pause();
            }

            else
            {
                animator.Play();
            }
        }

        // Animation speed selection
        // Implemented to try and check results with the sample project
        float speed = static_cast<float>(animator.GetPlaybackSpeed());
        if (ImGui::SliderFloat("Animation speed", &speed, 0.1f, 3.0f, "%.2fx",
            ImGuiSliderFlags_AlwaysClamp))
        {
            animator.SetPlaybackSpeed(speed);
        }

        ImGui::SameLine();
        if (ImGui::Button("Normal speed"))
        {
            animator.SetPlaybackSpeed(1.0);
        }
    }

    ImGui::Text("Range elapsed: %.3f seconds / Clip tick: %.3f",
        animator.GetPlaybackSeconds(), animator.GetSampleTimeTicks());

    ImGui::Checkbox("Draw Bone", &bone);
    ImGui::SameLine();
    ImGui::Checkbox("Draw Mesh", &mesh);

    if (synchronizeLocomotion)
        DrawLocomotionMenu();

    if (!synchronizeLocomotion)
    {
        // Build animation selection drop down menu
        // Implementation is a little weird because we had to manually figure out
        // animation payback ranges.
        const AnimationPreset& current = AnimationPresets[selectedAnimationPreset];
        if (ImGui::BeginCombo("Animation", current.name))
        {
            for (int index = 0; index < IM_ARRAYSIZE(AnimationPresets); ++index)
            {
                const bool selected = index == selectedAnimationPreset;
                const AnimationPreset& preset = AnimationPresets[index];

                ImGui::PushID(static_cast<int>(index));
                if (ImGui::Selectable(preset.name, selected) &&
                    animator.SetPlaybackRange(preset.range))
                {
                    selectedAnimationPreset = index;
                    animator.Play();
                }

                if (selected)
                {
                    ImGui::SetItemDefaultFocus();
                }

                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
    }

    ImGui::End();


    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

void Scene::BuildTransforms()
{
    const double now = glfwGetTime();
    const double deltaSeconds = now - last_time;
    const float dist = static_cast<float>(deltaSeconds * speed);
    last_time = now;
    if (key == GLFW_KEY_KP_8)
    {
        tr[1] -= dist;
    }

    if (key == GLFW_KEY_KP_2)
    {
        tr[1] += dist;
    }

    if (key == GLFW_KEY_KP_6)
    {
        tr[0] -= dist;
    }

    if (key == GLFW_KEY_KP_4)
    {
        tr[0] += dist;
    }

    if (key == GLFW_KEY_KP_5)
    {
        InitCamera();
    }

    if (key == GLFW_KEY_KP_0)
    {
        spin += dist*20.f;
    }

    const double zoomExponent = std::pow(1.01,
        deltaSeconds * TimingSettings::ZoomReferenceFramesPerSecond);
    if (key == GLFW_KEY_KP_ADD)
    {
        tr[2] = static_cast<float>(std::pow(tr[2], 1.0 / zoomExponent));
    }

    if (key == GLFW_KEY_KP_SUBTRACT)
    {
        tr[2] = static_cast<float>(std::pow(tr[2], zoomExponent));
    }


    CHECKERROR;
    WorldView = Translate(tr[0], tr[1], -tr[2]) * Rotate(0, tilt - 90) * Rotate(2, spin);
    WorldProj = Perspective((ry*width)/height, ry, front, (mode==0) ? 1000 : back);

}

void Scene::DrawScene()
{
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    BuildTransforms();

    WorldInverse = glm::inverse(WorldView);


    int loc, programId;

    //Draw the mesh to the g buffer
    gBufferProgram->UseShader();
    programId = gBufferProgram->programId;

    gBuffer.BindFBO();
    GLenum DrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    glDrawBuffers(4, DrawBuffers);

    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));

    const double now = glfwGetTime();
    const double deltaSeconds = now - previousFrameTime;
    previousFrameTime = now;

    /////////// Path Following //////////////////////////
    Animator& animator = model->GetAnimator();
    if (synchronizeLocomotion)
    {
        const auto sample = locomotion.Update(deltaSeconds, travelPath);
        model->SetPathOffset(sample.position);
        model->SetPathYawDegrees(sample.yawDegrees);
        animator.SetCyclePhase(sample.gaitPhase);
    }
    else
        animator.Update(deltaSeconds);

    /////////////// Animation /////////////////////////
    // Evaluate even while paused so the current playback position defines the pose.
    animator.EvaluatePose(animator.GetSampleTimeTicks());
    model->UpdateSkinningPose();
    model->UpdateSkeletonPose();
    if (mesh)
    {
        model->Draw(programId);
    }

    gBuffer.UnbindFBO();
    gBufferProgram->UnuseShader();


    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ////////////PHONG LIGHTING////////////////////
    lightingProgram->UseShader();
    programId = lightingProgram->programId;


    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    loc = glGetUniformLocation(programId, "WorldInverse");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldInverse));

    loc = glGetUniformLocation(programId, "lightPos");
    glUniform3fv(loc, 1, &(lightPos[0]));
    loc = glGetUniformLocation(programId, "lightCol");
    glUniform3fv(loc, 1, &(lightCol[0]));

    loc = glGetUniformLocation(programId, "width");
    glUniform1i(loc, width);
    loc = glGetUniformLocation(programId, "height");
    glUniform1i(loc, height);
    loc = glGetUniformLocation(programId, "eyePos");
    glUniform3fv(loc, 1, &(eye[0]));

    CHECKERROR;

    gBuffer.BindTexture(0, lightingProgram->programId, "gBufferWorldPos", 0);
    gBuffer.BindTexture(1, lightingProgram->programId, "gBufferNormal", 1);
    gBuffer.BindTexture(2, lightingProgram->programId, "gBufferDiffuse", 2);
    gBuffer.BindTexture(3, lightingProgram->programId, "gBufferSpecular", 3);
    CHECKERROR;

    quad->DrawVAO();
    CHECKERROR;

    lightingProgram->UnuseShader();

    glClear(GL_DEPTH_BUFFER_BIT);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gBuffer.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height,
        0, 0, width, height,
        GL_DEPTH_BUFFER_BIT,
        GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    floor.Draw(*basicShader, WorldProj, WorldView);

    if (bone)
    {
        model->DrawSkeleton(*basicShader, WorldProj, WorldView);
    }

    pathDebugger.Draw(travelPath, locomotion.GetSample().pathParam, model->GetModelTransform(),
        *basicShader, WorldProj, WorldView);

}
