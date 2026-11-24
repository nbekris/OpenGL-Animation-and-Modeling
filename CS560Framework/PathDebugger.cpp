#include "PathDebugger.h"
#include "Path.h"
#include "transform.h"
#include "imgui.h"
#include <cmath>
#include <string>
#include <glbinding/gl/gl.h>
using namespace gl;

void PathDebugger::Release()
{
    for (auto& lines : pathDebugLines) lines.reset();
}

bool PathDebugger::DrawMenu(const Path& travelPath, double& pathDistance,
    double& pathLoopSeconds, bool& pathLooping)
{
    bool scrubbed = false;
    if (ImGui::CollapsingHeader("Path debug", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Show path", &showPathDebug);
        ImGui::SameLine();
        ImGui::Checkbox("Draw on top", &pathDebugOnTop);
        ImGui::Checkbox("Show control polygon", &showControlPolygon);
        ImGui::Checkbox("Play locomotion", &pathLooping);
        float seconds = static_cast<float>(pathLoopSeconds);
        if (ImGui::SliderFloat("Travel seconds", &seconds, 1.f, 60.f))
            pathLoopSeconds = seconds;
        const double length = travelPath.TotalLength();
        float distance = length > 0.0 ? static_cast<float>(pathDistance / length) : 0.0f;
        if (ImGui::SliderFloat("Normalized distance", &distance, 0.f, 1.f, "%.3f"))
        {
            pathDistance = distance * length;
            scrubbed = true;
            pathLooping = false;
        }
        for (int i = 0; i < static_cast<int>(travelPath.points.size()); ++i)
        {
            ImGui::PushID(i);
            if (i) ImGui::SameLine();
            if (ImGui::SmallButton(std::to_string(i).c_str()))
            {
                pathDistance = travelPath.NormalizedDistanceAtParameter(double(i)) * length;
                scrubbed = true;
                pathLooping = false;
            }
            ImGui::PopID();
        }
        const float s = length > 0.0 ? static_cast<float>(pathDistance / length) : 0.0f;
        const double pathParam = travelPath.ParameterAtDistance(s);
        ImGui::Text("Path parameter: %.3f", pathParam);
        const auto p = travelPath.Position(pathParam);
        ImGui::Text("Offset: (%.3f, %.3f, %.3f)", p.x, p.y, p.z);
        ImGui::TextWrapped("Cyan: spline. Orange: control polygon and points. Green: point 0. White: current position and forward tangent. Click 0-7 to check each point.");
    }
    return scrubbed;
}

void PathDebugger::Initialize(const Path& travelPath)
{
    for (auto& lines : pathDebugLines) lines = std::make_unique<LineRenderer>();
    std::vector<glm::vec3> curve, polygon, markers, start;
    const int count = static_cast<int>(travelPath.points.size());
    const int samples = travelPath.SegmentCount() * 64;
    for (int i = 0; i < samples; ++i)
    {
        curve.push_back(travelPath.Position(double(i) / 64.0));
        curve.push_back(travelPath.Position(double(i + 1) / 64.0));
    }
    for (int i = 0; i < count; ++i)
    {
        const auto p = travelPath.points[i];
        if (travelPath.closed || i + 1 < count)
        {
            polygon.push_back(p);
            polygon.push_back(travelPath.points[(i + 1) % count]);
        }
        auto& endpoints = i == 0 ? start : markers;
        constexpr int segments = 32;
        constexpr float radius = 0.6f;
        auto previous = p + glm::vec3(radius, 0.f, 0.f);
        for (int segment = 1; segment <= segments; ++segment)
        {
            const float angle = glm::radians(360.f * (segment % segments) / segments);
            const auto next = p + glm::vec3(radius * std::cos(angle), radius * std::sin(angle), 0.f);
            endpoints.push_back(previous);
            endpoints.push_back(next);
            previous = next;
        }
    }
    pathDebugLines[0]->SetLines(curve);
    pathDebugLines[1]->SetLines(polygon);
    pathDebugLines[2]->SetLines(markers);
    pathDebugLines[3]->SetLines(start);
}

void PathDebugger::Draw(const Path& travelPath, double pathParam,
    const glm::mat4& modelTransform, ShaderProgram& shader,
    glm::mat4& worldProj, glm::mat4& worldView)
{
    if (!showPathDebug || !pathDebugLines[0]) return;
    const auto p = travelPath.Position(pathParam);
    std::vector<glm::vec3> marker;
    for (int axis = 0; axis < 3; ++axis)
    {
        glm::vec3 extent(0.f);
        extent[axis] = 0.45f;
        marker.push_back(p - extent);
        marker.push_back(p + extent);
    }
    const auto tangent = travelPath.Position(pathParam + 0.001)
                       - travelPath.Position(pathParam - 0.001);
    if (glm::length(tangent) > 0.00001f)
    {
        marker.push_back(p);
        marker.push_back(p + glm::normalize(tangent) * 1.5f);
    }
    pathDebugLines[4]->SetLines(marker);

    // Path values are world-space offsets. Include the model's base translation
    // so the white cross tracks its actual origin, without scaling the path.
    const glm::vec3 base = glm::vec3(modelTransform[3]) - p;
    glm::mat4 transform = Translate(base.x, base.y, base.z);
    const glm::vec3 colors[] = {
        {0.2f, 0.9f, 1.f}, {1.f, 0.6f, 0.15f}, {1.f, 0.6f, 0.15f},
        {0.2f, 1.f, 0.3f}, {1.f, 1.f, 1.f}
    };
    const auto depthEnabled = glIsEnabled(GL_DEPTH_TEST);
    GLboolean depthWrite;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWrite);
    if (pathDebugOnTop) glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    for (int i = 0; i < 5; ++i)
    {
        if (i == 1 && !showControlPolygon) continue;
        pathDebugLines[i]->Draw(shader, worldProj, worldView, transform, colors[i]);
    }
    glDepthMask(depthWrite);
    if (depthEnabled) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
}
