#pragma once

#include "LineRenderer.h"
#include <array>
#include <memory>

class Path;
class ShaderProgram;

// Owns path visualization and its UI; playback state remains with the scene.
class PathDebugger
{
public:
    void Initialize(const Path& travelPath);
    // Call before destroying the OpenGL context.
    void Release();
    bool DrawMenu(const Path& travelPath, double& pathDistance,
        double& pathLoopSeconds, bool& pathLooping);
    void Draw(const Path& travelPath, double pathParam,
        const glm::mat4& modelTransform, ShaderProgram& shader,
        glm::mat4& worldProj, glm::mat4& worldView);

private:
    bool showPathDebug = true;
    bool showControlPolygon = false;
    bool pathDebugOnTop = true;
    // Curve, control polygon, control markers, start marker, moving tangent.
    std::array<std::unique_ptr<LineRenderer>, 5> pathDebugLines;
};
