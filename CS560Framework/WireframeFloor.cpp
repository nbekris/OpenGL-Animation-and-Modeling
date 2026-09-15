#include "WireframeFloor.h"

#include <vector>

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#include "shader.h"
#include "transform.h"

void WireframeFloor::Initialize(int halfCellCount, float cellSize, float height)
{
    std::vector<glm::vec4> vertices;
    vertices.reserve((halfCellCount * 2 + 1) * 4);

    const float extent = halfCellCount * cellSize;
    minX = -extent;
    maxX = extent;
    for (int i = -halfCellCount; i <= halfCellCount; ++i)
    {
        const float coordinate = i * cellSize;
        vertices.emplace_back(-extent, coordinate, height, 1.0f);
        vertices.emplace_back( extent, coordinate, height, 1.0f);
        vertices.emplace_back(coordinate, -extent, height, 1.0f);
        vertices.emplace_back(coordinate,  extent, height, 1.0f);
    }

    vertexCount = static_cast<int>(vertices.size());
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec4),
        vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void WireframeFloor::Draw(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView)
{
    shader.UseShader();
    const int programId = shader.programId;
    glm::mat4 identity(1.0f);

    int location = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(worldProj));
    location = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(worldView));
    location = glGetUniformLocation(programId, "ModelTr");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(identity));
    location = glGetUniformLocation(programId, "floorMinX");
    glUniform1f(location, minX);
    location = glGetUniformLocation(programId, "floorMaxX");
    glUniform1f(location, maxX);
    location = glGetUniformLocation(programId, "useFloorGradient");
    glUniform1i(location, 1);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glLineWidth(1.0f);
    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, vertexCount);
    glBindVertexArray(0);
    glDepthMask(GL_TRUE);

    shader.UnuseShader();
}
