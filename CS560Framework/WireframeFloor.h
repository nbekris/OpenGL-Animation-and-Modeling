#pragma once

#include <glm/glm.hpp>

class ShaderProgram;

class WireframeFloor
{
public:
    void Initialize(int halfCellCount = 20, float cellSize = 4.0f, float height = -2.5f);
    void Draw(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView);

private:
    unsigned int vao = 0;
    unsigned int vbo = 0;
    int vertexCount = 0;
    float minX = 0.0f;
    float maxX = 0.0f;
};
