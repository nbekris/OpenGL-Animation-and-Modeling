#pragma once

#include <vector>
#include <glm/glm.hpp>

class ShaderProgram;

// Draws independent GL_LINES. Each consecutive pair of endpoints is one line.
class LineRenderer
{
public:
    void SetLines(const std::vector<glm::vec3>& endpoints);
    void Draw(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView,
        glm::mat4& modelTransform, const glm::vec3& color);

    int GetLineCount() const { return vertexCount / 2; }

private:
    unsigned int vao = 0;
    unsigned int vbo = 0;
    int vertexCount = 0;
};
