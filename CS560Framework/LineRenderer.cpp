#include "LineRenderer.h"

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#include "shader.h"
#include "transform.h"

void LineRenderer::SetLines(const std::vector<glm::vec3>& endpoints)
{
    // GL_LINES consumes vertices in pairs; discard an unmatched endpoint.
    vertexCount = static_cast<int>(endpoints.size() - endpoints.size() % 2);

    if (vao == 0)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
    }

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(glm::vec3),
        vertexCount > 0 ? endpoints.data() : nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    // The shader takes a vec4; OpenGL supplies 1.0 for its unspecified w component.
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void LineRenderer::Draw(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView,
    glm::mat4& modelTransform, const glm::vec3& color)
{
    if (vertexCount == 0)
        return;

    shader.UseShader();
    const int programId = shader.programId;

    int location = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(worldProj));
    location = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(worldView));
    location = glGetUniformLocation(programId, "ModelTr");
    glUniformMatrix4fv(location, 1, GL_FALSE, Pntr(modelTransform));
    location = glGetUniformLocation(programId, "useFloorGradient");
    glUniform1i(location, 0);
    location = glGetUniformLocation(programId, "lineColor");
    glUniform3fv(location, 1, &color[0]);

    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, vertexCount);
    glBindVertexArray(0);
    shader.UnuseShader();
}
