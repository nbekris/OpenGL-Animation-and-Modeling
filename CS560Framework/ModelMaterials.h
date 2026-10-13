#pragma once

#include <glbinding/gl/gl.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

struct aiScene;

// Owns a model's surface materials and shared GPU textures.
// Load, Clear, Bind and destruction require a current OpenGL context.
class ModelMaterials
{
public:
    ModelMaterials() = default;
    ~ModelMaterials();
    ModelMaterials(const ModelMaterials&) = delete;
    ModelMaterials& operator=(const ModelMaterials&) = delete;

    void Load(const aiScene& scene, const std::string& modelPath);
    void Clear();
    void Bind(unsigned int materialIndex, bool hasUVs, int programId);
    void Unbind();

private:
    struct Material
    {
        glm::vec3 diffuse = glm::vec3(0.7f, 0.7f, 0.5f);
        glm::vec3 specular = glm::vec3(0.5f);
        float shininess = 1.0f;
        gl::GLuint texture = 0;
    };

    void CacheUniformLocations(int programId);

    std::vector<Material> m_materials;
    std::vector<gl::GLuint> m_textures;
    int m_programId = -1;
    gl::GLint m_diffuseLocation = -1;
    gl::GLint m_specularLocation = -1;
    gl::GLint m_shininessLocation = -1;
    gl::GLint m_hasDiffuseMapLocation = -1;
    gl::GLint m_diffuseMapLocation = -1;
};
