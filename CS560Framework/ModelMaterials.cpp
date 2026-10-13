#include "ModelMaterials.h"
#include <assimp/scene.h>
#include <assimp/material.h>
#include "stb_image.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>

using namespace gl;

namespace
{
    std::string DirectoryOf(const std::string& path)
    {
        const auto slash = path.find_last_of("/\\");
        return slash == std::string::npos ? "." : path.substr(0, slash);
    }

    std::string ResolveTexturePath(std::string key, const std::string& directory, const aiTexture* embedded)
    {
        if (!embedded)
        {
            const std::string filename = key.substr(key.find_last_of('/') + 1);
            const std::vector<std::string> candidates = {
                directory + "/" + key, key, directory + "/" + filename,
                directory + "/textures/" + filename,
                DirectoryOf(directory) + "/textures/" + filename
            };
            for (const auto& candidate : candidates)
            {
                if (std::ifstream(candidate, std::ios::binary).good())
            {
                return candidate;
            }
        }
        }
        return key;
    }

    // Releases stb memory even if material loading exits through an exception.
    struct DecodedImage
    {
        int width = 0;
        int height = 0;
        unsigned char* decoded = nullptr;
        std::vector<unsigned char> raw;

        ~DecodedImage()
        {
            if (decoded)
        {
            stbi_image_free(decoded);
        }
    }
        DecodedImage() = default;
        DecodedImage(const DecodedImage&) = delete;
        DecodedImage& operator=(const DecodedImage&) = delete;

        const unsigned char* Pixels() const
        {
            return decoded ? decoded : (raw.empty() ? nullptr : raw.data());
        }

        void Load(const std::string& key, const aiTexture* embedded)
        {
            int channels = 0;

            // UVs are flipped by Assimp; images must not be flipped a second time.
            stbi_set_flip_vertically_on_load(false);
            if (embedded && embedded->mHeight == 0)
            {
                if (embedded->mWidth <= static_cast<unsigned int>((std::numeric_limits<int>::max)()))
            {
                decoded = stbi_load_from_memory(reinterpret_cast<const unsigned char *>(embedded->pcData),
                                                static_cast<int>(embedded->mWidth), &width, &height, &channels, 4);
            }
        }
            else if (embedded)
            {
                width = static_cast<int>(embedded->mWidth);
                height = static_cast<int>(embedded->mHeight);
                raw.reserve(static_cast<size_t>(width) * height * 4);
                for (size_t pixel = 0; pixel < static_cast<size_t>(width) * height; ++pixel)
                {
                    const auto& texel = embedded->pcData[pixel];
                    raw.insert(raw.end(), {texel.r, texel.g, texel.b, texel.a});
                }
            }
            else
            {
                decoded = stbi_load(key.c_str(), &width, &height, &channels, 4);
            }
        }
    };

    void UploadTexture(GLuint texture, const DecodedImage& image)
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(GL_RGBA8),
            image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.Pixels());
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(GL_LINEAR_MIPMAP_LINEAR));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, static_cast<GLint>(GL_LINEAR));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, static_cast<GLint>(GL_REPEAT));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, static_cast<GLint>(GL_REPEAT));
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

ModelMaterials::~ModelMaterials()
{
    Clear();
}

void ModelMaterials::Clear()
{
    if (!m_textures.empty())
    {
        glDeleteTextures(static_cast<GLsizei>(m_textures.size()), m_textures.data());
    }
    m_textures.clear();
    m_materials.clear();
    m_programId = -1;
}

void ModelMaterials::CacheUniformLocations(int programId)
{
    if (m_programId == programId)
    {
        return;
    }
    m_diffuseLocation = glGetUniformLocation(programId, "diffuse");
    m_specularLocation = glGetUniformLocation(programId, "specular");
    m_shininessLocation = glGetUniformLocation(programId, "shininess");
    m_hasDiffuseMapLocation = glGetUniformLocation(programId, "hasDiffuseMap");
    m_diffuseMapLocation = glGetUniformLocation(programId, "diffuseMap");
    m_programId = programId;
}

void ModelMaterials::Bind(unsigned int materialIndex, bool hasUVs, int programId)
{
    CacheUniformLocations(programId);
    const auto& material = m_materials.at(materialIndex);
    glUniform3fv(m_diffuseLocation, 1, &material.diffuse[0]);
    glUniform3fv(m_specularLocation, 1, &material.specular[0]);
    glUniform1f(m_shininessLocation, material.shininess);
    glUniform1i(m_hasDiffuseMapLocation, material.texture != 0 && hasUVs);
    glUniform1i(m_diffuseMapLocation, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, material.texture);
}

void ModelMaterials::Unbind()
{
    glUniform1i(m_hasDiffuseMapLocation, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void ModelMaterials::Load(const aiScene& importedScene, const std::string& path)
{
    Clear();
    const aiScene* scene = &importedScene;
    const std::string directory = DirectoryOf(path);
    std::map<std::string, GLuint> cache;
    m_materials.resize(scene->mNumMaterials);
    for (unsigned int i = 0; i < scene->mNumMaterials; ++i)
    {
        const aiMaterial* source = scene->mMaterials[i];
        auto& material = m_materials[i];
        aiColor3D color;
        if (source->Get(AI_MATKEY_BASE_COLOR, color) == AI_SUCCESS ||
            source->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS)
        {
            material.diffuse = glm::vec3(color.r, color.g, color.b);
        }
        if (source->Get(AI_MATKEY_COLOR_SPECULAR, color) == AI_SUCCESS)
        {
            material.specular = glm::vec3(color.r, color.g, color.b);
        }
        source->Get(AI_MATKEY_SHININESS, material.shininess);
        aiString reference;
        unsigned int uvChannel = 0;
        aiTextureMapping mapping = aiTextureMapping_UV;
        const bool baseColorMap = source->GetTexture(
            aiTextureType_BASE_COLOR, 0, &reference, &mapping, &uvChannel) == AI_SUCCESS;
        if (!baseColorMap && source->GetTexture(
            aiTextureType_DIFFUSE, 0, &reference, &mapping, &uvChannel) != AI_SUCCESS)
        {
            continue;
        }
        if (mapping != aiTextureMapping_UV || uvChannel != 0)
        {
            std::cerr << "Unsupported diffuse UV mapping: " << reference.C_Str() << '\n';
            continue;
        }
        std::string key = reference.C_Str();
        std::replace(key.begin(), key.end(), '\\', '/');
        const aiTexture* embedded = scene->GetEmbeddedTexture(reference.C_Str());
        key = ResolveTexturePath(key, directory, embedded);
        const auto found = cache.find(key);
        if (found != cache.end())
        {
            material.texture = found->second;
            if (material.texture && !baseColorMap)
            {
                material.diffuse = glm::vec3(1.0f);
            }
            continue;
        }
        DecodedImage image;
        image.Load(key, embedded);
        const unsigned char* pixels = image.Pixels();
        if (!pixels)
        {
            std::cerr << "Unable to load diffuse texture '" << key << "'; using material color\n";
            cache[key] = 0;
            continue;
        }
        glGenTextures(1, &material.texture);
        // Legacy diffuse maps supply the surface color; FBX files may store a black diffuse constant.
        if (!baseColorMap)
        {
            material.diffuse = glm::vec3(1.0f);
        }
        std::cout << "Loaded diffuse texture: " << key << " (" << image.width << "x" << image.height << ")\n";
        m_textures.push_back(material.texture);
        UploadTexture(material.texture, image);
        cache[key] = material.texture;
    }
}

