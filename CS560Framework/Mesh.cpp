#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)


using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line shapes.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <iomanip>
#include <iostream>
#include <cstddef>

#include "Mesh.h"
#include <assimp/Importer.hpp>
#include "transform.h"
#include "shader.h"

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
    ReleaseGpuResources();
}

void Mesh::ReleaseGpuResources()
{
    m_materials.Clear();
    if (!m_buffers.empty())
    {
        glDeleteBuffers(static_cast<GLsizei>(m_buffers.size()), m_buffers.data());
    }
    if (m_skinBuffer)
    {
        glDeleteBuffers(1, &m_skinBuffer);
    }
    if (m_vao)
    {
        glDeleteVertexArrays(1, &m_vao);
    }
    m_buffers.clear();
    m_skinBuffer = m_vao = 0;
    m_skinProgram = -1;
}

void Mesh::LoadMesh(const std::string& path)
{
	Assimp::Importer importer;
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS, MAX_NUM_BONES_PER_VERTEX);

	const aiScene* scene = importer.ReadFile(path.c_str(), aiProcess_Triangulate
											| aiProcess_GenSmoothNormals
											| aiProcess_FlipUVs
											| aiProcess_JoinIdenticalVertices
                                            | aiProcess_LimitBoneWeights
#ifndef NDEBUG
                                            | aiProcess_ValidateDataStructure
#endif
                                            );
	if (!scene)
	{
		printf("Error parsing '%s': '%s'\n", path.c_str(), importer.GetErrorString());
		return;
	}


    AnimationData imported = ImportAnimationData(scene);
    if (!imported.hasRoot || imported.meshInstances.empty())
    {
        throw std::runtime_error("Imported model has no mesh nodes");
    }
    for (const auto& bindings : imported.meshBones)
    {
        if (bindings.size() > MAX_SKIN_BONES)
        {
            throw std::runtime_error("Submesh exceeds the 128-bone skinning palette");
        }
    }
    m_animator.Reset();
    ReleaseGpuResources();
    m_animationData = std::move(imported);


	m_pnt.clear(); m_norm.clear(); m_tex.clear(); m_tan.clear();
	m_indices.clear(); m_bones.clear();
    m_drawRanges.clear();
    m_materials.Load(*scene, path);
	InitMesh(scene);
	importer.FreeScene();
	m_animator.Reset(&m_animationData);
	BuildFirstAnimationFrameSkeleton();
    UpdateSkinningPose();

	MakeVAO();
}

glm::mat4 Mesh::GetModelTransform() const
{
	return Translate(0.f, 0.f, -2.5f) * Rotate(0, 90.f) * Scale(0.1f, 0.1f, 0.1f);
}

void Mesh::BuildFirstAnimationFrameSkeleton()
{
    // With no clip, evaluation produces bind-pose transforms and endpoints.
    m_animator.EvaluatePose(m_animator.GetFirstKeyTimeTicks());
    PrintBoneCoords(m_animator.GetSkeletonEndpoints());
    UpdateSkeletonPose();
}

void Mesh::UpdateSkeletonPose()
{
    m_skeletonLines.SetLines(m_animator.GetSkeletonEndpoints());
}
bool Mesh::SetAnimation(unsigned int index)
{
    if (!m_animator.SetAnimation(index))
    {
        return false;
    }

    BuildFirstAnimationFrameSkeleton();
    UpdateSkinningPose();
    return true;
}

void Mesh::UpdateSkinningPose()
{
    const auto& pose = m_animator.GetNodeTransforms();
    for (auto& range : m_drawRanges)
    {
        BuildSkinPalette(range.bindings, pose, range.palette);
    }
}

void Mesh::PrintBoneCoords(std::vector<glm::vec3> endpoints)
{
	// Print once at load time so the sampled skeleton can be inspected without
	// flooding the console every frame from DrawSkeleton().
	const std::ios::fmtflags consoleFlags = std::cout.flags(); // round decimal values
	const std::streamsize consolePrecision = std::cout.precision();
	std::cout << std::fixed << std::setprecision(3);
	std::cout << "Skeleton pose: " << endpoints.size() / 2 << " line segment(s)\n";
	for (size_t lineIndex = 0; lineIndex + 1 < endpoints.size(); lineIndex += 2)
	{
		const glm::vec3& start = endpoints[lineIndex];
		const glm::vec3& end = endpoints[lineIndex + 1];
		std::cout << "  line " << lineIndex / 2
			<< ": (" << start.x << ", " << start.y << ", " << start.z << ")"
			<< " -> (" << end.x << ", " << end.y << ", " << end.z << ")\n";
	}
	std::cout.flags(consoleFlags);
	std::cout.precision(consolePrecision);
}

void Mesh::DrawSkeleton(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView)
{
	glm::mat4 modelTransform = GetModelTransform();
	const glm::vec3 skeletonColor(1.0f, 0.0f, 0.0f);

	// This is a debugging overlay: draw it over the model rather than letting surface depth hide it.
	const bool depthWasEnabled = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
	glDisable(GL_DEPTH_TEST);
	glLineWidth(3.0f);
	m_skeletonLines.Draw(shader, worldProj, worldView, modelTransform, skeletonColor);
	glLineWidth(1.0f);
	if (depthWasEnabled)
	{
		glEnable(GL_DEPTH_TEST);
	}

}

void Mesh::Draw(int programId)
{
    if (!m_vao)
    {
        return;
    }
    if (m_skinProgram != programId)
    {
        const GLuint block = glGetUniformBlockIndex(programId, "SkinPalette");
        if (block == GL_INVALID_INDEX)
        {
            throw std::runtime_error("Geometry shader is missing SkinPalette");
        }
        GLint blockSize = 0;
        glGetActiveUniformBlockiv(programId, block, GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);
        if (blockSize != MAX_SKIN_BONES * sizeof(glm::mat4))
        {
            throw std::runtime_error("SkinPalette shader/CPU layout mismatch");
        }
        glUniformBlockBinding(programId, block, 0);
        m_meshNodeLocation = glGetUniformLocation(programId, "MeshNodeTr");
        m_modelLocation = glGetUniformLocation(programId, "ModelTr");

        m_skinProgram = programId;
    }
    const glm::mat4 model = GetModelTransform();

    glUniformMatrix4fv(m_modelLocation, 1, GL_FALSE, &model[0][0]);
    glBindVertexArray(m_vao);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_skinBuffer);
    const auto& pose = m_animator.GetNodeTransforms();
    for (const auto& instance : m_animationData.meshInstances)
    {
        const auto& range = m_drawRanges.at(instance.meshIndex);
        m_materials.Bind(range.materialIndex, range.hasUVs, programId);
        const auto node = pose.find(instance.nodeName);
        if (node == pose.end())
        {
            throw std::runtime_error("Missing mesh node: " + instance.nodeName);
        }
        glUniformMatrix4fv(m_meshNodeLocation, 1, GL_FALSE, &node->second[0][0]);
        if (!range.palette.empty())
        {
            const glm::mat4* upload = range.palette.data();
            std::vector<glm::mat4> instancePalette;
            if (instance.nodeName != range.referenceNodeName)
            {
                // Bone poses are expressed in the original mesh instance's scene space.
                const auto delta = SkinInstanceTransform(pose.at(range.referenceNodeName), node->second);
                instancePalette.reserve(range.palette.size());
                for (const auto& matrix : range.palette)
                {
                    instancePalette.push_back(delta * matrix);
                }
                upload = instancePalette.data();
            }
            glBindBuffer(GL_UNIFORM_BUFFER, m_skinBuffer);
            glBufferSubData(GL_UNIFORM_BUFFER, 0,
                range.palette.size() * sizeof(glm::mat4), upload);
        }
        glDrawElementsBaseVertex(GL_TRIANGLES, range.indexCount, GL_UNSIGNED_INT,
            reinterpret_cast<const void*>(sizeof(unsigned int) * range.firstIndex),
            range.baseVertex);
    }
    glBindVertexArray(0);
    m_materials.Unbind();
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    CHECKERROR;
}
void Mesh::MakeVAO()
{
    GLint maxBlockSize = 0;
    glGetIntegerv(GL_MAX_UNIFORM_BLOCK_SIZE, &maxBlockSize);
    if (maxBlockSize < static_cast<GLint>(MAX_SKIN_BONES * sizeof(glm::mat4)))
    {
        throw std::runtime_error("GPU uniform block is too small for SkinPalette");
    }
    static_assert(sizeof(glm::mat4) == 64, "std140 mat4 must occupy 64 bytes");
    glGenBuffers(1, &m_skinBuffer);
    glBindBuffer(GL_UNIFORM_BUFFER, m_skinBuffer);
    glBufferData(GL_UNIFORM_BUFFER, MAX_SKIN_BONES * sizeof(glm::mat4), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	GLuint Pbuff;
	glGenBuffers(1, &Pbuff);
    m_buffers.push_back(Pbuff);
	glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * m_pnt.size(),
		m_pnt.data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (m_norm.size() > 0) {
		GLuint Nbuff;
		glGenBuffers(1, &Nbuff);
    m_buffers.push_back(Nbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Nbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_norm.size(),
			m_norm.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (m_tex.size() > 0) {
		GLuint Tbuff;
		glGenBuffers(1, &Tbuff);
    m_buffers.push_back(Tbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Tbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * m_tex.size(),
			m_tex.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (m_tan.size() > 0) {
		GLuint Dbuff;
		glGenBuffers(1, &Dbuff);
    m_buffers.push_back(Dbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Dbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_tan.size(),
			m_tan.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	GLuint Bbuff;
	glGenBuffers(1, &Bbuff);
    m_buffers.push_back(Bbuff);
	glBindBuffer(GL_ARRAY_BUFFER, Bbuff);
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoneData) * m_bones.size(),m_bones.data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(4);
	glVertexAttribIPointer(4, MAX_NUM_BONES_PER_VERTEX, GL_INT, sizeof(BoneData), reinterpret_cast<const void*>(offsetof(BoneData, ids)));
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, MAX_NUM_BONES_PER_VERTEX, GL_FLOAT,GL_FALSE, sizeof(BoneData),reinterpret_cast<const void*>(offsetof(BoneData, w)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	GLuint Ibuff;
	glGenBuffers(1, &Ibuff);
    m_buffers.push_back(Ibuff);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)* m_indices.size(),
		m_indices.data(), GL_STATIC_DRAW);

	glBindVertexArray(0);
}

void Mesh::InitMesh(const aiScene* scene)
{
	int num_vertices = 0;
	//load meshes
	for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
	{
		const aiMesh* m = scene->mMeshes[i];

		//individual mesh
		for (unsigned int j = 0; j < m->mNumVertices; ++j)
		{
			aiVector3D& p = m->mVertices[j];
			m_pnt.push_back(glm::vec4(p.x, p.y, p.z, 1.0));

			if (m->mNormals)
			{
				aiVector3D& n = m->mNormals[j];
				m_norm.push_back(glm::vec3(n.x, n.y, n.z));
			}
            else
            {
                m_norm.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
            }

            if (m->mTangents)
			{
				aiVector3D& tan = m->mTangents[j];
				m_tan.push_back(glm::vec3(tan.x, tan.y, tan.z));
			}
            else
            {
                m_tan.push_back(glm::vec3(0.0f));
            }

            const aiVector3D& tx = m->HasTextureCoords(0) ? m->mTextureCoords[0][j] : aiVector3D(0.0f, 0.0f, 0.0f);
			m_tex.push_back(glm::vec2(tx.x, tx.y));

			m_bones.push_back(BoneData());
		}

		InitBone(m, m_animationData.meshBones[i], num_vertices);
		DrawRange range;
        range.baseVertex = num_vertices;
        range.firstIndex = static_cast<unsigned int>(m_indices.size());
        range.indexCount = m->mNumFaces * 3;
        range.materialIndex = m->mMaterialIndex;
        range.hasUVs = m->HasTextureCoords(0);
        range.bindings = m_animationData.meshBones[i];
        for (const auto& instance : m_animationData.meshInstances)
        {
            if (instance.meshIndex == i)
            {
                range.referenceNodeName = instance.nodeName;
                break;
            }
        }
        if (range.referenceNodeName.empty())
        {
            throw std::runtime_error("Imported geometry has no mesh node reference");
        }
        m_drawRanges.push_back(std::move(range));

		for (unsigned int j = 0; j < m->mNumFaces; ++j)
		{
			const aiFace& tri = m->mFaces[j];
            if (tri.mNumIndices != 3)
            {
                throw std::runtime_error("Skinning requires triangle meshes");
            }
            for (unsigned int k = 0; k < 3; ++k)
            {
                if (tri.mIndices[k] >= m->mNumVertices)
                {
                    throw std::runtime_error("Invalid triangle vertex index");
                }
            }

            m_indices.push_back(tri.mIndices[0]);
			m_indices.push_back(tri.mIndices[1]);
			m_indices.push_back(tri.mIndices[2]);
		}

		num_vertices += m->mNumVertices;
	}
}

void Mesh::InitBone(const aiMesh* mesh, const std::vector<SkeletonBone>& bones, int index)
{
    if (bones.size() > MAX_SKIN_BONES)
    {
        throw std::runtime_error("Submesh exceeds the 128-bone skinning palette");
    }
    for (unsigned int i = 0; i < mesh->mNumBones; ++i)
    {
        const aiBone& bone = *mesh->mBones[i];
        for (unsigned int j = 0; j < bone.mNumWeights; ++j)
        {
            const aiVertexWeight& weight = bone.mWeights[j];
            if (weight.mVertexId >= mesh->mNumVertices)
            {
                throw std::runtime_error("Bone weight references an invalid vertex");
            }
            m_bones.at(index + weight.mVertexId).AddBoneData(i, weight.mWeight);
        }
    }
    for (unsigned int vertex = 0; vertex < mesh->mNumVertices; ++vertex)
    {
        m_bones.at(index + vertex).Normalize();
    }
}
