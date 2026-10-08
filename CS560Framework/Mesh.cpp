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

#include "Mesh.h"
#include <assimp/Importer.hpp>
#include "transform.h"
#include "shader.h"

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::LoadMesh(const std::string& path)
{
	m_animator.Reset();
	m_animationData = AnimationData();
	Assimp::Importer importer;
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	const aiScene* scene = importer.ReadFile(path.c_str(), aiProcess_Triangulate
											| aiProcess_GenSmoothNormals
											| aiProcess_FlipUVs
											| aiProcess_JoinIdenticalVertices);
	if (!scene)
	{
		printf("Error parsing '%s': '%s'\n", path.c_str(), importer.GetErrorString());
		return;
	}


	m_animationData = ImportAnimationData(scene);


	m_pnt.clear(); m_norm.clear(); m_tex.clear(); m_tan.clear();
	m_indices.clear(); m_bones.clear(); m_boneInfo.clear();
	m_name_index.clear(); m_drawRanges.clear();
	InitMesh(scene);
	importer.FreeScene();
	m_animator.Reset(&m_animationData);
	BuildFirstAnimationFrameSkeleton();

	MakeVAO();
}

glm::mat4 Mesh::GetModelTransform() const
{
	return Translate(0.f, 0.f, -2.5f) * Rotate(0, 90.f) * Scale(0.1f, 0.1f, 0.1f);
}

glm::vec3 Mesh::GetBindPosePosition(const std::string& boneName) const
{
	// An Assimp offset matrix converts a mesh-space vertex into the bone's
	// bind-pose local space.  Its inverse therefore gives the bone joint in the
	// exact coordinate system used by the vertices we render.  Scene-node
	// transforms are not reliable here: FBX files commonly add helper/pivot
	// nodes whose transforms are not applied to our flattened mesh vertices.
	const auto bone = m_name_index.find(boneName);
	if (bone == m_name_index.end() || bone->second >= static_cast<int>(m_boneInfo.size()))
	{
		// Might want to return an error here instead of 0,0,0
		// since 0,0,0 can be mistaken for the origin.
		return glm::vec3(0.0f);
	}

	const glm::mat4 boneToMesh = glm::inverse(m_boneInfo[bone->second].OffsetMatrix);
	return glm::vec3(boneToMesh[3]);
}

void Mesh::CollectBindPoseLines(const SkeletonNode* node,
	bool hasParentBone, const std::string& parentBoneName,
	std::vector<glm::vec3>& endpoints) const
{
	const std::string& nodeName = node->name;
	const bool isBone = m_name_index.find(nodeName) != m_name_index.end();

	if (isBone && hasParentBone)
	{
		endpoints.push_back(GetBindPosePosition(parentBoneName));
		endpoints.push_back(GetBindPosePosition(nodeName));
	}

	const bool childHasParentBone = isBone || hasParentBone;
	const std::string& childParentBoneName = isBone ? nodeName : parentBoneName;
	for (unsigned int i = 0; i < node->children.size(); ++i)
	{
		CollectBindPoseLines(
			&node->children[i],
			childHasParentBone,
			childParentBoneName,
			endpoints);
	}
}

void Mesh::BuildBindPoseSkeleton()
{
	std::vector<glm::vec3> endpoints;
	if (m_animationData.hasRoot)
	{
		CollectBindPoseLines(&m_animationData.root, false, std::string(), endpoints);
	}

	PrintBoneCoords(endpoints);

	m_skeletonLines.SetLines(endpoints);
}

void Mesh::CollectSkeletonLines(const SkeletonNode* node, bool hasParentBone,
    const glm::vec3& parentBonePosition, std::vector<glm::vec3>& endpoints) const
{
    const std::string name(node->name);
    const auto& transforms = m_animator.GetNodeTransforms();
    const auto found = transforms.find(name);

    if (found == transforms.end())
    {
        return;
    }

    const glm::mat4& transform = found->second;
    const glm::vec3 position(transform[3]);
    const bool isBone = m_name_index.find(name) != m_name_index.end();

    if (isBone && hasParentBone)
    {
        endpoints.push_back(parentBonePosition);
        endpoints.push_back(position);
    }

    for (unsigned int i = 0; i < node->children.size(); ++i)
    {
        CollectSkeletonLines(&node->children[i], isBone || hasParentBone,
            isBone ? position : parentBonePosition, endpoints);
    }
}

void Mesh::BuildFirstAnimationFrameSkeleton()
{
    if (!m_animationData.hasRoot || m_animator.GetAnimationCount() == 0)
    {
        BuildBindPoseSkeleton();
        return;
    }

    m_animator.EvaluatePose(m_animator.GetFirstKeyTimeTicks());
    std::vector<glm::vec3> endpoints;
    CollectSkeletonLines(&m_animationData.root, false, glm::vec3(0.0f), endpoints);
    PrintBoneCoords(endpoints);
    m_skeletonLines.SetLines(endpoints);
}

void Mesh::UpdateSkeletonPose()
{
    // Models without clips keep the bind-pose lines built at load time.
    if (m_animator.GetAnimationCount() == 0)
    {
        return;
    }

    std::vector<glm::vec3> endpoints;
    if (m_animationData.hasRoot)
    {
        // The animator includes helper-node transforms; only actual bones
        // become endpoints, connected to their nearest ancestor bone.
        CollectSkeletonLines(&m_animationData.root, false, glm::vec3(0.0f), endpoints);
    }
    m_skeletonLines.SetLines(endpoints);
}
bool Mesh::SetAnimation(unsigned int index)
{
    if (!m_animator.SetAnimation(index))
    {
        return false;
    }

    BuildFirstAnimationFrameSkeleton();
    return true;
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
	glm::vec3 diffuseColor{0.7,0.7,0.5};
	glm::vec3 specularColor{0.5,0.5,0.5};
	float shininess = 1.0f;

    const auto& pose = m_animator.GetNodeTransforms();
    for (const auto& bone : m_name_index)
    {
        const auto node = pose.find(bone.first);
        auto& info = m_boneInfo[bone.second];
        info.FinalTransformation = node != pose.end()
            ? node->second * info.OffsetMatrix : glm::mat4(1.0f);
    }
	CHECKERROR;
	glBindVertexArray(m_vao);
	CHECKERROR;


	for (int i = 0; i < m_boneInfo.size(); ++i)
	{
		std::string name = "gBones[" + std::to_string(i) + "]";
		int loc = glGetUniformLocation(programId, name.c_str());


		glUniformMatrix4fv(loc, 1, GL_FALSE, &m_boneInfo[i].FinalTransformation[0][0]);
		CHECKERROR;
	}

	int vertices = 0;
	int indices = 0;
	for (const auto& range : m_drawRanges)
	{
		int loc = glGetUniformLocation(programId, "diffuse");
		glUniform3fv(loc, 1, &diffuseColor[0]);

		loc = glGetUniformLocation(programId, "specular");
		glUniform3fv(loc, 1, &specularColor[0]);

		loc = glGetUniformLocation(programId, "shininess");
		glUniform1f(loc, shininess);
		CHECKERROR;

		glm::mat4 objectTr = GetModelTransform();
		loc = glGetUniformLocation(programId, "ModelTr");
		glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(objectTr));

		glm::mat4 inv = glm::inverse(objectTr);
		loc = glGetUniformLocation(programId, "NormalTr");
		glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(inv));

		const unsigned int curr_indices = range.indexCount;
		glDrawElementsBaseVertex(GL_TRIANGLES, curr_indices, GL_UNSIGNED_INT,
								(void*)(sizeof(unsigned int) * indices), vertices);

		vertices += range.vertexCount;
		indices += curr_indices;
	}
	CHECKERROR;
	glBindVertexArray(0);
}

void Mesh::MakeVAO()
{
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	GLuint Pbuff;
	glGenBuffers(1, &Pbuff);
	glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * m_pnt.size(),
		&m_pnt[0][0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (m_norm.size() > 0) {
		GLuint Nbuff;
		glGenBuffers(1, &Nbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Nbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_norm.size(),
			&m_norm[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (m_tex.size() > 0) {
		GLuint Tbuff;
		glGenBuffers(1, &Tbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Tbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * m_tex.size(),
			&m_tex[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (m_tan.size() > 0) {
		GLuint Dbuff;
		glGenBuffers(1, &Dbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Dbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_tan.size(),
			&m_tex[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	GLuint Bbuff;
	glGenBuffers(1, &Bbuff);
	glBindBuffer(GL_ARRAY_BUFFER, Bbuff);
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoneData) * m_bones.size(),&m_bones[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(4);
	glVertexAttribIPointer(4, MAX_NUM_BONES_PER_VERTEX, GL_INT, sizeof(BoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, MAX_NUM_BONES_PER_VERTEX, GL_FLOAT,GL_FALSE, sizeof(BoneData),(const GLvoid*)(MAX_NUM_BONES_PER_VERTEX *sizeof(int)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	GLuint Ibuff;
	glGenBuffers(1, &Ibuff);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)* m_indices.size(),
		&m_indices[0], GL_STATIC_DRAW);

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

			if (m->mTangents)
			{
				aiVector3D& tan = m->mTangents[j];
				m_tan.push_back(glm::vec3(tan.x, tan.y, tan.z));
			}

			const aiVector3D& tx = m->HasTextureCoords(0) ? m->mTextureCoords[0][j] : aiVector3D(0.0f, 0.0f, 0.0f);
			m_tex.push_back(glm::vec2(tx.x, tx.y));

			m_bones.push_back(BoneData());
		}

		InitBone(m, m_animationData.meshBones[i], num_vertices);
		m_drawRanges.push_back({m->mNumVertices, m->mNumFaces * 3});

		for (unsigned int j = 0; j < m->mNumFaces; ++j)
		{
			const aiFace& tri = m->mFaces[j];

			m_indices.push_back(tri.mIndices[0]);
			m_indices.push_back(tri.mIndices[1]);
			m_indices.push_back(tri.mIndices[2]);
		}

		num_vertices += m->mNumVertices;
	}
}

void Mesh::InitBone(const aiMesh* mesh, const std::vector<SkeletonBone>& bones, int index)
{
	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		const aiBone* b = mesh->mBones[i];
		int bone_id = GetBoneId(bones[i].name);
		if (bone_id == m_boneInfo.size()) {
			BoneInfo bi(bones[i].offsetMatrix);
			m_boneInfo.push_back(bi);
		}

		for (unsigned int j = 0; j < b->mNumWeights; ++j)
		{
			const aiVertexWeight& wt = b->mWeights[j];
			int vertexId = index + b->mWeights[j].mVertexId;
			m_bones[vertexId].AddBoneData(bone_id, wt.mWeight);
		}
	}
}

int Mesh::GetBoneId(const std::string& name)
{


	auto it = m_name_index.find(name);
	if(it == m_name_index.end())
	{
		m_name_index[name] = (int)m_name_index.size();
	}

	return m_name_index[name];
}
