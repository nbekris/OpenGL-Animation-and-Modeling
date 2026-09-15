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
#include "transform.h"
#include "shader.h"

glm::mat4 convert(const aiMatrix4x4& m)
{
	glm::mat4 res;

	res[0][0] = m.a1; res[0][1] = m.b1; res[0][2] = m.c1; res[0][3] = m.d1;
	res[1][0] = m.a2; res[1][1] = m.b2; res[1][2] = m.c2; res[1][3] = m.d2;
	res[2][0] = m.a3; res[2][1] = m.b3; res[2][2] = m.c3; res[2][3] = m.d3;
	res[3][0] = m.a4; res[3][1] = m.b4; res[3][2] = m.c4; res[3][3] = m.d4;
	return res;
}

glm::mat4 convert(const aiMatrix3x3& m)
{
	glm::mat4 res;
	res[0][0] = m.a1; res[0][1] = m.a2; res[0][2] = m.a3; res[0][3] = 0.0;
	res[1][0] = m.b1; res[1][1] = m.b2; res[1][2] = m.b3; res[1][3] = 0.0;
	res[2][0] = m.c1; res[2][1] = m.c2; res[2][2] = m.c3; res[2][3] = 0.0;
	res[3][0] = 0.0; res[3][1] =0.0; res[3][2] = 0.0; res[3][3] = 1.0;
	return res;
}



Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::LoadMesh(const std::string& path)
{
	Importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	m_scene = Importer.ReadFile(path.c_str(), aiProcess_Triangulate 
											| aiProcess_GenSmoothNormals 
											| aiProcess_FlipUVs 
											| aiProcess_JoinIdenticalVertices);
	if (!m_scene)
	{
		printf("Error parsing '%s': '%s'\n", path.c_str(), Importer.GetErrorString());
		return;
	}

	
	m_inverseTrans = m_scene->mRootNode->mTransformation;
	m_inverseTrans = m_inverseTrans.Inverse();

	InitMesh(path);
	BuildBindPoseSkeleton();

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
		return glm::vec3(0.0f);

	aiMatrix4x4 boneToMesh = m_boneInfo[bone->second].OffsetMatrix;
	boneToMesh.Inverse();
	return glm::vec3(boneToMesh.a4, boneToMesh.b4, boneToMesh.c4);
}

void Mesh::CollectBindPoseLines(const aiNode* node, const aiMatrix4x4& parentTransform,
	bool hasParentBone, const std::string& parentBoneName,
	std::vector<glm::vec3>& endpoints) const
{
	const aiMatrix4x4 globalTransform = parentTransform * node->mTransformation;
	const std::string nodeName(node->mName.C_Str());
	const bool isBone = m_name_index.find(nodeName) != m_name_index.end();

	if (isBone && hasParentBone)
	{
		endpoints.push_back(GetBindPosePosition(parentBoneName));
		endpoints.push_back(GetBindPosePosition(nodeName));
	}

	const bool childHasParentBone = isBone || hasParentBone;
	const std::string& childParentBoneName = isBone ? nodeName : parentBoneName;
	for (unsigned int i = 0; i < node->mNumChildren; ++i)
	{
		CollectBindPoseLines(
			node->mChildren[i], 
			globalTransform, 
			childHasParentBone,
			childParentBoneName,
			endpoints);
	}
}

void Mesh::BuildBindPoseSkeleton()
{
	std::vector<glm::vec3> endpoints;
	if (m_scene && m_scene->mRootNode)
	{
		const aiMatrix4x4 identity(1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0);
		CollectBindPoseLines(m_scene->mRootNode, identity, false, std::string(), endpoints);
	}

	// Print once at load time so the bind-pose skeleton can be inspected without
	// flooding the console every frame from DrawBindPoseSkeleton().
	const std::ios::fmtflags consoleFlags = std::cout.flags(); // round decimal values
	const std::streamsize consolePrecision = std::cout.precision();
	std::cout << std::fixed << std::setprecision(3);
	std::cout << "Bind-pose skeleton: " << endpoints.size() / 2 << " line segment(s)\n";
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

	m_bindPoseLines.SetLines(endpoints);
}

void Mesh::DrawBindPoseSkeleton(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView)
{
	glm::mat4 modelTransform = GetModelTransform();
	const glm::vec3 skeletonColor(1.0f, 0.0f, 0.0f);

	// This is a debugging overlay: draw it over the model rather than letting surface depth hide it.
	const bool depthWasEnabled = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
	glDisable(GL_DEPTH_TEST);
	glLineWidth(3.0f);
	m_bindPoseLines.Draw(shader, worldProj, worldView, modelTransform, skeletonColor);
	glLineWidth(1.0f);
	if (depthWasEnabled)
		glEnable(GL_DEPTH_TEST);
}

void Mesh::Draw(int programId, long double dt)
{
	glm::vec3 diffuseColor{0.7,0.7,0.5};
	glm::vec3 specularColor{0.5,0.5,0.5};
	float shininess = 1.0f;

	aiMatrix4x4 Identity(1.0, 0.0, 0.0, 0.0,
						 0.0, 1.0, 0.0, 0.0,
						 0.0, 0.0, 1.0, 0.0,
						 0.0, 0.0, 0.0, 1.0);

	CHECKERROR;
	glBindVertexArray(m_vao);
	CHECKERROR;


	for (int i = 0; i < m_boneInfo.size(); ++i)
	{
		std::string name = "gBones[" + std::to_string(i) + "]";
		int loc = glGetUniformLocation(programId, name.c_str());
		
		
		glUniformMatrix4fv(loc, 1, GL_TRUE, &m_boneInfo[i].FinalTransformation[0][0]);
		CHECKERROR;
	}

	int vertices = 0;
	int indices = 0;
	for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		int matIndex = m_scene->mMeshes[i]->mMaterialIndex;

		//m_diffuse[matIndex]->BindTexture(0, programId, "diffuseTex");

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

		int curr_indices = m_scene->mMeshes[i]->mNumFaces * 3;
		glDrawElementsBaseVertex(GL_TRIANGLES, curr_indices, GL_UNSIGNED_INT,
								(void*)(sizeof(unsigned int) * indices), vertices);

		vertices += m_scene->mMeshes[i]->mNumVertices;
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

void Mesh::InitMesh(const std::string& path)
{
	int num_vertices = 0;
	//load meshes
	for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		const aiMesh* m = m_scene->mMeshes[i];
		
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

		InitBone(m, num_vertices);

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

void Mesh::InitBone(const aiMesh* mesh, int index)
{
	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		const aiBone* b = mesh->mBones[i];
		int bone_id = GetBoneId(b);
		if (bone_id == m_boneInfo.size()) {
			BoneInfo bi(b->mOffsetMatrix);
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

int Mesh::GetBoneId(const aiBone* b)
{
	std::string name(b->mName.C_Str());

	auto it = m_name_index.find(name);
	if(it == m_name_index.end())
		m_name_index[name] = (int)m_name_index.size();

	return m_name_index[name];
}
