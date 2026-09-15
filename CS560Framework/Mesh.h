#pragma once

#include <map>
#include <vector>
#include "LineRenderer.h"
#include "texture.h"
#include "shapes.h"

class ShaderProgram;


#define MAX_NUM_BONES_PER_VERTEX 4

//Bone Ids and weights (up to 4)
struct BoneData
{
    int ids[MAX_NUM_BONES_PER_VERTEX] = { 0 };
    float w[MAX_NUM_BONES_PER_VERTEX] = { 0.0f };

    BoneData()
    {
    }

    void AddBoneData(int _id, float _w)
    {
        int arr_size = sizeof(ids) / sizeof(ids[0]);
        for (int i = 0; i < arr_size; i++) {
            if (w[i] == 0.0) {
                ids[i] = _id;
                w[i] = _w;
                return;
            }
        }

        assert(0);
    }
};

//Bone information for transform
struct BoneInfo
{
    aiMatrix4x4 OffsetMatrix;
    aiMatrix4x4 FinalTransformation;

    BoneInfo(const aiMatrix4x4& Offset)
    {
        OffsetMatrix = Offset;
    }
};


class Mesh
{
public:
	Mesh();
	~Mesh();

    void LoadMesh(const std::string& path);
	void Draw(int programId, long double dt);
    void DrawBindPoseSkeleton(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView);
    void MakeVAO();

    int anim_num = 0;
private:

#define MAX_NUM_BONES_PER_VERTEX 4

    void InitMesh(const std::string& path);
    void InitBone(const aiMesh* mesh, int index);
    int GetBoneId(const aiBone* b);
    void BuildBindPoseSkeleton();
    void CollectBindPoseLines(const aiNode* node, const aiMatrix4x4& parentTransform,
        bool hasParentBone, const aiMatrix4x4& parentBoneTransform,
        std::vector<glm::vec3>& endpoints) const;
    glm::vec3 GetBindPosePosition(const aiMatrix4x4& globalTransform) const;
    glm::mat4 GetModelTransform() const;

    GLuint m_vao = 0;

	Assimp::Importer Importer;
	const aiScene* m_scene = NULL;

	std::vector<glm::vec4> m_pnt;
	std::vector<glm::vec3> m_norm;
	std::vector<glm::vec2> m_tex;
	std::vector<glm::vec3> m_tan;
	std::vector<int> m_indices;
    std::vector<BoneData> m_bones;
    std::vector<BoneInfo> m_boneInfo;
    LineRenderer m_bindPoseLines;

    aiMatrix4x4 m_inverseTrans;
    std::map<std::string, int> m_name_index;

};
