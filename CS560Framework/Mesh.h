#pragma once

#include <assimp/scene.h>
#include <map>
#include <vector>
#include "LineRenderer.h"
#include "Animator.h"
#include "AnimationData.h"
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
        for (int i = 0; i < arr_size; i++)
        {
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
    glm::mat4 OffsetMatrix{1.0f};
    glm::mat4 FinalTransformation{1.0f};

    BoneInfo(const glm::mat4& Offset)
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
	void Draw(int programId);
    void DrawSkeleton(ShaderProgram& shader, glm::mat4& worldProj, glm::mat4& worldView);
    void MakeVAO();

    Animator& GetAnimator() { return m_animator; }
    const AnimationData& GetAnimationData() const { return m_animationData; }
    bool SetAnimation(unsigned int index);
    // Refresh line endpoints from the pose already evaluated by the frame update.
    void UpdateSkeletonPose();
private:

#define MAX_NUM_BONES_PER_VERTEX 4

    void InitMesh(const aiScene* scene);
    void InitBone(const aiMesh* mesh, const std::vector<SkeletonBone>& bones, int index);
    int GetBoneId(const std::string& name);
    void BuildBindPoseSkeleton();
    void BuildFirstAnimationFrameSkeleton();
    void PrintBoneCoords(std::vector<glm::vec3> endpoints);
    void CollectBindPoseLines(const SkeletonNode* node,
        bool hasParentBone, const std::string& parentBoneName,
        std::vector<glm::vec3>& endpoints) const;
    glm::vec3 GetBindPosePosition(const std::string& boneName) const;
    void CollectSkeletonLines(const SkeletonNode* node, bool hasParentBone,
        const glm::vec3& parentBonePosition, std::vector<glm::vec3>& endpoints) const;
    glm::mat4 GetModelTransform() const;

    GLuint m_vao = 0;

    struct DrawRange
    {
        unsigned int vertexCount;
        unsigned int indexCount;
    };
    std::vector<DrawRange> m_drawRanges;

	std::vector<glm::vec4> m_pnt;
	std::vector<glm::vec3> m_norm;
	std::vector<glm::vec2> m_tex;
	std::vector<glm::vec3> m_tan;
	std::vector<int> m_indices;
    std::vector<BoneData> m_bones;
    std::vector<BoneInfo> m_boneInfo;
    Animator m_animator;
    AnimationData m_animationData;
    LineRenderer m_skeletonLines;

    std::map<std::string, int> m_name_index;

};
