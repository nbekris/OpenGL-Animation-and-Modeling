#pragma once

#include <assimp/scene.h>
#include <map>
#include <vector>
#include "LineRenderer.h"
#include "Animator.h"
#include "AnimationData.h"
#include "Skinning.h"
#include "ModelMaterials.h"
#include "shapes.h"

class ShaderProgram;


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
    void UpdateSkinningPose();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
private:



    void InitMesh(const aiScene* scene);
    void InitBone(const aiMesh* mesh, const std::vector<SkeletonBone>& bones, int index);
    void ReleaseGpuResources();
    void BuildFirstAnimationFrameSkeleton();
    void PrintBoneCoords(std::vector<glm::vec3> endpoints);
    glm::mat4 GetModelTransform() const;

    GLuint m_vao = 0;
    std::vector<GLuint> m_buffers;
    GLuint m_skinBuffer = 0;
    int m_skinProgram = -1;
    GLint m_meshNodeLocation = -1;
    GLint m_modelLocation = -1;
    ModelMaterials m_materials;

    struct DrawRange
    {
        unsigned int baseVertex;
        unsigned int firstIndex;
        unsigned int indexCount;
        unsigned int materialIndex;
        bool hasUVs;
        std::string referenceNodeName;
        std::vector<SkeletonBone> bindings;
        std::vector<glm::mat4> palette;
    };
    std::vector<DrawRange> m_drawRanges;

	std::vector<glm::vec4> m_pnt;
	std::vector<glm::vec3> m_norm;
	std::vector<glm::vec2> m_tex;
	std::vector<glm::vec3> m_tan;
	std::vector<int> m_indices;
    std::vector<BoneData> m_bones;

    Animator m_animator;
    AnimationData m_animationData;
    LineRenderer m_skeletonLines;


};
