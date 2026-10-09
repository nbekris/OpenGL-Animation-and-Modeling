#include "MeshAnimation.h"
#include <cmath>
#include <stdexcept>
#include <utility>

glm::vec3 MeshAnimation::GetBindPosePosition(const std::string& boneName) const
{
	// An Assimp offset matrix converts a mesh-space vertex into the bone's
	// bind-pose local space.  Its inverse therefore gives the bone joint in the
	// exact coordinate system used by the vertices we render.  Scene-node
	// transforms are not reliable here: FBX files commonly add helper/pivot
	// nodes whose transforms are not applied to our flattened mesh vertices.
	const auto bone = m_name_index.find(boneName);
	if (bone == m_name_index.end() || bone->second >= static_cast<int>(m_offsets.size()))
		// Might want to return an error here instead of 0,0,0
		// since 0,0,0 can be mistaken for the origin.
		return glm::vec3(0.0f);

	aiMatrix4x4 boneToMesh = m_offsets[bone->second];
	boneToMesh.Inverse();
	return glm::vec3(boneToMesh.a4, boneToMesh.b4, boneToMesh.c4);
}

void MeshAnimation::CollectBindPoseLines(const aiNode* node, const aiMatrix4x4& parentTransform,
	bool hasParentBone, const std::string& parentBoneName,
	std::vector<glm::vec3>& endpoints) const
{
	// This global transform variable and transform accumulation is not actually needed.
	// GetBindPosePosition uses offset matrix from Assimp to find line endpoints.
	// Leaving for now since we will need this most likely for animating.
	const aiMatrix4x4 globalTransform = parentTransform * node->mTransformation;
	const std::string nodeName(node->mName.C_Str());
	// Might want to take another look at this since I think this might be redundant logic
	// if there is another location where we figure bone names.
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

void MeshAnimation::BuildBindPoseSkeleton()
{
	std::vector<glm::vec3> endpoints;
	if (m_scene && m_scene->mRootNode)
	{
		const aiMatrix4x4 identity(
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0);
		CollectBindPoseLines(m_scene->mRootNode, identity, false, std::string(), endpoints);
	}

	

	m_endpoints = std::move(endpoints);
}

const aiNodeAnim* MeshAnimation::FindAnimationChannel(const aiAnimation* animation,
	const std::string& nodeName) const
{
	if (!animation)
		return nullptr;

	for (unsigned int i = 0; i < animation->mNumChannels; ++i)
	{
		const aiNodeAnim* channel = animation->mChannels[i];
		if (channel && nodeName == channel->mNodeName.C_Str())
			return channel;
	}
	return nullptr;
}

namespace
{
    Quaternion ToQuaternion(const aiQuaternion& q) { return Quaternion(q.w, q.x, q.y, q.z); }
    float UniformScale(const aiVector3D& s)
    {
        const float tolerance = 1e-4f * glm::max(1.0f, std::abs(s.x));
        if (std::abs(s.x - s.y) > tolerance || std::abs(s.x - s.z) > tolerance)
            throw std::runtime_error("VQS animation requires uniform scaling.");
        return s.x;
    }
    VQS ToVQS(const aiMatrix4x4& m)
    {
        aiVector3D scale, position;
        aiQuaternion rotation;
        m.Decompose(scale, rotation, position);
        return VQS(glm::vec3(position.x, position.y, position.z), ToQuaternion(rotation), UniformScale(scale));
    }

	aiVector3D SampleVectorKey(const aiVectorKey* keys, unsigned int keyCount,
		double time, const aiVector3D& fallback)
	{
		if (!keys || keyCount == 0)
			return fallback;
		if (keyCount == 1 || time <= keys[0].mTime)
			return keys[0].mValue;
		if (time >= keys[keyCount - 1].mTime)
			return keys[keyCount - 1].mValue;

		unsigned int next = 1;
		while (next < keyCount && time > keys[next].mTime)
			++next;
		const unsigned int previous = next - 1;
		const double span = keys[next].mTime - keys[previous].mTime;
		const float factor = span > 0.0
			? static_cast<float>((time - keys[previous].mTime) / span)
			: 0.0f;
        const auto& a = keys[previous].mValue;
        const auto& b = keys[next].mValue;
        const VQS pose = VQS::Interpolate(
            VQS(glm::vec3(a.x, a.y, a.z), Quaternion(1, 0, 0, 0), 1),
            VQS(glm::vec3(b.x, b.y, b.z), Quaternion(1, 0, 0, 0), 1), factor);
        return aiVector3D(pose._translation.x, pose._translation.y, pose._translation.z);
	}

	Quaternion SampleRotationKey(const aiQuatKey* keys, unsigned int keyCount,
		double time, const aiQuaternion& fallback)
	{
		if (!keys || keyCount == 0)
			return ToQuaternion(fallback);
		if (keyCount == 1 || time <= keys[0].mTime)
			return ToQuaternion(keys[0].mValue);
		if (time >= keys[keyCount - 1].mTime)
			return ToQuaternion(keys[keyCount - 1].mValue);

		unsigned int next = 1;
		while (next < keyCount && time > keys[next].mTime)
			++next;
		const unsigned int previous = next - 1;
		const double span = keys[next].mTime - keys[previous].mTime;
		const float factor = span > 0.0
			? static_cast<float>((time - keys[previous].mTime) / span)
			: 0.0f;
        return Quaternion::Slerp(ToQuaternion(keys[previous].mValue), ToQuaternion(keys[next].mValue), factor);
	}
}

void MeshAnimation::Initialize(const aiScene* scene,
    const std::map<std::string, int>& boneIndices,
    const std::vector<aiMatrix4x4>& offsets, int animationIndex)
{
    m_scene = scene;
    m_name_index = boneIndices;
    m_offsets = offsets;
    m_endpoints.clear();
    // Initialize even when the initial selection falls back to the bind pose.
    if (m_scene && m_scene->mRootNode && m_scene->HasAnimations())
        m_inverseRootVQS = ToVQS(m_scene->mRootNode->mTransformation).Inverse();
    BuildFirstAnimationFrameSkeleton(animationIndex);
}

VQS MeshAnimation::SampleLocalTransform(const aiNode* node,
	const aiNodeAnim* channel, double animationTime) const
{
	if (!channel)
		return ToVQS(node->mTransformation);

	aiVector3D defaultScale;
	aiQuaternion defaultRotation;
	aiVector3D defaultPosition;
	node->mTransformation.Decompose(defaultScale, defaultRotation, defaultPosition);

	const aiVector3D scale = SampleVectorKey(channel->mScalingKeys,
		channel->mNumScalingKeys, animationTime, defaultScale);
	const Quaternion rotation = SampleRotationKey(channel->mRotationKeys,
		channel->mNumRotationKeys, animationTime, defaultRotation);
	const aiVector3D position = SampleVectorKey(channel->mPositionKeys,
		channel->mNumPositionKeys, animationTime, defaultPosition);
	return VQS(glm::vec3(position.x, position.y, position.z), rotation, UniformScale(scale));
}

void MeshAnimation::CollectAnimatedPoseLines(const aiNode* node, const aiAnimation* animation,
	double animationTime, const VQS& parentTransform,
	bool hasParentBone, const glm::vec3& parentBonePosition,
	std::vector<glm::vec3>& endpoints) const
{
	const std::string nodeName(node->mName.C_Str());
	const aiNodeAnim* channel = FindAnimationChannel(animation, nodeName);
	const VQS localTransform = SampleLocalTransform(node, channel, animationTime);
	const VQS globalTransform = parentTransform * localTransform;
	const VQS meshTransform = m_inverseRootVQS * globalTransform;
    const glm::vec3 nodePosition = meshTransform._translation;
	const bool isBone = m_name_index.find(nodeName) != m_name_index.end();

	if (isBone && hasParentBone)
	{
		endpoints.push_back(parentBonePosition);
		endpoints.push_back(nodePosition);
	}

	const bool childHasParentBone = isBone || hasParentBone;
	const glm::vec3 childParentPosition = isBone ? nodePosition : parentBonePosition;
	for (unsigned int i = 0; i < node->mNumChildren; ++i)
	{
		CollectAnimatedPoseLines(node->mChildren[i], animation, animationTime,
			globalTransform, childHasParentBone, childParentPosition, endpoints);
	}
}

void MeshAnimation::BuildFirstAnimationFrameSkeleton(int anim_num)
{
	if (!m_scene || !m_scene->mRootNode || !m_scene->HasAnimations())
	{
		BuildBindPoseSkeleton();
		return;
	}

	const unsigned int animationIndex = static_cast<unsigned int>(anim_num);
	if (animationIndex >= m_scene->mNumAnimations)
	{
		BuildBindPoseSkeleton();
		return;
	}

	const aiAnimation* animation = m_scene->mAnimations[animationIndex];
	m_inverseRootVQS = ToVQS(m_scene->mRootNode->mTransformation).Inverse();
	double firstKeyTime = 0.0;
	bool foundKey = false;
	for (unsigned int i = 0; i < animation->mNumChannels; ++i)
	{
		const aiNodeAnim* channel = animation->mChannels[i];
		if (!channel)
			continue;
		const double times[] = {
			channel->mNumPositionKeys ? channel->mPositionKeys[0].mTime : 0.0,
			channel->mNumRotationKeys ? channel->mRotationKeys[0].mTime : 0.0,
			channel->mNumScalingKeys ? channel->mScalingKeys[0].mTime : 0.0
		};
		const bool present[] = {
			channel->mNumPositionKeys != 0,
			channel->mNumRotationKeys != 0,
			channel->mNumScalingKeys != 0
		};
		for (int keyType = 0; keyType < 3; ++keyType)
		{
			if (present[keyType] && (!foundKey || times[keyType] < firstKeyTime))
			{
				firstKeyTime = times[keyType];
				foundKey = true;
			}
		}
	}

	std::vector<glm::vec3> endpoints;
	CollectAnimatedPoseLines(m_scene->mRootNode, animation, firstKeyTime,
		VQS(), false, glm::vec3(0.0f), endpoints);
	
	m_endpoints = std::move(endpoints);
}

unsigned int MeshAnimation::GetAnimationCount() const
{
    return m_scene ? m_scene->mNumAnimations : 0;
}

std::string MeshAnimation::GetAnimationName(unsigned int index) const
{
    if (index >= GetAnimationCount())
        return "No animation";
    const std::string name = m_scene->mAnimations[index]->mName.C_Str();
    return name.empty() ? "Animation " + std::to_string(index + 1) : name;
}

void MeshAnimation::UpdateAnimation(int anim_num, double elapsedSeconds, bool loop)
{
    if (!m_scene || !m_scene->mRootNode || !m_scene->HasAnimations()
        || anim_num < 0 || static_cast<unsigned int>(anim_num) >= m_scene->mNumAnimations)
        return;
    const aiAnimation* animation = m_scene->mAnimations[anim_num];
    const double ticksPerSecond = animation->mTicksPerSecond > 0.0 ? animation->mTicksPerSecond : 25.0;
    const double ticks = glm::max(0.0, elapsedSeconds) * ticksPerSecond;
    const double time = animation->mDuration > 0.0
        ? (loop ? std::fmod(ticks, animation->mDuration) : glm::min(ticks, animation->mDuration)) : 0.0;
    std::vector<glm::vec3> endpoints;
    CollectAnimatedPoseLines(m_scene->mRootNode, animation, time,
        VQS(), false, glm::vec3(0.0f), endpoints);
    m_endpoints = std::move(endpoints);
}

double MeshAnimation::GetAnimationDurationSeconds(unsigned int index) const
{
    if (index >= GetAnimationCount())
        return 0.0;
    const aiAnimation* animation = m_scene->mAnimations[index];
    const double rate = animation->mTicksPerSecond > 0.0 ? animation->mTicksPerSecond : 25.0;
    return animation->mDuration / rate;
}

