#include "Animator.h"
#include <cmath>

void Animator::Reset(const aiScene* scene)
{
    m_scene = scene;
    m_animationIndex = 0;
    m_playbackSeconds = 0.0;
    m_playing = false;
    m_nodeTransforms.clear();
    m_inverseRoot = aiMatrix4x4();
    if (scene && scene->mRootNode)
        m_inverseRoot = aiMatrix4x4(scene->mRootNode->mTransformation).Inverse();
}

const aiAnimation* Animator::GetAnimation() const
{
    return m_scene && m_animationIndex < m_scene->mNumAnimations
        ? m_scene->mAnimations[m_animationIndex] : nullptr;
}

unsigned int Animator::GetAnimationCount() const
{
    return m_scene ? m_scene->mNumAnimations : 0;
}

std::string Animator::GetAnimationName(unsigned int index) const
{
    if (index >= GetAnimationCount()) return {};
    const std::string name = m_scene->mAnimations[index]->mName.C_Str();
    return name.empty() ? "Animation " + std::to_string(index) : name;
}

bool Animator::SetAnimation(unsigned int index)
{
    if (index >= GetAnimationCount()) return false;
    if (index != m_animationIndex)
    {
        m_animationIndex = index;
        m_playbackSeconds = 0.0;
        m_nodeTransforms.clear();
    }
    return true;
}

void Animator::Update(double deltaSeconds)
{
    if (!m_playing || !GetAnimation() || !std::isfinite(deltaSeconds) || deltaSeconds <= 0.0)
        return;
    const aiAnimation* animation = GetAnimation();
    // Our fallback for files with an unspecified tick rate is 25 ticks/second.
    const double ticksPerSecond = animation->mTicksPerSecond > 0.0 ? animation->mTicksPerSecond : 25.0;
    const double durationSeconds = animation->mDuration / ticksPerSecond;
    m_playbackSeconds = durationSeconds > 0.0
        ? std::fmod(m_playbackSeconds + std::fmod(deltaSeconds, durationSeconds), durationSeconds)
        : 0.0;
}

double Animator::GetSampleTimeTicks() const
{
    const aiAnimation* animation = GetAnimation();
    if (!animation || animation->mDuration <= 0.0) return 0.0;
    const double ticksPerSecond = animation->mTicksPerSecond > 0.0 ? animation->mTicksPerSecond : 25.0;
    return std::fmod(m_playbackSeconds * ticksPerSecond, animation->mDuration);
}

void Animator::EvaluatePose(double timeTicks)
{
    m_nodeTransforms.clear();
    if (m_scene && m_scene->mRootNode)
        EvaluateNode(m_scene->mRootNode, aiMatrix4x4(), timeTicks);
}

void Animator::EvaluateNode(const aiNode* node, const aiMatrix4x4& parentTransform, double timeTicks)
{
    const std::string name(node->mName.C_Str());
    const aiMatrix4x4 global = parentTransform * SampleLocalTransform(node,
        FindAnimationChannel(GetAnimation(), name), timeTicks);
    m_nodeTransforms[name] = m_inverseRoot * global;
    for (unsigned int i = 0; i < node->mNumChildren; ++i)
        EvaluateNode(node->mChildren[i], global, timeTicks);
}

const aiNodeAnim* Animator::FindAnimationChannel(const aiAnimation* animation,
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
		return keys[previous].mValue
			+ factor * (keys[next].mValue - keys[previous].mValue);
	}

	aiQuaternion SampleRotationKey(const aiQuatKey* keys, unsigned int keyCount,
		double time, const aiQuaternion& fallback)
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
		aiQuaternion result;
		aiQuaternion::Interpolate(result, keys[previous].mValue, keys[next].mValue, factor);
		return result.Normalize();
	}
}

aiMatrix4x4 Animator::SampleLocalTransform(const aiNode* node,
	const aiNodeAnim* channel, double animationTime) const
{
	if (!channel)
		return node->mTransformation;

	aiVector3D defaultScale;
	aiQuaternion defaultRotation;
	aiVector3D defaultPosition;
	node->mTransformation.Decompose(defaultScale, defaultRotation, defaultPosition);

	const aiVector3D scale = SampleVectorKey(channel->mScalingKeys,
		channel->mNumScalingKeys, animationTime, defaultScale);
	const aiQuaternion rotation = SampleRotationKey(channel->mRotationKeys,
		channel->mNumRotationKeys, animationTime, defaultRotation);
	const aiVector3D position = SampleVectorKey(channel->mPositionKeys,
		channel->mNumPositionKeys, animationTime, defaultPosition);
	return aiMatrix4x4(scale, rotation, position);
}


double Animator::GetFirstKeyTimeTicks() const
{
    const aiAnimation* animation = GetAnimation();
    if (!animation) return 0.0;
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

    return firstKeyTime;
}
