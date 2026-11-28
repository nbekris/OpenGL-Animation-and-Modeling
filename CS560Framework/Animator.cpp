#define GLM_FORCE_RADIANS
#include "Animator.h"
#include "VQS.h"
#include "TimingSettings.h"
#include <cmath>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

void Animator::Reset(const AnimationData* data)
{
    m_data = data;
    m_animationIndex = 0;
    m_playbackSeconds = 0.0;
    m_playbackSpeed = 1.0;
    m_interpolationMode = InterpolationMode::Incremental;
    m_hasPlaybackRange = false;
    m_playing = false;
    m_blendWeight = 0.0f;
    m_blendTargetTicks = 0.0;
    m_nodeTransforms.clear();
    m_skeletonEndpoints.clear();
    m_boneNames.clear();
    m_poseNodes.clear();
    CacheAnimationChannels();
    if (m_data)
    {
        for (const auto& bones : m_data->meshBones)
        {
            for (const auto& bone : bones)
            {
                m_boneNames.insert(bone.name);
            }
        }
    }
    if (m_data && m_data->hasRoot)
        IndexPoseNodes(&m_data->root, -1, -1);
}

const AnimationClip* Animator::GetAnimation() const
{
    return m_data && m_animationIndex < m_data->clips.size()
        ? &m_data->clips[m_animationIndex] : nullptr;
}

unsigned int Animator::GetAnimationCount() const
{
    return m_data ? static_cast<unsigned int>(m_data->clips.size()) : 0;
}

std::string Animator::GetAnimationName(unsigned int index) const
{
    if (index >= GetAnimationCount())
    {
        return {};
    }

    const std::string name = m_data->clips[index].name;
    return name.empty() ? "Animation " + std::to_string(index) : name;
}

bool Animator::SetAnimation(unsigned int index)
{
    if (index >= GetAnimationCount())
    {
        return false;
    }

    if (index != m_animationIndex)
    {
        m_animationIndex = index;
        CacheAnimationChannels();
        m_playbackSeconds = 0.0;
        m_hasPlaybackRange = false;
        m_nodeTransforms.clear();
        m_skeletonEndpoints.clear();
    }

    return true;
}

double Animator::GetTicksPerSecond() const
{
    const AnimationClip* clip = GetAnimation();
    return clip && clip->ticksPerSecond > 0.0 ? clip->ticksPerSecond : 25.0;
}

AnimationRange Animator::GetPlaybackRange() const
{
    if (m_hasPlaybackRange)
    {
        return m_playbackRange;
    }

    const AnimationClip* clip = GetAnimation();
    return { 0.0, clip ? clip->durationTicks : 0.0 };
}

bool Animator::SetPlaybackRange(AnimationRange range)
{
    const AnimationClip* clip = GetAnimation();
    if (!clip || !std::isfinite(range.startTicks) || !std::isfinite(range.endTicks) ||
        range.startTicks < 0.0 || range.endTicks <= range.startTicks ||
        range.endTicks > clip->durationTicks)
    {
        return false;
    }

    m_playbackRange = range;
    m_hasPlaybackRange = true;
    m_playbackSeconds = 0.0;
    return true;
}

bool Animator::SetPlaybackSpeed(double multiplier)
{
    if (!std::isfinite(multiplier) || multiplier <= 0.0)
    {
        return false;
    }

    m_playbackSpeed = multiplier;
    return true;
}

void Animator::Update(double deltaSeconds)
{
    if (!m_playing || !GetAnimation() || !std::isfinite(deltaSeconds) || deltaSeconds <= 0.0)
    {
        return;
    }

    const double scaledDelta = deltaSeconds * m_playbackSpeed;
    if (!std::isfinite(scaledDelta))
    {
        return;
    }

    const AnimationRange range = GetPlaybackRange();
    const double durationSeconds = (range.endTicks - range.startTicks) / GetTicksPerSecond();
    m_playbackSeconds = durationSeconds > 0.0
        ? std::fmod(m_playbackSeconds + std::fmod(scaledDelta, durationSeconds), durationSeconds)
        : 0.0;
}

// Set the time within one animation cycle using distance traveled along the path
void Animator::SetCyclePhase(double phase)
{
    if (!GetAnimation() || !std::isfinite(phase))
        return;

    const AnimationRange range = GetPlaybackRange();
    const double duration = (range.endTicks - range.startTicks) / GetTicksPerSecond();
    if (duration <= 0.0)
        return;

    // wrap phase to [0,1), then convert it to animation seconds
    phase -= std::floor(phase);
    m_playbackSeconds = phase * duration;
}

double Animator::GetSampleTimeTicks() const
{
    const AnimationRange range = GetPlaybackRange();
    const double durationTicks = range.endTicks - range.startTicks;
    return durationTicks > 0.0
        ? range.startTicks + std::fmod(m_playbackSeconds * GetTicksPerSecond(), durationTicks)
        : 0.0;
}

// Configures the standing-pose overlay without changing the gait playback clock.
void Animator::SetPoseBlend(double targetTicks, float weight)
{
    if (!std::isfinite(targetTicks) || !std::isfinite(weight))
        return;
    const auto* clip = GetAnimation();
    m_blendTargetTicks = (std::max)(0.0, (std::min)(clip ? clip->durationTicks : 0.0, targetTicks));
    m_blendWeight = (std::max)(0.0f, (std::min)(1.0f, weight));
}

// Samples animation once, then publishes the composed hierarchy.
void Animator::EvaluatePose(double timeTicks)
{
    for (auto& node : m_poseNodes)
        node.local = SampleLocalTransform(node.source,
            FindAnimationChannel(node.source->name), timeTicks);
    RebuildGlobalPose();
}

// Retains every imported node and records its nearest skeletal ancestor.
void Animator::IndexPoseNodes(const SkeletonNode* node, int parent, int parentBone)
{
    const int index = static_cast<int>(m_poseNodes.size());
    const bool bone = m_boneNames.count(node->name) != 0;
    m_poseNodes.push_back({node, parent, parentBone, bone});
    for (const auto& child : node->children)
        IndexPoseNodes(&child, index, bone ? index : parentBone);
}

// Recomputes transforms and skeleton lines from the current editable local pose.
void Animator::RebuildGlobalPose()
{
    m_nodeTransforms.clear();
    m_skeletonEndpoints.clear();
    for (auto& node : m_poseNodes)
    {
        node.global = (node.parent < 0 ? glm::mat4(1) : m_poseNodes[node.parent].global) * node.local;
        const glm::mat4 transform = m_data->inverseRootMatrix * node.global;
        m_nodeTransforms[node.source->name] = transform;
        if (node.bone && node.parentBone >= 0)
        {
            m_skeletonEndpoints.push_back(glm::vec3(
                m_data->inverseRootMatrix * m_poseNodes[node.parentBone].global[3]));
            m_skeletonEndpoints.push_back(glm::vec3(transform[3]));
        }
    }
}

void Animator::CacheAnimationChannels()
{
    m_animationChannels.clear();
    m_rotationSamplers.clear();
    const AnimationClip* animation = GetAnimation();
    if (animation)
    {
        m_animationChannels.reserve(animation->channels.size());
        for (const auto& channel : animation->channels)
        {
            // Preserve the original lookup's first match for duplicate names.
            m_animationChannels.emplace(channel.nodeName, &channel);
        }
    }
}

const AnimationChannel* Animator::FindAnimationChannel(const std::string& nodeName) const
{
    const auto found = m_animationChannels.find(nodeName);
    return found != m_animationChannels.end() ? found->second : nullptr;
}

// Returns true when no interpolation is needed and writes the sampled value.
// This would happen when playback is close to the beginning or end of the clip
template <typename Key, typename Value>
static bool TrySampleBoundary(const Key* keys, unsigned int keyCount, double time,
	const Value& fallback, Value& result)
{
	if (!keys || keyCount == 0)
	{
		result = fallback;
		return true;
	}

	if (keyCount == 1 || time <= keys[0].timeTicks)
	{
		result = keys[0].value;
		return true;
	}

	if (time >= keys[keyCount - 1].timeTicks)
	{
		result = keys[keyCount - 1].value;
		return true;
	}

	return false;
}

// Callers handle empty arrays and endpoint times before searching sorted keys.
template <typename Key>
static KeyIndices FindSurroundingKeys(const Key* keys, unsigned int keyCount, double time)
{
	unsigned int low = 1;
	unsigned int high = keyCount - 1;

	while (low < high)
	{
		const unsigned int middle = low + (high - low) / 2;
		if (keys[middle].timeTicks < time)
		{
			low = middle + 1;
		}
		else
		{
			high = middle;
		}
	}

	return {low - 1, low};
}

// Boundary tracks have identical endpoints and a zero interpolation factor.
template <typename Value>
struct KeySegment
{
    Value start;
    Value end;
    KeyIndices indices{0, 0};
    float factor = 0.0f;
    bool interpolating = false;
    int n = 1;
    int k = 0;
};

// Finds two keyframes around the requested time and how far we are between them
// Use templates for types Quaternions, Translation, and Scaler (vec3, quaternion, float)
template <typename Key, typename Value>
static KeySegment<Value> FindKeySegment(const Key* keys, unsigned int count,
    double time, const Value& fallback, double ticksPerSecond)
{
    KeySegment<Value> segment{fallback, fallback};
    if (TrySampleBoundary(keys, count, time, fallback, segment.start))
    {
        segment.end = segment.start;
        return segment;
    }

    segment.indices = FindSurroundingKeys(keys, count, time);
    const unsigned int first = segment.indices.previous;
    const unsigned int last = segment.indices.next;

    segment.start = keys[first].value;
    segment.end = keys[last].value;

    // Caluclate progress between keys
    const double duration = keys[last].timeTicks - keys[first].timeTicks;
    segment.factor = duration > 0.0 ? static_cast<float>((time - keys[first].timeTicks) / duration) : 0.0f;

    // Keep pose sampling independent of how often the scene is rendered.
    segment.n = std::max(1, int(std::ceil(duration / ticksPerSecond *
        TimingSettings::AnimationSamplesPerSecond)));
    segment.k = std::min(segment.n, std::max(0, int(std::floor(double(segment.factor) * segment.n + 1e-6))));
    segment.interpolating = true;

    return segment;
}

static VQS SampleVQS(const AnimationChannel* channel, double time,
    const glm::vec3& defaultPosition, const Quaternion& defaultRotation,
    float defaultScale, double ticksPerSecond, 
    InterpolationMode mode, RotationInterpolationState& rotationSampler)
{
    const KeySegment<glm::vec3> position = FindKeySegment(channel->positions.data(),
        static_cast<unsigned int>(channel->positions.size()), time, defaultPosition, ticksPerSecond);
    const KeySegment<Quaternion> rotation = FindKeySegment(channel->rotations.data(),
        static_cast<unsigned int>(channel->rotations.size()), time, defaultRotation, ticksPerSecond);
    const KeySegment<float> scale = FindKeySegment(channel->scales.data(),
        static_cast<unsigned int>(channel->scales.size()), time, defaultScale, ticksPerSecond);

    const VQS start(position.start, rotation.start, scale.start);
    const VQS end(position.end, rotation.end, scale.end);

    if (mode == InterpolationMode::NonIncremental)
    {
        // Missing neighbors repeat the segment endpoints. Boundary tracks remain held.
        glm::vec3 previousPosition = position.start;
        glm::vec3 nextPosition = position.end;
        if (position.interpolating)
        {
            const auto first = position.indices.previous;
            const auto last = position.indices.next;
            if (first > 0)
            {
                previousPosition = channel->positions[first - 1].value;
            }
            if (last + 1 < channel->positions.size())
            {
                nextPosition = channel->positions[last + 1].value;
            }
        }
        return VQS::Interpolate(VQS(previousPosition, rotation.start, scale.start),
            start, end, VQS(nextPosition, rotation.end, scale.end),
            position.factor, rotation.factor, scale.factor);
    }

    const auto same = [](const Quaternion& a, const Quaternion& b) {
        return a._s == b._s && a._x == b._x && a._y == b._y && a._z == b._z;
    };

    if (rotationSampler.n != rotation.n ||
        !same(rotationSampler.start, rotation.start) ||
        !same(rotationSampler.end, rotation.end))
    {
        rotationSampler.interpolation = ISlerpState{};
        rotationSampler.start = rotation.start;
        rotationSampler.end = rotation.end;
        rotationSampler.n = rotation.n;
    }

    return VQS::InterpolateIncremental(start, end,
        position.n, position.k, rotation.n, rotation.k, scale.n, scale.k,
        rotationSampler.interpolation);
}

glm::mat4 Animator::SampleLocalTransform(const SkeletonNode* node,
	const AnimationChannel* channel, double animationTime)
{
    // We can assume that there will be animations, but just for safety
    // adding this check for code completeness
	if (!channel)
	{
		return node->bindMatrix;
	}

    const VQS base = SampleVQS(channel, animationTime,
        node->bindVQS._translation,
        node->bindVQS._rotation,
        node->bindVQS._scale,
        GetTicksPerSecond(), m_interpolationMode, m_rotationSamplers[channel]);
    if (m_blendWeight <= 0.0f)
        return base.ToMatrix();
    RotationInterpolationState targetSampler;
    const VQS target = SampleVQS(channel, m_blendTargetTicks,
        node->bindVQS._translation, node->bindVQS._rotation, node->bindVQS._scale,
        GetTicksPerSecond(), InterpolationMode::NonIncremental, targetSampler);
    // Blend local components, then let EvaluateNode rebuild the global hierarchy.
    return VQS(glm::mix(base._translation, target._translation, m_blendWeight),
        base._rotation.Slerp(base._rotation, target._rotation, m_blendWeight),
        base._scale + (target._scale - base._scale) * m_blendWeight).ToMatrix();
}
double Animator::GetFirstKeyTimeTicks() const
{
    const AnimationClip* animation = GetAnimation();
    if (!animation)
    {
        return 0.0;
    }

    double firstKeyTime = 0.0;
    bool foundKey = false;
    const auto includeTrack = [&](const auto& keys) {
        if (!keys.empty() && (!foundKey || keys.front().timeTicks < firstKeyTime))
        {
            firstKeyTime = keys.front().timeTicks;
            foundKey = true;
        }
    };

    for (const auto& channel : animation->channels)
    {
        includeTrack(channel.positions);
        includeTrack(channel.rotations);
        includeTrack(channel.scales);
    }

    return firstKeyTime;
}
