#include "Animator.h"
#include "VQS.h"
#include "TimingSettings.h"
#include <cmath>
#include <algorithm>

void Animator::Reset(const AnimationData* data)
{
    m_data = data;
    m_animationIndex = 0;
    m_playbackSeconds = 0.0;
    m_playbackSpeed = 1.0;
    m_interpolationMode = InterpolationMode::Incremental;
    m_hasPlaybackRange = false;
    m_playing = false;
    m_nodeTransforms.clear();
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
        m_playbackSeconds = 0.0;
        m_hasPlaybackRange = false;
        m_nodeTransforms.clear();
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
        return m_playbackRange;

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
        return false;

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
        return;

    const AnimationRange range = GetPlaybackRange();
    const double durationSeconds = (range.endTicks - range.startTicks) / GetTicksPerSecond();
    m_playbackSeconds = durationSeconds > 0.0
        ? std::fmod(m_playbackSeconds + std::fmod(scaledDelta, durationSeconds), durationSeconds)
        : 0.0;
}

double Animator::GetSampleTimeTicks() const
{
    const AnimationRange range = GetPlaybackRange();
    const double durationTicks = range.endTicks - range.startTicks;
    return durationTicks > 0.0
        ? range.startTicks + std::fmod(m_playbackSeconds * GetTicksPerSecond(), durationTicks)
        : 0.0;
}

void Animator::EvaluatePose(double timeTicks)
{
    m_nodeTransforms.clear();
    if (m_data && m_data->hasRoot)
    {
        EvaluateNode(&m_data->root, glm::mat4(1.0f), timeTicks);
    }

}

void Animator::EvaluateNode(const SkeletonNode* node, const glm::mat4& parentTransform, double timeTicks)
{
    const std::string name(node->name);
    const glm::mat4 global = parentTransform * SampleLocalTransform(node,
        FindAnimationChannel(GetAnimation(), name), timeTicks);
    m_nodeTransforms[name] = m_data->inverseRootMatrix * global;
    for (unsigned int i = 0; i < node->children.size(); ++i)
    {
        EvaluateNode(&node->children[i], global, timeTicks);
    }
}

const AnimationChannel* Animator::FindAnimationChannel(const AnimationClip* animation,
	const std::string& nodeName) const
{
	if (!animation)
	{
		return nullptr;
	}

	for (unsigned int i = 0; i < animation->channels.size(); ++i)
	{
		const AnimationChannel* channel = &animation->channels[i];
		if (nodeName == channel->nodeName)
		{
			return channel;
		}

	}
	return nullptr;
}

// Returns true when no interpolation is needed and writes the sampled value.
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
    const auto first = segment.indices.previous;
    const auto last = segment.indices.next;

    segment.start = keys[first].value;
    segment.end = keys[last].value;

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
    float defaultScale, double ticksPerSecond, InterpolationMode mode)
{
    const auto position = FindKeySegment(channel->positions.data(),
        static_cast<unsigned int>(channel->positions.size()), time, defaultPosition, ticksPerSecond);
    const auto rotation = FindKeySegment(channel->rotations.data(),
        static_cast<unsigned int>(channel->rotations.size()), time, defaultRotation, ticksPerSecond);
    const auto scale = FindKeySegment(channel->scales.data(),
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
                previousPosition = channel->positions[first - 1].value;
            if (last + 1 < channel->positions.size())
                nextPosition = channel->positions[last + 1].value;
        }
        return VQS::Interpolate(VQS(previousPosition, rotation.start, scale.start),
            start, end, VQS(nextPosition, rotation.end, scale.end),
            position.factor, rotation.factor, scale.factor);
    }

    return VQS::InterpolateIncremental(start, end,
        position.n, position.k, rotation.n, rotation.k, scale.n, scale.k);
}

glm::mat4 Animator::SampleLocalTransform(const SkeletonNode* node,
	const AnimationChannel* channel, double animationTime) const
{
	if (!channel)
	{
		return node->bindMatrix;
	}

    return SampleVQS(channel, animationTime, node->bindVQS._translation,
        node->bindVQS._rotation, node->bindVQS._scale,
        GetTicksPerSecond(), m_interpolationMode).ToMatrix();
}
double Animator::GetFirstKeyTimeTicks() const
{
    const AnimationClip* animation = GetAnimation();
    if (!animation)
        return 0.0;

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
