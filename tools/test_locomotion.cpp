#include "MotionProfile.h"
#include "LocomotionController.h"
#include "Animator.h"
#include "Path.h"
#include <iostream>
#include <limits>
#include <stdexcept>

void Check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const char* message)
{
    Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

int main()
{
    try
    {
        // check different ramp lengths, including a short cruise section
        for (const auto ramps : { std::pair<double, double>{0.2, 0.8},
            {0.05, 0.95}, {0.4, 0.45}, {0.1, 0.6} })
        {
            const double a = ramps.first, b = ramps.second;
            Near(MotionProfile::Evaluate(0, a, b).distance, 0, 1e-12, "start distance");
            Near(MotionProfile::Evaluate(1, a, b).distance, 1, 1e-12, "end distance");
            Near(MotionProfile::Evaluate(0, a, b).velocity, 0, 1e-12, "start speed");
            Near(MotionProfile::Evaluate(1, a, b).velocity, 0, 1e-12, "end speed");
            for (double boundary : {a, b})
            {
                const auto left = MotionProfile::Evaluate(boundary - 1e-9, a, b);
                const auto right = MotionProfile::Evaluate(boundary + 1e-9, a, b);
                Near(left.distance, right.distance, 1e-7, "distance continuity");
                Near(left.velocity, right.velocity, 1e-7, "speed continuity");
            }
            double previous = 0;
            double area = 0;
            for (int i = 0; i <= 10000; ++i)
            {
                const double t = i / 10000.0;
                const auto sample = MotionProfile::Evaluate(t, a, b);
                Check(sample.distance >= previous, "forward distance");
                previous = sample.distance;
                area += sample.velocity / 10000.0;
                Near(MotionProfile::TimeAtDistance(sample.distance, a, b), t,
                    1e-9, "scrub inverse");
                if (t > a && t < b)
                    Near(sample.velocity, 2 / (1 - a + b), 1e-12, "cruise speed");
            }
            Near(area, 1, 1e-7, "velocity integral");
        }

        // Verify local blending preserves a child's bone length under rotation.
        AnimationData blendData;
        blendData.hasRoot = true;
        blendData.root.name = "root";
        SkeletonNode child;
        child.name = "child";
        child.bindMatrix[3] = glm::vec4(2, 0, 0, 1);
        blendData.root.children.push_back(child);
        AnimationClip blendClip;
        blendClip.durationTicks = 10;
        blendClip.ticksPerSecond = 1;
        AnimationChannel channel;
        channel.nodeName = "root";
        channel.positions = {{0, glm::vec3(0)}, {10, glm::vec3(10, 0, 0)}};
        channel.rotations = {{0, Quaternion(1, 0, 0, 0)}, {10, Quaternion(0, 0, 0, 1)}};
        channel.scales = {{0, 1.0f}, {10, 1.0f}};
        blendClip.channels.push_back(channel);
        blendData.clips.push_back(blendClip);
        Animator blendAnimator;
        blendAnimator.Reset(&blendData);
        blendAnimator.SetPoseBlend(10, 0.5f);
        blendAnimator.EvaluatePose(0);
        auto transforms = blendAnimator.GetNodeTransforms();
        Near(transforms.at("root")[3].x, 5, 1e-5, "blend translation");
        Near(glm::length(glm::vec3(transforms.at("child")[3] - transforms.at("root")[3])),
            2, 1e-5, "blend preserves bone length");
        Near(std::abs(transforms.at("child")[3].y), 2, 1e-5, "blend interpolates rotation");
        blendAnimator.SetPoseBlend(10, 1);
        blendAnimator.EvaluatePose(0);
        Near(blendAnimator.GetNodeTransforms().at("child")[3].x, 8, 1e-5, "full standing pose");
        blendAnimator.SetPoseBlend(10, 0);
        blendAnimator.EvaluatePose(0);
        Near(blendAnimator.GetNodeTransforms().at("child")[3].x, 2, 1e-5, "disable standing blend");

        AnimationData data;
        data.clips.push_back({});
        data.clips[0].durationTicks = 500;
        data.clips[0].ticksPerSecond = 25;
        Animator animator;
        animator.Reset(&data);
        Check(animator.SetPlaybackRange({100, 125}), "animation range");
        animator.Pause();
        animator.SetCyclePhase(0.5);
        Near(animator.GetSampleTimeTicks(), 112.5, 1e-12, "paused phase sampling");
        animator.SetCyclePhase(-0.25);
        Near(animator.GetSampleTimeTicks(), 118.75, 1e-12, "negative phase wrap");
        animator.SetCyclePhase(std::numeric_limits<double>::quiet_NaN());
        Near(animator.GetSampleTimeTicks(), 118.75, 1e-12, "invalid phase ignored");

        Path path;
        const double length = path.TotalLength();
        const double stride = 2.0;
        const double duration = 16.0;
        const double endTime = 39.5;
        const double expectedDistance = MotionProfile::TotalDistance(endTime, duration, length, .2, .8);
        const double expectedPhase = expectedDistance / stride - std::floor(expectedDistance / stride);
        for (int fps : {30, 60, 144})
        {
            LocomotionController controller;
            auto settings = controller.GetSettings();
            settings.strideWorld = static_cast<float>(stride);
            Check(controller.SetSettings(settings, path), "controller settings");
            double time = 0;
            while (time < endTime)
            {
                const double next = (std::min)(endTime, time + 1.0 / fps);
                controller.Update(next - time, path);
                time = next;
            }
            const double phase = controller.GetSample().gaitPhase;
            Near(phase, expectedPhase, 1e-10, "frame independent gait");
            animator.SetCyclePhase(phase);
            Near(animator.GetSampleTimeTicks(), 100 + 25 * expectedPhase, 1e-8, "gait tick");
        }
        LocomotionController controller;
        const auto moving = controller.Update(4.0, path);
        controller.SetPlaying(false);
        const auto paused = controller.Update(5.0, path);
        Near(paused.distance, moving.distance, 0, "pause position");
        Near(paused.gaitPhase, moving.gaitPhase, 0, "pause gait");
        Near(paused.speed, 0, 0, "pause speed");
        controller.SetPlaying(true);
        Check(controller.Update(.1, path).distance > paused.distance, "resume travel");

        // settings edits preserve position and phase, including a new stride
        const auto beforeEdit = controller.GetSample();
        auto settings = controller.GetSettings();
        settings.loopSeconds = 8;
        settings.easeInEnd = .3;
        settings.strideWorld = 12.6f;
        Check(controller.SetSettings(settings, path), "edit settings");
        Near(controller.GetSample().distance, beforeEdit.distance, 0, "edit position");
        Near(controller.GetSample().gaitPhase, beforeEdit.gaitPhase, 0, "edit phase");
        Check(controller.Update(.01, path).distance > beforeEdit.distance, "edit resume");
        settings.loopSeconds = 0;
        Check(!controller.SetSettings(settings, path), "reject zero duration");
        Near(controller.GetSettings().loopSeconds, 8, 0, "keep valid settings");
        settings = controller.GetSettings();
        settings.lookAheadWorld = std::numeric_limits<float>::quiet_NaN();
        Check(!controller.SetSettings(settings, path), "reject invalid look ahead");

        controller.SetPlaying(false);
        controller.SeekDistance(length, path);
        const auto seam = controller.Update(0, path);
        Near(seam.distance, length, 0, "scrub endpoint retained");
        Near(glm::length(seam.position - path.Position(0)), 0, 1e-5, "scrub endpoint position");
        const auto forward = path.Position(path.ParameterAtDistance(float(.75 / length))) - seam.position;
        const double heading = std::atan2(forward.y, forward.x) * 180 / 3.14159265358979323846;
        Near(seam.yawDegrees, heading + 90, 1e-4, "COI heading and asset correction");
        controller.SetPlaying(true);
        const auto afterSeam = controller.Update(.1, path);
        const double step = MotionProfile::TotalDistance(8.1, 8, length, .3, .8) - length;
        double phase = seam.gaitPhase + step / controller.GetSettings().strideWorld;
        phase -= std::floor(phase);
        Near(afterSeam.gaitPhase, phase, 1e-10, "resume from scrub without extra gait advance");
        controller.Restart(path);
        Near(controller.GetSample().distance, 0, 0, "restart position");
        Near(controller.GetSample().gaitPhase, 0, 0, "restart gait");
        const auto restarted = controller.GetSample();
        controller.Update(std::numeric_limits<double>::quiet_NaN(), path);
        Near(controller.GetSample().distance, restarted.distance, 0, "invalid dt ignored");
        controller.Update(24, path);
        Near(controller.GetSample().distance, 0, 1e-10, "controller multiple laps position");
        phase = 3 * length / controller.GetSettings().strideWorld;
        Near(controller.GetSample().gaitPhase, phase - std::floor(phase), 1e-10, "controller multiple laps gait");

        // a long frame can cross several complete laps
        Near(MotionProfile::TotalDistance(48, duration, length, .2, .8),
            3 * length, 1e-10, "multiple laps");

        // COI remains defined at the seam and points into the next path segment
        const glm::vec3 start = path.Position(0);
        const glm::vec3 end = path.Position(path.points.size());
        Near(glm::length(start - end), 0, 1e-5, "closed path position");
        const glm::vec3 coi = path.Position(path.ParameterAtDistance(float(.75 / length)));
        Check(glm::length(coi - start) > 1e-4, "COI direction at rest");

        Path approach;
        approach.closed = false;
        for (std::size_t i = 0; i < approach.points.size(); ++i)
            approach.points[i] = glm::vec3(3.0f + float(i) * 2.0f, 4.0f, 0.0f);
        approach.RebuildArcLengthTables();
        Near(approach.TotalLength(), 14.0, 1e-4, "open path length");
        Near(glm::length(approach.Position(-1) - approach.points.front()), 0, 1e-5, "open start clamp");
        Near(glm::length(approach.Position(99) - approach.points.back()), 0, 1e-5, "open end clamp");
        Near(approach.ParameterAtDistance(1), 7, 1e-6, "open parameter endpoint");
        Near(approach.NormalizedDistanceAtParameter(7), 1, 1e-6, "open distance endpoint");
        LocomotionController arrival;
        arrival.Restart(approach);
        arrival.Update(100, approach);
        Check(!arrival.IsPlaying(), "open path stops playback");
        Near(glm::length(arrival.GetSample().position - approach.points.back()), 0, 1e-5, "arrival position");
        Near(arrival.GetSample().speed, 0, 0, "arrival speed");
        Near(arrival.GetSample().yawDegrees, 90, 1e-4, "arrival heading");
        const double arrivalPhase = arrival.GetSample().gaitPhase;
        arrival.Update(100, approach);
        Near(arrival.GetSample().distance, 14, 1e-4, "arrival does not wrap");
        Near(arrival.GetSample().gaitPhase, arrivalPhase, 0, "arrival holds gait");
        arrival.SeekDistance(7, approach);
        arrival.Update(1, approach);
        Near(arrival.GetSample().distance, 7, 0, "open paused seek");
        arrival.SetPlaying(true);
        arrival.Update(100, approach);
        Near(arrival.GetSample().distance, 14, 1e-4, "open resume arrival");

        std::cout << "PASS: profile math, controller pause/resume, seeking, settings, COI, loop seams, phase sampling, 30/60/144 FPS, open path arrival\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
