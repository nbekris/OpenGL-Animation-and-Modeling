#pragma once

namespace TimingSettings
{
    constexpr double TargetFramesPerSecond = 144.0;
    // Pose resolution in animation time, independent of the render frame rate.
    constexpr double AnimationSamplesPerSecond = 144.0;
    // Preserve the original held-key zoom speed at 60 FPS.
    constexpr double ZoomReferenceFramesPerSecond = 60.0;
}
