#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <chrono>
#include <thread>

// Owns the Windows timer and keeps frame pacing separate from animation timing.
class FrameLimiter
{
public:
    using Clock = std::chrono::steady_clock;

    explicit FrameLimiter(double framesPerSecond)
        : m_frameDuration(1.0 / framesPerSecond),
          m_timer(CreateWaitableTimerExW(nullptr, nullptr,
              CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_MODIFY_STATE | SYNCHRONIZE))
    {
    }

    ~FrameLimiter()
    {
        if (m_timer)
            CloseHandle(m_timer);
    }

    FrameLimiter(const FrameLimiter&) = delete;
    FrameLimiter& operator=(const FrameLimiter&) = delete;

    void Wait(Clock::time_point frameStart)
    {
        const auto deadline = frameStart + m_frameDuration;
        // Wake slightly early, then finish precisely without another coarse sleep.
        const auto sleepDuration = deadline - Clock::now() - std::chrono::microseconds(500);
        if (m_timer && sleepDuration.count() > 0.0)
        {
            using TimerTicks = std::chrono::duration<long long, std::ratio<1, 10000000>>;
            LARGE_INTEGER dueTime;
            dueTime.QuadPart = -std::chrono::duration_cast<TimerTicks>(sleepDuration).count();
            if (SetWaitableTimer(m_timer, &dueTime, 0, nullptr, nullptr, FALSE))
                WaitForSingleObject(m_timer, INFINITE);
        }

        // Also works as a fallback if a high-resolution timer is unavailable.
        while (Clock::now() < deadline)
            std::this_thread::yield();
    }

private:
    std::chrono::duration<double> m_frameDuration;
    HANDLE m_timer;
};
