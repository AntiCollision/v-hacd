#pragma once
#include <chrono>

namespace VHACD {
    class Timer
    {
    public:
        Timer()
            : m_startTime(std::chrono::high_resolution_clock::now())
        {
        }

        void Reset()
        {
            m_startTime = std::chrono::high_resolution_clock::now();
        }

        double GetElapsedSeconds()
        {
            auto s = PeekElapsedSeconds();
            Reset();
            return s;
        }

        double PeekElapsedSeconds()
        {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = now - m_startTime;
            return diff.count();
        }

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
    };
}
