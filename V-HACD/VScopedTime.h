#pragma once
#include "IVHACD.h"
#include "Timer.h"

namespace VHACD {
    class ScopedTime
    {
    public:
        ScopedTime(const char* action,
            VHACD::IVHACD::IUserLogger* logger)
            : m_action(action)
            , m_logger(logger)
        {
            m_timer.Reset();
        }

        ~ScopedTime()
        {
            double dtime = m_timer.GetElapsedSeconds();
            if (m_logger)
            {
                char scratch[512];
                snprintf(scratch,
                    sizeof(scratch), "%s took %0.5f seconds",
                    m_action,
                    dtime);
                m_logger->Log(scratch);
            }
        }

        const char* m_action{ nullptr };
        Timer       m_timer;
        VHACD::IVHACD::IUserLogger* m_logger{ nullptr };
    };
}
