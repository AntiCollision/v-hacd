#pragma once
#include <cstdint>

class HullPair
{
public:
    HullPair() = default;
    HullPair(uint32_t hullA,
        uint32_t hullB,
        double concavity)
        : m_hullA(hullA)
        , m_hullB(hullB)
        , m_concavity(concavity)
    {
    }

    bool operator<(const HullPair& h) const
    {
        return m_concavity > h.m_concavity ? true : false;
    }

    uint32_t    m_hullA{ 0 };
    uint32_t    m_hullB{ 0 };
    double      m_concavity{ 0 };
};
