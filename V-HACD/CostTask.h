#pragma once

class VHACDImpl;

// This class represents a single task to compute the volume error
// of two convex hulls combined
class CostTask
{
public:
    VHACDImpl* m_this{ nullptr };
    IVHACD::ConvexHull* m_hullA{ nullptr };
    IVHACD::ConvexHull* m_hullB{ nullptr };
    double              m_concavity{ 0 }; // concavity of the two combined
    std::future<void>   m_future;
};
