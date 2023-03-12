#pragma once

BoundsAABB::BoundsAABB(const std::vector<VHACD::Vertex>& points)
    : m_min(points[0])
    , m_max(points[0])
{
    for (uint32_t i = 1; i < points.size(); ++i)
    {
        const VHACD::Vertex& p = points[i];
        m_min = m_min.CWiseMin(p);
        m_max = m_max.CWiseMax(p);
    }
}

BoundsAABB::BoundsAABB(const VHACD::Vect3& min,
    const VHACD::Vect3& max)
    : m_min(min)
    , m_max(max)
{
}

BoundsAABB BoundsAABB::Union(const BoundsAABB& b)
{
    return BoundsAABB(GetMin().CWiseMin(b.GetMin()),
        GetMax().CWiseMax(b.GetMax()));
}

bool VHACD::BoundsAABB::Intersects(const VHACD::BoundsAABB& b) const
{
    if ((GetMin().GetX() > b.GetMax().GetX())
        || (b.GetMin().GetX() > GetMax().GetX()))
        return false;
    if ((GetMin().GetY() > b.GetMax().GetY())
        || (b.GetMin().GetY() > GetMax().GetY()))
        return false;
    if ((GetMin().GetZ() > b.GetMax().GetZ())
        || (b.GetMin().GetZ() > GetMax().GetZ()))
        return false;
    return true;
}

double BoundsAABB::SurfaceArea() const
{
    VHACD::Vect3 d = GetMax() - GetMin();
    return double(2.0) * (d.GetX() * d.GetY() + d.GetX() * d.GetZ() + d.GetY() * d.GetZ());
}

double VHACD::BoundsAABB::Volume() const
{
    VHACD::Vect3 d = GetMax() - GetMin();
    return d.GetX() * d.GetY() * d.GetZ();
}

BoundsAABB VHACD::BoundsAABB::Inflate(double ratio) const
{
    double inflate = (GetMin() - GetMax()).GetNorm() * double(0.5) * ratio;
    return BoundsAABB(GetMin() - inflate,
        GetMax() + inflate);
}

VHACD::Vect3 VHACD::BoundsAABB::ClosestPoint(const VHACD::Vect3& p) const
{
    return p.CWiseMax(GetMin()).CWiseMin(GetMax());
}

VHACD::Vect3& VHACD::BoundsAABB::GetMin()
{
    return m_min;
}

VHACD::Vect3& VHACD::BoundsAABB::GetMax()
{
    return m_max;
}

inline const VHACD::Vect3& VHACD::BoundsAABB::GetMin() const
{
    return m_min;
}

const VHACD::Vect3& VHACD::BoundsAABB::GetMax() const
{
    return m_max;
}

VHACD::Vect3 VHACD::BoundsAABB::GetSize() const
{
    return GetMax() - GetMin();
}

VHACD::Vect3 VHACD::BoundsAABB::GetCenter() const
{
    return (GetMin() + GetMax()) * double(0.5);
}
