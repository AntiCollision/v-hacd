#pragma once
#include "Vector3.h"

namespace VHACD {
    typedef VHACD::Vector3<double> Vect3;
    class HullPlane : public VHACD::Vect3
    {
    public:
        HullPlane(const HullPlane&) = default;
        HullPlane(double x,
            double y,
            double z,
            double w);

        HullPlane(const VHACD::Vect3& p,
            double w);

        HullPlane(const VHACD::Vect3& p0,
            const VHACD::Vect3& p1,
            const VHACD::Vect3& p2);

        HullPlane Scale(double s) const;

        HullPlane& operator=(const HullPlane& rhs);

        double Evalue(const VHACD::Vect3& point) const;

        double& GetW();
        const double& GetW() const;

    private:
        double m_w;
    };  


    HullPlane::HullPlane(double x,
        double y,
        double z,
        double w)
        : VHACD::Vect3(x, y, z)
        , m_w(w)
    {
    }

    HullPlane::HullPlane(const VHACD::Vect3& p,
        double w)
        : VHACD::Vect3(p)
        , m_w(w)
    {
    }

    HullPlane::HullPlane(const VHACD::Vect3& p0,
        const VHACD::Vect3& p1,
        const VHACD::Vect3& p2)
        : VHACD::Vect3((p1 - p0).Cross(p2 - p0))
        , m_w(-Dot(p0))
    {
    }

    HullPlane HullPlane::Scale(double s) const
    {
        return HullPlane(*this * s,
            m_w * s);
    }

    HullPlane& HullPlane::operator=(const HullPlane& rhs)
    {
        GetX() = rhs.GetX();
        GetY() = rhs.GetY();
        GetZ() = rhs.GetZ();
        m_w = rhs.m_w;
        return *this;
    }

    double HullPlane::Evalue(const VHACD::Vect3& point) const
    {
        return Dot(point) + m_w;
    }

    double& HullPlane::GetW()
    {
        return m_w;
    }

    const double& HullPlane::GetW() const
    {
        return m_w;
    }

}
