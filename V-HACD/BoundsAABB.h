#pragma once
#include <vector>
#include "Vertex.h"
#include "Vector3.h"

namespace VHACD {
    typedef VHACD::Vector3<double> Vect3;

    struct BoundsAABB
    {
        BoundsAABB() = default;
        BoundsAABB(const std::vector<VHACD::Vertex>& points);
        BoundsAABB(const Vect3& min,
            const Vect3& max);

        BoundsAABB Union(const BoundsAABB& b);

        bool Intersects(const BoundsAABB& b) const;

        double SurfaceArea() const;
        double Volume() const;

        BoundsAABB Inflate(double ratio) const;

        VHACD::Vect3 ClosestPoint(const VHACD::Vect3& p) const;

        VHACD::Vect3& GetMin();
        VHACD::Vect3& GetMax();
        const VHACD::Vect3& GetMin() const;
        const VHACD::Vect3& GetMax() const;

        VHACD::Vect3 GetSize() const;
        VHACD::Vect3 GetCenter() const;

        VHACD::Vect3 m_min{ double(0.0) };
        VHACD::Vect3 m_max{ double(0.0) };
    };

    #include "BoundsAABB.inl"
}
