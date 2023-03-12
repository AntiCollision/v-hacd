#pragma once
#include <vector>
#include <list>
#include <array>

#include "HullPlane.h"
#include "Vector3.h"
#include "Googol.h"

namespace VHACD {

    class ConvexHullFace
    {
    public:
        ConvexHullFace() = default;
        double Evalue(const std::vector<VHACD::Vect3>& pointArray,
            const VHACD::Vect3& point) const;
        HullPlane GetPlaneEquation(const std::vector<VHACD::Vect3>& pointArray,
            bool& isValid) const;

        std::array<int, 3> m_index;
    private:
        int m_mark{ 0 };
        std::array<std::list<ConvexHullFace>::iterator, 3> m_twin;

        friend class ConvexHull;
    };


    double ConvexHullFace::Evalue(const std::vector<VHACD::Vect3>& pointArray,
        const VHACD::Vect3& point) const
    {
        const VHACD::Vect3& p0 = pointArray[m_index[0]];
        const VHACD::Vect3& p1 = pointArray[m_index[1]];
        const VHACD::Vect3& p2 = pointArray[m_index[2]];

        std::array<VHACD::Vect3, 3> matrix = { p2 - p0, p1 - p0, point - p0 };
        double error;
        double det = Determinant3x3(matrix,
            error);

        // the code use double, however the threshold for accuracy test is the machine precision of a float.
        // by changing this to a smaller number, the code should run faster since many small test will be considered valid
        // the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
        // float64(1<<30) can be a good value

        // double precision	= double (1.0f) / double (1<<30);
        double precision = double(1.0) / double(1 << 24);
        double errbound = error * precision;
        if (fabs(det) > errbound)
        {
            return det;
        }

        const VHACD::Vector3<Googol> p0g = pointArray[m_index[0]];
        const VHACD::Vector3<Googol> p1g = pointArray[m_index[1]];
        const VHACD::Vector3<Googol> p2g = pointArray[m_index[2]];
        const VHACD::Vector3<Googol> pointg = point;
        std::array<VHACD::Vector3<Googol>, 3> exactMatrix = { p2g - p0g, p1g - p0g, pointg - p0g };
        return Determinant3x3(exactMatrix);
    }

    HullPlane ConvexHullFace::GetPlaneEquation(const std::vector<VHACD::Vect3>& pointArray,
        bool& isvalid) const
    {
        const VHACD::Vect3& p0 = pointArray[m_index[0]];
        const VHACD::Vect3& p1 = pointArray[m_index[1]];
        const VHACD::Vect3& p2 = pointArray[m_index[2]];
        HullPlane plane(p0, p1, p2);

        isvalid = false;
        double mag2 = plane.Dot(plane);
        if (mag2 > double(1.0e-16))
        {
            isvalid = true;
            plane = plane.Scale(double(1.0) / sqrt(mag2));
        }
        return plane;
    }

}