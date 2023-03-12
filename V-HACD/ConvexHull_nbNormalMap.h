#pragma once

class ConvexHull::ndNormalMap
{
public:
    ndNormalMap();

    static const ndNormalMap& GetNormalMap();

    void TessellateTriangle(int level,
        const VHACD::Vect3& p0,
        const VHACD::Vect3& p1,
        const VHACD::Vect3& p2,
        int& count);

    std::array<VHACD::Vect3, 128> m_normal;
    int m_count{ 128 };
};

const ConvexHull::ndNormalMap& ConvexHull::ndNormalMap::GetNormalMap()
{
    static ndNormalMap normalMap;
    return normalMap;
}

void ConvexHull::ndNormalMap::TessellateTriangle(int level,
    const VHACD::Vect3& p0,
    const VHACD::Vect3& p1,
    const VHACD::Vect3& p2,
    int& count)
{
    if (level)
    {
        assert(fabs(p0.Dot(p0) - double(1.0)) < double(1.0e-4));
        assert(fabs(p1.Dot(p1) - double(1.0)) < double(1.0e-4));
        assert(fabs(p2.Dot(p2) - double(1.0)) < double(1.0e-4));
        VHACD::Vect3 p01(p0 + p1);
        VHACD::Vect3 p12(p1 + p2);
        VHACD::Vect3 p20(p2 + p0);

        p01 = p01 * (double(1.0) / p01.GetNorm());
        p12 = p12 * (double(1.0) / p12.GetNorm());
        p20 = p20 * (double(1.0) / p20.GetNorm());

        assert(fabs(p01.GetNormSquared() - double(1.0)) < double(1.0e-4));
        assert(fabs(p12.GetNormSquared() - double(1.0)) < double(1.0e-4));
        assert(fabs(p20.GetNormSquared() - double(1.0)) < double(1.0e-4));

        TessellateTriangle(level - 1, p0, p01, p20, count);
        TessellateTriangle(level - 1, p1, p12, p01, count);
        TessellateTriangle(level - 1, p2, p20, p12, count);
        TessellateTriangle(level - 1, p01, p12, p20, count);
    }
    else
    {
        /*
         * This is just m_normal[index] = n.Normalized(), but due to tiny floating point errors, causes
         * different outputs, so I'm leaving it
         */
        HullPlane n(p0, p1, p2);
        n = n.Scale(double(1.0) / n.GetNorm());
        n.GetW() = double(0.0);
        int index = dBitReversal(count,
            int(m_normal.size()));
        m_normal[index] = n;
        count++;
        assert(count <= int(m_normal.size()));
    }
}

ConvexHull::ndNormalMap::ndNormalMap()
{
    VHACD::Vect3 p0(double(1.0), double(0.0), double(0.0));
    VHACD::Vect3 p1(double(-1.0), double(0.0), double(0.0));
    VHACD::Vect3 p2(double(0.0), double(1.0), double(0.0));
    VHACD::Vect3 p3(double(0.0), double(-1.0), double(0.0));
    VHACD::Vect3 p4(double(0.0), double(0.0), double(1.0));
    VHACD::Vect3 p5(double(0.0), double(0.0), double(-1.0));

    int count = 0;
    int subdivisions = 2;
    TessellateTriangle(subdivisions, p4, p0, p2, count);
    TessellateTriangle(subdivisions, p0, p5, p2, count);
    TessellateTriangle(subdivisions, p5, p1, p2, count);
    TessellateTriangle(subdivisions, p1, p4, p2, count);
    TessellateTriangle(subdivisions, p0, p4, p3, count);
    TessellateTriangle(subdivisions, p5, p0, p3, count);
    TessellateTriangle(subdivisions, p1, p5, p3, count);
    TessellateTriangle(subdivisions, p4, p1, p3, count);
}
