#pragma once

bool PlaneBoxOverlap(const VHACD::Vect3& normal,
    const VHACD::Vect3& vert,
    const VHACD::Vect3& maxbox)
{
    int32_t q;
    VHACD::Vect3 vmin;
    VHACD::Vect3 vmax;
    double v;
    for (q = 0; q < 3; q++)
    {
        v = vert[q];
        if (normal[q] > double(0.0))
        {
            vmin[q] = -maxbox[q] - v;
            vmax[q] = maxbox[q] - v;
        }
        else
        {
            vmin[q] = maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if (normal.Dot(vmin) > double(0.0))
        return false;
    if (normal.Dot(vmax) >= double(0.0))
        return true;
    return false;
}

bool AxisTest(double  a, double  b, double fa, double fb,
    double v0, double v1, double v2, double v3,
    double boxHalfSize1, double boxHalfSize2)
{
    double p0 = a * v0 + b * v1;
    double p1 = a * v2 + b * v3;

    double min = std::min(p0, p1);
    double max = std::max(p0, p1);

    double rad = fa * boxHalfSize1 + fb * boxHalfSize2;
    if (min > rad || max < -rad)
    {
        return false;
    }

    return true;
}

bool TriBoxOverlap(const VHACD::Vect3& boxCenter,
    const VHACD::Vect3& boxHalfSize,
    const VHACD::Vect3& triVer0,
    const VHACD::Vect3& triVer1,
    const VHACD::Vect3& triVer2)
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-direction) */
    /*       this gives 3x3=9 more tests */

    VHACD::Vect3 v0 = triVer0 - boxCenter;
    VHACD::Vect3 v1 = triVer1 - boxCenter;
    VHACD::Vect3 v2 = triVer2 - boxCenter;
    VHACD::Vect3 e0 = v1 - v0;
    VHACD::Vect3 e1 = v2 - v1;
    VHACD::Vect3 e2 = v0 - v2;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    double fex = fabs(e0[0]);
    double fey = fabs(e0[1]);
    double fez = fabs(e0[2]);

    /*
     * These should use Get*() instead of subscript for consistency, but the function calls are long enough already
     */
    if (!AxisTest(e0[2], -e0[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e0[2], e0[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest(e0[1], -e0[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    fex = fabs(e1[0]);
    fey = fabs(e1[1]);
    fez = fabs(e1[2]);

    if (!AxisTest(e1[2], -e1[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e1[2], e1[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest(e1[1], -e1[0], fey, fex, v0[0], v0[1], v1[0], v1[1], boxHalfSize[0], boxHalfSize[2])) return 0; // Z0

    fex = fabs(e2[0]);
    fey = fabs(e2[1]);
    fez = fabs(e2[2]);

    if (!AxisTest(e2[2], -e2[1], fez, fey, v0[1], v0[2], v1[1], v1[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X2
    if (!AxisTest(-e2[2], e2[0], fez, fex, v0[0], v0[2], v1[0], v1[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y1
    if (!AxisTest(e2[1], -e2[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    double min = std::min({ v0.GetX(), v1.GetX(), v2.GetX() });
    double max = std::max({ v0.GetX(), v1.GetX(), v2.GetX() });
    if (min > boxHalfSize[0] || max < -boxHalfSize[0])
        return false;

    /* test in 1-direction */
    min = std::min({ v0.GetY(), v1.GetY(), v2.GetY() });
    max = std::max({ v0.GetY(), v1.GetY(), v2.GetY() });
    if (min > boxHalfSize[1] || max < -boxHalfSize[1])
        return false;

    /* test in getZ-direction */
    min = std::min({ v0.GetZ(), v1.GetZ(), v2.GetZ() });
    max = std::max({ v0.GetZ(), v1.GetZ(), v2.GetZ() });
    if (min > boxHalfSize[2] || max < -boxHalfSize[2])
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    VHACD::Vect3 normal = e0.Cross(e1);

    if (!PlaneBoxOverlap(normal, v0, boxHalfSize))
        return false;
    return true; /* box and triangle overlaps */
}

inline void WalkForward(int64_t start,
    int64_t end,
    VoxelValue* ptr,
    int64_t stride,
    int64_t maxDistance)
{
    for (int64_t i = start, count = 0;
        count < maxDistance && i < end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
        ++i, ptr += stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

inline void WalkBackward(int64_t start,
    int64_t end,
    VoxelValue* ptr,
    int64_t stride,
    int64_t maxDistance)
{
    for (int64_t i = start, count = 0;
        count < maxDistance && i >= end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
        --i, ptr -= stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}


class Volume
{
public:
    void Voxelize(const std::vector<VHACD::Vertex>& points,
        const std::vector<VHACD::Triangle>& triangles,
        const size_t dim,
        FillMode fillMode,
        const AABBTree& aabbTree);

    void RaycastFill(const AABBTree& aabbTree);

    void SetVoxel(const size_t i,
        const size_t j,
        const size_t k,
        VoxelValue value);

    VoxelValue& GetVoxel(const size_t i,
        const size_t j,
        const size_t k);

    const VoxelValue& GetVoxel(const size_t i,
        const size_t j,
        const size_t k) const;

    const std::vector<Voxel>& GetSurfaceVoxels() const;
    const std::vector<Voxel>& GetInteriorVoxels() const;

    double GetScale() const;
    const VHACD::BoundsAABB& GetBounds() const;
    const VHACD::Vector3<uint32_t>& GetDimensions() const;

    VHACD::BoundsAABB m_bounds;
    double m_scale{ 1.0 };
    VHACD::Vector3<uint32_t> m_dim{ 0 };
    size_t m_numVoxelsOnSurface{ 0 };
    size_t m_numVoxelsInsideSurface{ 0 };
    size_t m_numVoxelsOutsideSurface{ 0 };
    std::vector<VoxelValue> m_data;
private:

    void MarkOutsideSurface(const size_t i0,
        const size_t j0,
        const size_t k0,
        const size_t i1,
        const size_t j1,
        const size_t k1);
    void FillOutsideSurface();

    void FillInsideSurface();

    std::vector<VHACD::Voxel> m_surfaceVoxels;
    std::vector<VHACD::Voxel> m_interiorVoxels;
};
