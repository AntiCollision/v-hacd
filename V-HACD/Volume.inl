#pragma once

void Volume::Voxelize(const std::vector<VHACD::Vertex>& points,
    const std::vector<VHACD::Triangle>& indices,
    const size_t dimensions,
    FillMode fillMode,
    const AABBTree& aabbTree)
{
    double a = std::pow(dimensions, 0.33);
    size_t dim = a * double(1.5);
    dim = std::max(dim, size_t(32));

    if (points.size() == 0)
    {
        return;
    }

    m_bounds = BoundsAABB(points);

    VHACD::Vect3 d = m_bounds.GetSize();
    double r;
    // Equal comparison is important here to avoid taking the last branch when d[0] == d[1] with d[2] being the smallest
    // dimension. That would lead to dimensions in i and j to be a lot bigger than expected and make the amount of
    // voxels in the volume totally unmanageable.
    if (d[0] >= d[1] && d[0] >= d[2])
    {
        r = d[0];
        m_dim[0] = uint32_t(dim);
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[0]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[0]));
    }
    else if (d[1] >= d[0] && d[1] >= d[2])
    {
        r = d[1];
        m_dim[1] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[1]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[1]));
    }
    else
    {
        r = d[2];
        m_dim[2] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[2]));
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[2]));
    }

    m_scale = r / (dim - 1);
    double invScale = (dim - 1) / r;

    m_data = std::vector<VoxelValue>(m_dim[0] * m_dim[1] * m_dim[2],
        VoxelValue::PRIMITIVE_UNDEFINED);
    m_numVoxelsOnSurface = 0;
    m_numVoxelsInsideSurface = 0;
    m_numVoxelsOutsideSurface = 0;

    VHACD::Vect3 p[3];
    VHACD::Vect3 boxcenter;
    VHACD::Vect3 pt;
    const VHACD::Vect3 boxhalfsize(double(0.5));
    for (size_t t = 0; t < indices.size(); ++t)
    {
        size_t i0, j0, k0;
        size_t i1, j1, k1;
        VHACD::Vector3<uint32_t> tri = indices[t];
        for (int32_t c = 0; c < 3; ++c)
        {
            pt = points[tri[c]];

            p[c] = (pt - m_bounds.GetMin()) * invScale;

            size_t i = static_cast<size_t>(p[c][0] + double(0.5));
            size_t j = static_cast<size_t>(p[c][1] + double(0.5));
            size_t k = static_cast<size_t>(p[c][2] + double(0.5));

            assert(i < m_dim[0] && j < m_dim[1] && k < m_dim[2]);

            if (c == 0)
            {
                i0 = i1 = i;
                j0 = j1 = j;
                k0 = k1 = k;
            }
            else
            {
                i0 = std::min(i0, i);
                j0 = std::min(j0, j);
                k0 = std::min(k0, k);

                i1 = std::max(i1, i);
                j1 = std::max(j1, j);
                k1 = std::max(k1, k);
            }
        }
        if (i0 > 0)
            --i0;
        if (j0 > 0)
            --j0;
        if (k0 > 0)
            --k0;
        if (i1 < m_dim[0])
            ++i1;
        if (j1 < m_dim[1])
            ++j1;
        if (k1 < m_dim[2])
            ++k1;
        for (size_t i_id = i0; i_id < i1; ++i_id)
        {
            boxcenter[0] = uint32_t(i_id);
            for (size_t j_id = j0; j_id < j1; ++j_id)
            {
                boxcenter[1] = uint32_t(j_id);
                for (size_t k_id = k0; k_id < k1; ++k_id)
                {
                    boxcenter[2] = uint32_t(k_id);
                    bool res = TriBoxOverlap(boxcenter,
                        boxhalfsize,
                        p[0],
                        p[1],
                        p[2]);
                    VoxelValue& value = GetVoxel(i_id,
                        j_id,
                        k_id);
                    if (res
                        && value == VoxelValue::PRIMITIVE_UNDEFINED)
                    {
                        value = VoxelValue::PRIMITIVE_ON_SURFACE;
                        ++m_numVoxelsOnSurface;
                        m_surfaceVoxels.emplace_back(uint32_t(i_id),
                            uint32_t(j_id),
                            uint32_t(k_id));
                    }
                }
            }
        }
    }

    if (fillMode == FillMode::SURFACE_ONLY)
    {
        const size_t i0_local = m_dim[0];
        const size_t j0_local = m_dim[1];
        const size_t k0_local = m_dim[2];
        for (size_t i_id = 0; i_id < i0_local; ++i_id)
        {
            for (size_t j_id = 0; j_id < j0_local; ++j_id)
            {
                for (size_t k_id = 0; k_id < k0_local; ++k_id)
                {
                    const VoxelValue& voxel = GetVoxel(i_id,
                        j_id,
                        k_id);
                    if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                    {
                        SetVoxel(i_id,
                            j_id,
                            k_id,
                            VoxelValue::PRIMITIVE_OUTSIDE_SURFACE);
                    }
                }
            }
        }
    }
    else if (fillMode == FillMode::FLOOD_FILL)
    {
        /*
         * Marking the outside edges of the voxel cube to be outside surfaces to walk
         */
        MarkOutsideSurface(0, 0, 0, m_dim[0], m_dim[1], 1);
        MarkOutsideSurface(0, 0, m_dim[2] - 1, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0, 0, 0, m_dim[0], 1, m_dim[2]);
        MarkOutsideSurface(0, m_dim[1] - 1, 0, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0, 0, 0, 1, m_dim[1], m_dim[2]);
        MarkOutsideSurface(m_dim[0] - 1, 0, 0, m_dim[0], m_dim[1], m_dim[2]);
        FillOutsideSurface();
        FillInsideSurface();
    }
    else if (fillMode == FillMode::RAYCAST_FILL)
    {
        RaycastFill(aabbTree);
    }
}

void Volume::RaycastFill(const AABBTree& aabbTree)
{
    const uint32_t i0 = m_dim[0];
    const uint32_t j0 = m_dim[1];
    const uint32_t k0 = m_dim[2];

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };
    m_numVoxelsInsideSurface = 0;
    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& voxel = GetVoxel(i, j, k);
                if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                {
                    VHACD::Vect3 start = VHACD::Vect3(i, j, k) * m_scale + m_bounds.GetMin();

                    uint32_t insideCount = 0;
                    uint32_t outsideCount = 0;

                    VHACD::Vect3 directions[6] = {
                        VHACD::Vect3(1,  0,  0),
                        VHACD::Vect3(-1,  0,  0), // this was 1, 0, 0 in the original code, but looks wrong
                        VHACD::Vect3(0,  1,  0),
                        VHACD::Vect3(0, -1,  0),
                        VHACD::Vect3(0,  0,  1),
                        VHACD::Vect3(0,  0, -1)
                    };

                    for (uint32_t r = 0; r < 6; r++)
                    {
                        aabbTree.TraceRay(start,
                            directions[r],
                            insideCount,
                            outsideCount);
                        // Early out if we hit the outside of the mesh
                        if (outsideCount)
                        {
                            break;
                        }
                        // Early out if we accumulated 3 inside hits
                        if (insideCount >= 3)
                        {
                            break;
                        }
                    }

                    if (outsideCount == 0 && insideCount >= 3)
                    {
                        voxel = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                        temp.emplace_back(i, j, k);
                        count++;
                        m_numVoxelsInsideSurface++;
                    }
                    else
                    {
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;
                    }
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

void Volume::SetVoxel(const size_t i,
    const size_t j,
    const size_t k,
    VoxelValue value)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);

    m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]] = value;
}

VoxelValue& Volume::GetVoxel(const size_t i,
    const size_t j,
    const size_t k)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const VoxelValue& Volume::GetVoxel(const size_t i,
    const size_t j,
    const size_t k) const
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const std::vector<Voxel>& Volume::GetSurfaceVoxels() const
{
    return m_surfaceVoxels;
}

const std::vector<Voxel>& Volume::GetInteriorVoxels() const
{
    return m_interiorVoxels;
}

double Volume::GetScale() const
{
    return m_scale;
}

const VHACD::BoundsAABB& Volume::GetBounds() const
{
    return m_bounds;
}

const VHACD::Vector3<uint32_t>& Volume::GetDimensions() const
{
    return m_dim;
}

void Volume::MarkOutsideSurface(const size_t i0,
    const size_t j0,
    const size_t k0,
    const size_t i1,
    const size_t j1,
    const size_t k1)
{
    for (size_t i = i0; i < i1; ++i)
    {
        for (size_t j = j0; j < j1; ++j)
        {
            for (size_t k = k0; k < k1; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
                }
            }
        }
    }
}


void Volume::FillOutsideSurface()
{
    size_t voxelsWalked = 0;
    const int64_t i0 = m_dim[0];
    const int64_t j0 = m_dim[1];
    const int64_t k0 = m_dim[2];

    // Avoid striding too far in each direction to stay in L1 cache as much as possible.
    // The cache size required for the walk is roughly (4 * walkDistance * 64) since
    // the k direction doesn't count as it's walking byte per byte directly in a cache lines.
    // ~16k is required for a walk distance of 64 in each directions.
    const size_t walkDistance = 64;

    // using the stride directly instead of calling GetVoxel for each iterations saves
    // a lot of multiplications and pipeline stalls due to data dependencies on imul.
    const size_t istride = &GetVoxel(1, 0, 0) - &GetVoxel(0, 0, 0);
    const size_t jstride = &GetVoxel(0, 1, 0) - &GetVoxel(0, 0, 0);
    const size_t kstride = &GetVoxel(0, 0, 1) - &GetVoxel(0, 0, 0);

    // It might seem counter intuitive to go over the whole voxel range multiple times
    // but since we do the run in memory order, it leaves us with far fewer cache misses
    // than a BFS algorithm and it has the additional benefit of not requiring us to
    // store and manipulate a fifo for recursion that might become huge when the number
    // of voxels is large.
    // This will outperform the BFS algorithm by several orders of magnitude in practice.
    do
    {
        voxelsWalked = 0;
        for (int64_t i = 0; i < i0; ++i)
        {
            for (int64_t j = 0; j < j0; ++j)
            {
                for (int64_t k = 0; k < k0; ++k)
                {
                    VoxelValue& voxel = GetVoxel(i, j, k);
                    if (voxel == VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK)
                    {
                        voxelsWalked++;
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;

                        // walk in each direction to mark other voxel that should be walked.
                        // this will generate a 3d pattern that will help the overall
                        // algorithm converge faster while remaining cache friendly.
                        WalkForward(k + 1, k0, &voxel + kstride, kstride, walkDistance);
                        WalkBackward(k - 1, 0, &voxel - kstride, kstride, walkDistance);

                        WalkForward(j + 1, j0, &voxel + jstride, jstride, walkDistance);
                        WalkBackward(j - 1, 0, &voxel - jstride, jstride, walkDistance);

                        WalkForward(i + 1, i0, &voxel + istride, istride, walkDistance);
                        WalkBackward(i - 1, 0, &voxel - istride, istride, walkDistance);
                    }
                }
            }
        }

        m_numVoxelsOutsideSurface += voxelsWalked;
    } while (voxelsWalked != 0);
}

void Volume::FillInsideSurface()
{
    const uint32_t i0 = uint32_t(m_dim[0]);
    const uint32_t j0 = uint32_t(m_dim[1]);
    const uint32_t k0 = uint32_t(m_dim[2]);

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };

    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                    temp.emplace_back(i, j, k);
                    count++;
                    ++m_numVoxelsInsideSurface;
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

