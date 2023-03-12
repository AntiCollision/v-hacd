#pragma once

VoxelHull::VoxelHull(const VoxelHull& parent,
    SplitAxis axis,
    uint32_t splitLoc)
    : m_axis(axis)
    , m_voxels(parent.m_voxels)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale* double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    , m_depth(parent.m_depth + 1)
    , m_index(++m_voxelHullCount)
    , m_1(parent.m_1)
    , m_2(parent.m_2)
    , m_params(parent.m_params)
{
    // Default copy the voxel region from the parent, but values will
    // be adjusted next based on the split axis and location
    switch (m_axis)
    {
    case SplitAxis::X_AXIS_NEGATIVE:
        m_2.GetX() = splitLoc;
        break;
    case SplitAxis::X_AXIS_POSITIVE:
        m_1.GetX() = splitLoc + 1;
        break;
    case SplitAxis::Y_AXIS_NEGATIVE:
        m_2.GetY() = splitLoc;
        break;
    case SplitAxis::Y_AXIS_POSITIVE:
        m_1.GetY() = splitLoc + 1;
        break;
    case SplitAxis::Z_AXIS_NEGATIVE:
        m_2.GetZ() = splitLoc;
        break;
    case SplitAxis::Z_AXIS_POSITIVE:
        m_1.GetZ() = splitLoc + 1;
        break;
    }

    // First, we copy all of the interior voxels from our parent
    // which intersect our region
    for (auto& i : parent.m_interiorVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            bool newSurface = false;
            switch (m_axis)
            {
            case SplitAxis::X_AXIS_NEGATIVE:
                if (v.GetX() == splitLoc)
                {
                    newSurface = true;
                }
                break;
            case SplitAxis::X_AXIS_POSITIVE:
                if (v.GetX() == m_1.GetX())
                {
                    newSurface = true;
                }
                break;
            case SplitAxis::Y_AXIS_NEGATIVE:
                if (v.GetY() == splitLoc)
                {
                    newSurface = true;
                }
                break;
            case SplitAxis::Y_AXIS_POSITIVE:
                if (v.GetY() == m_1.GetY())
                {
                    newSurface = true;
                }
                break;
            case SplitAxis::Z_AXIS_NEGATIVE:
                if (v.GetZ() == splitLoc)
                {
                    newSurface = true;
                }
                break;
            case SplitAxis::Z_AXIS_POSITIVE:
                if (v.GetZ() == m_1.GetZ())
                {
                    newSurface = true;
                }
                break;
            }
            // If his interior voxels lie directly on the split plane then
            // these become new surface voxels for our patch
            if (newSurface)
            {
                m_newSurfaceVoxels.push_back(i);
            }
            else
            {
                m_interiorVoxels.push_back(i);
            }
        }
    }
    // Next we copy all of the surface voxels which intersect our region
    for (auto& i : parent.m_surfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_surfaceVoxels.push_back(i);
        }
    }
    // Our parent's new surface voxels become our new surface voxels so long as they intersect our region
    for (auto& i : parent.m_newSurfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_newSurfaceVoxels.push_back(i);
        }
    }

    // Recompute the min-max bounding box which would be different after the split occurs
    m_1 = VHACD::Vector3<uint32_t>(0x7FFFFFFF);
    m_2 = VHACD::Vector3<uint32_t>(0);
    for (auto& i : m_surfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_newSurfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_interiorVoxels)
    {
        MinMaxVoxelRegion(i);
    }

    BuildVoxelMesh();
    BuildRaycastMesh(); // build a raycast mesh of the voxel mesh
    ComputeConvexHull();
}

VoxelHull::VoxelHull(Volume& voxels,
    const IVHACD::Parameters& params,
    VHACDCallbacks* callbacks)
    : m_voxels(&voxels)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale* double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    , m_index(++m_voxelHullCount)
    // Here we get a copy of all voxels which lie on the surface mesh
    , m_surfaceVoxels(m_voxels->GetSurfaceVoxels())
    // Now we get a copy of all voxels which are considered part of the 'interior' of the source mesh
    , m_interiorVoxels(m_voxels->GetInteriorVoxels())
    , m_2(m_voxels->GetDimensions() - 1)
    , m_params(params)
    , m_callbacks(callbacks)
{
    BuildVoxelMesh();
    BuildRaycastMesh(); // build a raycast mesh of the voxel mesh
    ComputeConvexHull();
}

void VoxelHull::MinMaxVoxelRegion(const Voxel& v)
{
    VHACD::Vector3<uint32_t> x = v.GetVoxel();
    m_1 = m_1.CWiseMin(x);
    m_2 = m_2.CWiseMax(x);
}

void VoxelHull::BuildRaycastMesh()
{
    // Create a raycast mesh representation of the voxelized surface mesh
    if (!m_indices.empty())
    {
        m_AABBTree = AABBTree(m_vertices,
            m_indices);
    }
}

void VoxelHull::ComputeConvexHull()
{
    if (!m_vertices.empty())
    {
        // we compute the convex hull as follows...
        VHACD::QuickHull qh;
        uint32_t tcount = qh.ComputeConvexHull(m_vertices,
            uint32_t(m_vertices.size()));
        if (tcount)
        {
            m_convexHull = std::unique_ptr<IVHACD::ConvexHull>(new IVHACD::ConvexHull);

            m_convexHull->m_points = qh.GetVertices();
            m_convexHull->m_triangles = qh.GetIndices();

            VHACD::ComputeCentroid(m_convexHull->m_points,
                m_convexHull->m_triangles,
                m_convexHull->m_center);
            m_convexHull->m_volume = VHACD::ComputeMeshVolume(m_convexHull->m_points,
                m_convexHull->m_triangles);
        }
    }
    if (m_convexHull)
    {
        m_hullVolume = m_convexHull->m_volume;
    }
    // This is the volume of a single voxel
    double singleVoxelVolume = m_voxelScale * m_voxelScale * m_voxelScale;
    size_t voxelCount = m_interiorVoxels.size() + m_newSurfaceVoxels.size() + m_surfaceVoxels.size();
    m_voxelVolume = singleVoxelVolume * double(voxelCount);
    double diff = fabs(m_hullVolume - m_voxelVolume);
    m_volumeError = (diff * 100) / m_voxelVolume;
}

bool VoxelHull::IsComplete()
{
    bool ret = false;
    if (m_convexHull == nullptr)
    {
        ret = true;
    }
    else if (m_volumeError < m_params.m_minimumVolumePercentErrorAllowed)
    {
        ret = true;
    }
    else if (m_depth > m_params.m_maxRecursionDepth)
    {
        ret = true;
    }
    else
    {
        // We compute the voxel width on all 3 axes and see if they are below the min threshold size
        VHACD::Vector3<uint32_t> d = m_2 - m_1;
        if (d.GetX() <= m_params.m_minEdgeLength &&
            d.GetY() <= m_params.m_minEdgeLength &&
            d.GetZ() <= m_params.m_minEdgeLength)
        {
            ret = true;
        }
    }
    return ret;
}

VHACD::Vect3 VoxelHull::GetPoint(const int32_t x,
    const int32_t y,
    const int32_t z,
    const double scale,
    const VHACD::Vect3& bmin) const
{
    return VHACD::Vect3(x * scale + bmin.GetX(),
        y * scale + bmin.GetY(),
        z * scale + bmin.GetZ());
}

uint32_t VoxelHull::GetVertexIndex(const VHACD::Vector3<uint32_t>& p)
{
    uint32_t ret = 0;
    uint32_t address = (p.GetX() << 20) | (p.GetY() << 10) | p.GetZ();
    auto found = m_voxelIndexMap.find(address);
    if (found != m_voxelIndexMap.end())
    {
        ret = found->second;
    }
    else
    {
        VHACD::Vect3 vertex = GetPoint(p.GetX(),
            p.GetY(),
            p.GetZ(),
            m_voxelScale,
            m_voxelAdjust);
        ret = uint32_t(m_voxelIndexMap.size());
        m_voxelIndexMap[address] = ret;
        m_vertices.emplace_back(vertex);
    }
    return ret;
}

void VoxelHull::BuildVoxelMesh()
{
    // When we build the triangle mesh we do *not* need the interior voxels, only the ones
    // which lie upon the logical surface of the mesh.
    // Each time we perform a plane split, voxels which are along the splitting plane become
    // 'new surface voxels'.

    for (auto& i : m_surfaceVoxels)
    {
        AddVoxelBox(i);
    }
    for (auto& i : m_newSurfaceVoxels)
    {
        AddVoxelBox(i);
    }
}

void VoxelHull::AddVoxelBox(const Voxel& v)
{
    // The voxel position of the upper left corner of the box
    VHACD::Vector3<uint32_t> bmin(v.GetX(),
        v.GetY(),
        v.GetZ());
    // The voxel position of the lower right corner of the box
    VHACD::Vector3<uint32_t> bmax(bmin.GetX() + 1,
        bmin.GetY() + 1,
        bmin.GetZ() + 1);

    // Build the set of 8 voxel positions representing
    // the coordinates of the box
    std::array<VHACD::Vector3<uint32_t>, 8> box{ {
        { bmin.GetX(), bmin.GetY(), bmin.GetZ() },
        { bmax.GetX(), bmin.GetY(), bmin.GetZ() },
        { bmax.GetX(), bmax.GetY(), bmin.GetZ() },
        { bmin.GetX(), bmax.GetY(), bmin.GetZ() },
        { bmin.GetX(), bmin.GetY(), bmax.GetZ() },
        { bmax.GetX(), bmin.GetY(), bmax.GetZ() },
        { bmax.GetX(), bmax.GetY(), bmax.GetZ() },
        { bmin.GetX(), bmax.GetY(), bmax.GetZ() }
    } };

    // Now add the 12 triangles comprising the 3d box
    AddTri(box, 2, 1, 0);
    AddTri(box, 3, 2, 0);

    AddTri(box, 7, 2, 3);
    AddTri(box, 7, 6, 2);

    AddTri(box, 5, 1, 2);
    AddTri(box, 5, 2, 6);

    AddTri(box, 5, 4, 1);
    AddTri(box, 4, 0, 1);

    AddTri(box, 4, 6, 7);
    AddTri(box, 4, 5, 6);

    AddTri(box, 4, 7, 0);
    AddTri(box, 7, 3, 0);
}

void VoxelHull::AddTri(const std::array<VHACD::Vector3<uint32_t>, 8>& box,
    uint32_t i1,
    uint32_t i2,
    uint32_t i3)
{
    AddTriangle(box[i1], box[i2], box[i3]);
}

void VoxelHull::AddTriangle(const VHACD::Vector3<uint32_t>& p1,
    const VHACD::Vector3<uint32_t>& p2,
    const VHACD::Vector3<uint32_t>& p3)
{
    uint32_t i1 = GetVertexIndex(p1);
    uint32_t i2 = GetVertexIndex(p2);
    uint32_t i3 = GetVertexIndex(p3);

    m_indices.emplace_back(i1, i2, i3);
}

SplitAxis VoxelHull::ComputeSplitPlane(uint32_t& location)
{
    SplitAxis ret = SplitAxis::X_AXIS_NEGATIVE;

    VHACD::Vector3<uint32_t> d = m_2 - m_1;

    if (d.GetX() >= d.GetY() && d.GetX() >= d.GetZ())
    {
        ret = SplitAxis::X_AXIS_NEGATIVE;
        location = (m_2.GetX() + 1 + m_1.GetX()) / 2;
        uint32_t edgeLoc;
        if (m_params.m_findBestPlane && FindConcavityX(edgeLoc))
        {
            location = edgeLoc;
        }
    }
    else if (d.GetY() >= d.GetX() && d.GetY() >= d.GetZ())
    {
        ret = SplitAxis::Y_AXIS_NEGATIVE;
        location = (m_2.GetY() + 1 + m_1.GetY()) / 2;
        uint32_t edgeLoc;
        if (m_params.m_findBestPlane && FindConcavityY(edgeLoc))
        {
            location = edgeLoc;
        }
    }
    else
    {
        ret = SplitAxis::Z_AXIS_NEGATIVE;
        location = (m_2.GetZ() + 1 + m_1.GetZ()) / 2;
        uint32_t edgeLoc;
        if (m_params.m_findBestPlane && FindConcavityZ(edgeLoc))
        {
            location = edgeLoc;
        }
    }

    return ret;
}

VHACD::Vect3 VoxelHull::GetPosition(const VHACD::Vector3<int32_t>& ip) const
{
    return GetPoint(ip.GetX(),
        ip.GetY(),
        ip.GetZ(),
        m_voxelScale,
        m_voxelAdjust);
}

double VoxelHull::Raycast(const VHACD::Vector3<int32_t>& p1,
    const VHACD::Vector3<int32_t>& p2) const
{
    double ret;
    VHACD::Vect3 from = GetPosition(p1);
    VHACD::Vect3 to = GetPosition(p2);

    double outT;
    double faceSign;
    VHACD::Vect3 hitLocation;
    if (m_AABBTree.TraceRay(from, to, outT, faceSign, hitLocation))
    {
        ret = (from - hitLocation).GetNorm();
    }
    else
    {
        ret = 0; // if it doesn't hit anything, just assign it to zero.
    }

    return ret;
}

bool VoxelHull::FindConcavity(uint32_t idx,
    uint32_t& splitLoc)
{
    bool ret = false;

    int32_t d = (m_2[idx] - m_1[idx]) + 1; // The length of the getX axis in voxel space

    uint32_t idx1;
    uint32_t idx2;
    uint32_t idx3;
    switch (idx)
    {
    case 0: // X
        idx1 = 0;
        idx2 = 1;
        idx3 = 2;
        break;
    case 1: // Y
        idx1 = 1;
        idx2 = 0;
        idx3 = 2;
        break;
    case 2:
        idx1 = 2;
        idx2 = 1;
        idx3 = 0;
        break;
    default:
        /*
            * To silence uninitialized variable warnings
            */
        idx1 = 0;
        idx2 = 0;
        idx3 = 0;
        assert(0 && "findConcavity::idx must be 0, 1, or 2");
        break;
    }

    // We will compute the edge error on the XY plane and the XZ plane
    // searching for the greatest location of concavity
    std::vector<double> edgeError1 = std::vector<double>(d);
    std::vector<double> edgeError2 = std::vector<double>(d);

    // Counter of number of voxel samples on the XY plane we have accumulated
    uint32_t index1 = 0;

    // Compute Edge Error on the XY plane
    for (uint32_t i0 = m_1[idx1]; i0 <= m_2[idx1]; i0++)
    {
        double errorTotal = 0;
        // We now perform a raycast from the sides inward on the XY plane to
        // determine the total error (distance of the surface from the sides)
        // along this getX position.
        for (uint32_t i1 = m_1[idx2]; i1 <= m_2[idx2]; i1++)
        {
            VHACD::Vector3<int32_t> p1;
            VHACD::Vector3<int32_t> p2;
            switch (idx)
            {
            case 0:
            {
                p1 = VHACD::Vector3<int32_t>(i0, i1, m_1.GetZ() - 2);
                p2 = VHACD::Vector3<int32_t>(i0, i1, m_2.GetZ() + 2);
                break;
            }
            case 1:
            {
                p1 = VHACD::Vector3<int32_t>(i1, i0, m_1.GetZ() - 2);
                p2 = VHACD::Vector3<int32_t>(i1, i0, m_2.GetZ() + 2);
                break;
            }
            case 2:
            {
                p1 = VHACD::Vector3<int32_t>(m_1.GetX() - 2, i1, i0);
                p2 = VHACD::Vector3<int32_t>(m_2.GetX() + 2, i1, i0);
                break;
            }
            }

            double e1 = Raycast(p1, p2);
            double e2 = Raycast(p2, p1);

            errorTotal = errorTotal + e1 + e2;
        }
        // The total amount of edge error along this voxel location
        edgeError1[index1] = errorTotal;
        index1++;
    }

    // Compute edge error along the XZ plane
    uint32_t index2 = 0;

    for (uint32_t i0 = m_1[idx1]; i0 <= m_2[idx1]; i0++)
    {
        double errorTotal = 0;

        for (uint32_t i1 = m_1[idx3]; i1 <= m_2[idx3]; i1++)
        {
            VHACD::Vector3<int32_t> p1;
            VHACD::Vector3<int32_t> p2;
            switch (idx)
            {
            case 0:
            {
                p1 = VHACD::Vector3<int32_t>(i0, m_1.GetY() - 2, i1);
                p2 = VHACD::Vector3<int32_t>(i0, m_2.GetY() + 2, i1);
                break;
            }
            case 1:
            {
                p1 = VHACD::Vector3<int32_t>(m_1.GetX() - 2, i0, i1);
                p2 = VHACD::Vector3<int32_t>(m_2.GetX() + 2, i0, i1);
                break;
            }
            case 2:
            {
                p1 = VHACD::Vector3<int32_t>(i1, m_1.GetY() - 2, i0);
                p2 = VHACD::Vector3<int32_t>(i1, m_2.GetY() + 2, i0);
                break;
            }
            }

            double e1 = Raycast(p1, p2); // raycast from one side to the interior
            double e2 = Raycast(p2, p1); // raycast from the other side to the interior

            errorTotal = errorTotal + e1 + e2;
        }
        edgeError2[index2] = errorTotal;
        index2++;
    }


    // we now compute the first derivative to find the greatest spot of concavity on the XY plane
    double maxDiff = 0;
    uint32_t maxC = 0;
    for (uint32_t x = 1; x < index1; x++)
    {
        if (edgeError1[x] > 0 && edgeError1[x - 1] > 0)
        {
            double diff = abs(edgeError1[x] - edgeError1[x - 1]);
            if (diff > maxDiff)
            {
                maxDiff = diff;
                maxC = x - 1;
            }
        }
    }

    // Now see if there is a greater concavity on the XZ plane
    for (uint32_t x = 1; x < index2; x++)
    {
        if (edgeError2[x] > 0 && edgeError2[x - 1] > 0)
        {
            double diff = abs(edgeError2[x] - edgeError2[x - 1]);
            if (diff > maxDiff)
            {
                maxDiff = diff;
                maxC = x - 1;
            }
        }
    }

    splitLoc = maxC + m_1[idx1];

    // we do not allow an edge split if it is too close to the ends
    if (splitLoc > (m_1[idx1] + 4)
        && splitLoc < (m_2[idx1] - 4))
    {
        ret = true;
    }

    return ret;
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityX(uint32_t& splitLoc)
{
    return FindConcavity(0, splitLoc);
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityY(uint32_t& splitLoc)
{
    return FindConcavity(1, splitLoc);
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityZ(uint32_t& splitLoc)
{
    return FindConcavity(2, splitLoc);
}

void VoxelHull::PerformPlaneSplit()
{
    if (IsComplete())
    {
    }
    else
    {
        uint32_t splitLoc;
        SplitAxis axis = ComputeSplitPlane(splitLoc);
        switch (axis)
        {
        case SplitAxis::X_AXIS_NEGATIVE:
        case SplitAxis::X_AXIS_POSITIVE:
            // Split on the getX axis at this split location
            m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_NEGATIVE, splitLoc));
            m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_POSITIVE, splitLoc));
            break;
        case SplitAxis::Y_AXIS_NEGATIVE:
        case SplitAxis::Y_AXIS_POSITIVE:
            // Split on the 1 axis at this split location
            m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_NEGATIVE, splitLoc));
            m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_POSITIVE, splitLoc));
            break;
        case SplitAxis::Z_AXIS_NEGATIVE:
        case SplitAxis::Z_AXIS_POSITIVE:
            // Split on the getZ axis at this split location
            m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_NEGATIVE, splitLoc));
            m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_POSITIVE, splitLoc));
            break;
        }
    }
}

void VoxelHull::SaveVoxelMesh(const SimpleMesh& inputMesh,
    bool saveVoxelMesh,
    bool saveSourceMesh)
{
    char scratch[512];
    snprintf(scratch,
        sizeof(scratch),
        "voxel-mesh-%03d.obj",
        m_index);
    FILE* fph = fopen(scratch,
        "wb");
    if (fph)
    {
        uint32_t baseIndex = 1;
        if (saveVoxelMesh)
        {
            WriteOBJ(fph,
                m_vertices,
                m_indices,
                baseIndex);
            baseIndex += uint32_t(m_vertices.size());
        }
        if (saveSourceMesh)
        {
            WriteOBJ(fph,
                inputMesh.m_vertices,
                inputMesh.m_indices,
                baseIndex);
        }
        fclose(fph);
    }
}

void VoxelHull::SaveOBJ(const char* fname,
    const VoxelHull* h)
{
    FILE* fph = fopen(fname, "wb");
    if (fph)
    {
        uint32_t baseIndex = 1;
        WriteOBJ(fph,
            m_vertices,
            m_indices,
            baseIndex);

        baseIndex += uint32_t(m_vertices.size());

        WriteOBJ(fph,
            h->m_vertices,
            h->m_indices,
            baseIndex);
        fclose(fph);
    }
}

void VoxelHull::SaveOBJ(const char* fname)
{
    FILE* fph = fopen(fname, "wb");
    if (fph)
    {
        printf("Saving '%s' with %d vertices and %d triangles\n",
            fname,
            uint32_t(m_vertices.size()),
            uint32_t(m_indices.size()));
        WriteOBJ(fph,
            m_vertices,
            m_indices,
            1);
        fclose(fph);
    }
}

void VoxelHull::WriteOBJ(FILE* fph,
    const std::vector<VHACD::Vertex>& vertices,
    const std::vector<VHACD::Triangle>& indices,
    uint32_t baseIndex)
{
    if (!fph)
    {
        return;
    }

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const VHACD::Vertex& v = vertices[i];
        fprintf(fph, "v %0.9f %0.9f %0.9f\n",
            v.mX,
            v.mY,
            v.mZ);
    }

    for (size_t i = 0; i < indices.size(); ++i)
    {
        const VHACD::Triangle& t = indices[i];
        fprintf(fph, "f %d %d %d\n",
            t.mI0 + baseIndex,
            t.mI1 + baseIndex,
            t.mI2 + baseIndex);
    }
}