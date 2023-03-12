#pragma once

AABBTree::FaceSorter::FaceSorter(const std::vector<VHACD::Vertex>& positions,
    const std::vector<VHACD::Triangle>& indices,
    uint32_t axis)
    : m_vertices(positions)
    , m_indices(indices)
    , m_axis(axis)
{
}

inline bool AABBTree::FaceSorter::operator()(uint32_t lhs,
    uint32_t rhs) const
{
    double a = GetCentroid(lhs);
    double b = GetCentroid(rhs);

    if (a == b)
    {
        return lhs < rhs;
    }
    else
    {
        return a < b;
    }
}

inline double AABBTree::FaceSorter::GetCentroid(uint32_t face) const
{
    const VHACD::Vect3& a = m_vertices[m_indices[face].mI0];
    const VHACD::Vect3& b = m_vertices[m_indices[face].mI1];
    const VHACD::Vect3& c = m_vertices[m_indices[face].mI2];

    return (a[m_axis] + b[m_axis] + c[m_axis]) / double(3.0);
}

AABBTree::AABBTree(const std::vector<VHACD::Vertex>& vertices,
    const std::vector<VHACD::Triangle>& indices)
    : m_vertices(&vertices)
    , m_indices(&indices)
{
    Build();
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
    const VHACD::Vect3& to,
    double& outT,
    double& faceSign,
    VHACD::Vect3& hitLocation) const
{
    VHACD::Vect3 dir = to - start;
    double distance = dir.Normalize();
    double u, v, w;
    uint32_t faceIndex;
    bool hit = TraceRay(start,
        dir,
        outT,
        u,
        v,
        w,
        faceSign,
        faceIndex);
    if (hit)
    {
        hitLocation = start + dir * outT;
    }

    if (hit && outT > distance)
    {
        hit = false;
    }
    return hit;
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
    const VHACD::Vect3& dir,
    uint32_t& insideCount,
    uint32_t& outsideCount) const
{
    double outT, u, v, w, faceSign;
    uint32_t faceIndex;
    bool hit = TraceRay(start,
        dir,
        outT,
        u,
        v,
        w,
        faceSign,
        faceIndex);
    if (hit)
    {
        if (faceSign >= 0)
        {
            insideCount++;
        }
        else
        {
            outsideCount++;
        }
    }
    return hit;
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
    const VHACD::Vect3& dir,
    double& outT,
    double& u,
    double& v,
    double& w,
    double& faceSign,
    uint32_t& faceIndex) const
{
    outT = FLT_MAX;
    TraceRecursive(0,
        start,
        dir,
        outT,
        u,
        v,
        w,
        faceSign,
        faceIndex);
    return (outT != FLT_MAX);
}

VHACD::Vect3 AABBTree::GetCenter() const
{
    return m_nodes[0].m_extents.GetCenter();
}

VHACD::Vect3 AABBTree::GetMinExtents() const
{
    return m_nodes[0].m_extents.GetMin();
}

VHACD::Vect3 AABBTree::GetMaxExtents() const
{
    return m_nodes[0].m_extents.GetMax();
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
    double maxDistance,
    VHACD::Vect3& closestPoint) const
{
    double dis, v, w;
    uint32_t faceIndex;
    bool hit = GetClosestPointWithinDistance(point,
        maxDistance,
        dis,
        v,
        w,
        faceIndex,
        closestPoint);
    return hit;
}

// partition faces around the median face
uint32_t AABBTree::PartitionMedian(Node& n,
    uint32_t* faces,
    uint32_t numFaces)
{
    FaceSorter predicate(*m_vertices,
        *m_indices,
        n.m_extents.GetSize().LongestAxis());
    std::nth_element(faces,
        faces + numFaces / 2,
        faces + numFaces,
        predicate);

    return numFaces / 2;
}

// partition faces based on the surface area heuristic
uint32_t AABBTree::PartitionSAH(Node&,
    uint32_t* faces,
    uint32_t numFaces)
{
    uint32_t bestAxis = 0;
    uint32_t bestIndex = 0;
    double bestCost = FLT_MAX;

    for (uint32_t a = 0; a < 3; ++a)
    {
        // sort faces by centroids
        FaceSorter predicate(*m_vertices,
            *m_indices,
            a);
        std::sort(faces,
            faces + numFaces,
            predicate);

        // two passes over data to calculate upper and lower bounds
        std::vector<double> cumulativeLower(numFaces);
        std::vector<double> cumulativeUpper(numFaces);

        VHACD::BoundsAABB lower;
        VHACD::BoundsAABB upper;

        for (uint32_t i = 0; i < numFaces; ++i)
        {
            lower.Union(m_faceBounds[faces[i]]);
            upper.Union(m_faceBounds[faces[numFaces - i - 1]]);

            cumulativeLower[i] = lower.SurfaceArea();
            cumulativeUpper[numFaces - i - 1] = upper.SurfaceArea();
        }

        double invTotalSA = double(1.0) / cumulativeUpper[0];

        // test all split positions
        for (uint32_t i = 0; i < numFaces - 1; ++i)
        {
            double pBelow = cumulativeLower[i] * invTotalSA;
            double pAbove = cumulativeUpper[i] * invTotalSA;

            double cost = double(0.125) + (pBelow * i + pAbove * (numFaces - i));
            if (cost <= bestCost)
            {
                bestCost = cost;
                bestIndex = i;
                bestAxis = a;
            }
        }
    }

    // re-sort by best axis
    FaceSorter predicate(*m_vertices,
        *m_indices,
        bestAxis);
    std::sort(faces,
        faces + numFaces,
        predicate);

    return bestIndex + 1;
}

void AABBTree::Build()
{
    const uint32_t numFaces = uint32_t(m_indices->size());

    // build initial list of faces
    m_faces.reserve(numFaces);

    // calculate bounds of each face and store
    m_faceBounds.reserve(numFaces);

    std::vector<VHACD::BoundsAABB> stack;
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        VHACD::BoundsAABB top = CalculateFaceBounds(&i,
            1);

        m_faces.push_back(i);
        m_faceBounds.push_back(top);
    }

    m_nodes.reserve(uint32_t(numFaces * double(1.5)));

    // allocate space for all the nodes
    m_freeNode = 1;

    // start building
    BuildRecursive(0,
        m_faces.data(),
        numFaces);

    assert(s_depth == 0);
}

void AABBTree::BuildRecursive(uint32_t nodeIndex,
    uint32_t* faces,
    uint32_t numFaces)
{
    const uint32_t kMaxFacesPerLeaf = 6;

    // if we've run out of nodes allocate some more
    if (nodeIndex >= m_nodes.size())
    {
        uint32_t s = std::max(uint32_t(double(1.5) * m_nodes.size()), 512U);
        m_nodes.resize(s);
    }

    // a reference to the current node, need to be careful here as this reference may become invalid if array is resized
    Node& n = m_nodes[nodeIndex];

    // track max tree depth
    ++s_depth;
    m_treeDepth = std::max(m_treeDepth, s_depth);

    n.m_extents = CalculateFaceBounds(faces,
        numFaces);

    // calculate bounds of faces and add node
    if (numFaces <= kMaxFacesPerLeaf)
    {
        n.m_faces = faces;
        n.m_numFaces = numFaces;

        ++m_leafNodes;
    }
    else
    {
        ++m_innerNodes;

        // face counts for each branch
        const uint32_t leftCount = PartitionMedian(n, faces, numFaces);
        // const uint32_t leftCount = PartitionSAH(n, faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // alloc 2 nodes
        m_nodes[nodeIndex].m_children = m_freeNode;

        // allocate two nodes
        m_freeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(m_nodes[nodeIndex].m_children + 0, faces, leftCount);
        BuildRecursive(m_nodes[nodeIndex].m_children + 1, faces + leftCount, rightCount);
    }

    --s_depth;
}

void AABBTree::TraceRecursive(uint32_t nodeIndex,
    const VHACD::Vect3& start,
    const VHACD::Vect3& dir,
    double& outT,
    double& outU,
    double& outV,
    double& outW,
    double& faceSign,
    uint32_t& faceIndex) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        double dist[2] = { FLT_MAX, FLT_MAX };

        IntersectRayAABB(start,
            dir,
            leftChild.m_extents,
            dist[0]);
        IntersectRayAABB(start,
            dir,
            rightChild.m_extents,
            dist[1]);

        uint32_t closest = 0;
        uint32_t furthest = 1;

        if (dist[1] < dist[0])
        {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
        {
            TraceRecursive(node.m_children + closest,
                start,
                dir,
                outT,
                outU,
                outV,
                outW,
                faceSign,
                faceIndex);
        }

        if (dist[furthest] < outT)
        {
            TraceRecursive(node.m_children + furthest,
                start,
                dir,
                outT,
                outU,
                outV,
                outW,
                faceSign,
                faceIndex);
        }
    }
    else
    {
        double t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];
            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s, NULL))
            {
                if (t < outT)
                {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
    const double maxDis,
    double& dis,
    double& v,
    double& w,
    uint32_t& faceIndex,
    VHACD::Vect3& closest) const
{
    dis = maxDis;
    faceIndex = uint32_t(~0);
    double disSq = dis * dis;

    GetClosestPointWithinDistanceSqRecursive(0,
        point,
        disSq,
        v,
        w,
        faceIndex,
        closest);
    dis = sqrt(disSq);

    return (faceIndex < (~(static_cast<unsigned int>(0))));
}

void AABBTree::GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
    const VHACD::Vect3& point,
    double& outDisSq,
    double& outV,
    double& outW,
    uint32_t& outFaceIndex,
    VHACD::Vect3& closestPoint) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == nullptr)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        // double dist[2] = { FLT_MAX, FLT_MAX };
        VHACD::Vect3 lp = leftChild.m_extents.ClosestPoint(point);
        VHACD::Vect3 rp = rightChild.m_extents.ClosestPoint(point);


        uint32_t closest = 0;
        uint32_t furthest = 1;
        double dcSq = (point - lp).GetNormSquared();
        double dfSq = (point - rp).GetNormSquared();
        if (dfSq < dcSq)
        {
            closest = 1;
            furthest = 0;
            std::swap(dfSq, dcSq);
        }

        if (dcSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + closest,
                point,
                outDisSq,
                outV,
                outW,
                outFaceIndex,
                closestPoint);
        }

        if (dfSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + furthest,
                point,
                outDisSq,
                outV,
                outW,
                outFaceIndex,
                closestPoint);
        }
    }
    else
    {

        double v, w;
        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];

            VHACD::Vect3 cp = ClosestPointOnTriangle(a, b, c, point, v, w);
            double disSq = (cp - point).GetNormSquared();

            if (disSq < outDisSq)
            {
                closestPoint = cp;
                outDisSq = disSq;
                outV = v;
                outW = w;
                outFaceIndex = node.m_faces[i];
            }
        }
    }
}

VHACD::BoundsAABB AABBTree::CalculateFaceBounds(uint32_t* faces,
    uint32_t numFaces)
{
    VHACD::Vect3 minExtents(FLT_MAX);
    VHACD::Vect3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        VHACD::Vect3 a = (*m_vertices)[(*m_indices)[faces[i]].mI0];
        VHACD::Vect3 b = (*m_vertices)[(*m_indices)[faces[i]].mI1];
        VHACD::Vect3 c = (*m_vertices)[(*m_indices)[faces[i]].mI2];

        minExtents = a.CWiseMin(minExtents);
        maxExtents = a.CWiseMax(maxExtents);

        minExtents = b.CWiseMin(minExtents);
        maxExtents = b.CWiseMax(maxExtents);

        minExtents = c.CWiseMin(minExtents);
        maxExtents = c.CWiseMax(maxExtents);
    }

    return VHACD::BoundsAABB(minExtents,
        maxExtents);
}
