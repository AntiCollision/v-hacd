#pragma once

void VHACDImpl::Cancel()
{
    m_canceled = true;
}

bool VHACDImpl::Compute(const float* const points,
    const uint32_t countPoints,
    const uint32_t* const triangles,
    const uint32_t countTriangles,
    const Parameters& params)
{
    std::vector<VHACD::Vertex> v;
    v.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        v.emplace_back(points[i * 3 + 0],
            points[i * 3 + 1],
            points[i * 3 + 2]);
    }

    std::vector<VHACD::Triangle> t;
    t.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        t.emplace_back(triangles[i * 3 + 0],
            triangles[i * 3 + 1],
            triangles[i * 3 + 2]);
    }

    return Compute(v, t, params);
}

bool VHACDImpl::Compute(const double* const points,
    const uint32_t countPoints,
    const uint32_t* const triangles,
    const uint32_t countTriangles,
    const Parameters& params)
{
    std::vector<VHACD::Vertex> v;
    v.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        v.emplace_back(points[i * 3 + 0],
            points[i * 3 + 1],
            points[i * 3 + 2]);
    }

    std::vector<VHACD::Triangle> t;
    t.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        t.emplace_back(triangles[i * 3 + 0],
            triangles[i * 3 + 1],
            triangles[i * 3 + 2]);
    }

    return Compute(v, t, params);
}

uint32_t VHACDImpl::GetNConvexHulls() const
{
    return uint32_t(m_convexHulls.size());
}

bool VHACDImpl::GetConvexHull(const uint32_t index,
    ConvexHull& ch) const
{
    bool ret = false;

    if (index < uint32_t(m_convexHulls.size()))
    {
        ch = *m_convexHulls[index];
        ret = true;
    }

    return ret;
}

void VHACDImpl::Clean()
{
#if !VHACD_DISABLE_THREADING
    m_threadPool = nullptr;
#endif

    m_trees.clear();

    for (auto& ch : m_convexHulls)
    {
        ReleaseConvexHull(ch);
    }
    m_convexHulls.clear();

    for (auto& ch : m_hulls)
    {
        ReleaseConvexHull(ch.second);
    }
    m_hulls.clear();

    m_voxelHulls.clear();

    m_pendingHulls.clear();

    m_vertices.clear();
    m_indices.clear();
}

void VHACDImpl::Release()
{
    delete this;
}

bool VHACDImpl::ComputeCenterOfMass(double centerOfMass[3]) const
{
    bool ret = false;

    return ret;
}

bool VHACDImpl::IsReady() const
{
    return true;
}

uint32_t VHACDImpl::findNearestConvexHull(const double pos[3],
    double& distanceToHull)
{
    uint32_t ret = 0; // The default return code is zero

    uint32_t hullCount = GetNConvexHulls();
    distanceToHull = 0;
    // First, make sure that we have valid and completed results
    if (hullCount)
    {
        // See if we already have AABB trees created for each convex hull
        if (m_trees.empty())
        {
            // For each convex hull, we generate an AABB tree for fast closest point queries
            for (uint32_t i = 0; i < hullCount; i++)
            {
                VHACD::IVHACD::ConvexHull ch;
                GetConvexHull(i, ch);
                // Pass the triangle mesh to create an AABB tree instance based on it.
                m_trees.emplace_back(new AABBTree(ch.m_points,
                    ch.m_triangles));
            }
        }
        // We now compute the closest point to each convex hull and save the nearest one
        double closest = 1e99;
        for (uint32_t i = 0; i < hullCount; i++)
        {
            std::unique_ptr<AABBTree>& t = m_trees[i];
            if (t)
            {
                VHACD::Vect3 closestPoint;
                VHACD::Vect3 position(pos[0],
                    pos[1],
                    pos[2]);
                if (t->GetClosestPointWithinDistance(position, 1e99, closestPoint))
                {
                    VHACD::Vect3 d = position - closestPoint;
                    double distanceSquared = d.GetNormSquared();
                    if (distanceSquared < closest)
                    {
                        closest = distanceSquared;
                        ret = i;
                    }
                }
            }
        }
        distanceToHull = sqrt(closest); // compute the distance to the nearest convex hull
    }

    return ret;
}

bool VHACDImpl::Compute(const std::vector<VHACD::Vertex>& points,
    const std::vector<VHACD::Triangle>& triangles,
    const Parameters& params)
{
    bool ret = false;

    m_params = params;
    m_canceled = false;

    Clean(); // release any previous results
#if !VHACD_DISABLE_THREADING
    if (m_params.m_asyncACD)
    {
        m_threadPool = std::unique_ptr<ThreadPool>(new ThreadPool(8));
    }
#endif
    CopyInputMesh(points,
        triangles);
    if (!m_canceled)
    {
        // We now recursively perform convex decomposition until complete
        PerformConvexDecomposition();
    }

    if (m_canceled)
    {
        Clean();
        ret = false;
        if (m_params.m_logger)
        {
            m_params.m_logger->Log("VHACD operation canceled before it was complete.");
        }
    }
    else
    {
        ret = true;
    }
#if !VHACD_DISABLE_THREADING
    m_threadPool = nullptr;
#endif
    return ret;
}

uint32_t VHACDImpl::GetIndex(VHACD::VertexIndex& vi,
    const VHACD::Vertex& p)
{
    VHACD::Vect3 pos = (VHACD::Vect3(p) - m_center) * m_recipScale;
    bool newPos;
    uint32_t ret = vi.GetIndex(pos,
        newPos);
    return ret;
}

void VHACDImpl::CopyInputMesh(const std::vector<VHACD::Vertex>& points,
    const std::vector<VHACD::Triangle>& triangles)
{
    m_vertices.clear();
    m_indices.clear();
    m_indices.reserve(triangles.size());

    // First we must find the bounding box of this input vertices and normalize them into a unit-cube
    VHACD::Vect3 bmin(FLT_MAX);
    VHACD::Vect3 bmax(-FLT_MAX);
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
        0,
        "ComputingBounds");
    for (uint32_t i = 0; i < points.size(); i++)
    {
        const VHACD::Vertex& p = points[i];

        bmin = bmin.CWiseMin(p);
        bmax = bmax.CWiseMax(p);
    }
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
        100,
        "ComputingBounds");

    m_center = (bmax + bmin) * double(0.5);

    VHACD::Vect3 scale = bmax - bmin;
    m_scale = scale.MaxCoeff();

    m_recipScale = m_scale > double(0.0) ? double(1.0) / m_scale : double(0.0);

    {
        VHACD::VertexIndex vi = VHACD::VertexIndex(double(0.001), false);

        uint32_t dcount = 0;

        for (uint32_t i = 0; i < triangles.size() && !m_canceled; ++i)
        {
            const VHACD::Triangle& t = triangles[i];
            const VHACD::Vertex& p1 = points[t.mI0];
            const VHACD::Vertex& p2 = points[t.mI1];
            const VHACD::Vertex& p3 = points[t.mI2];

            uint32_t i1 = GetIndex(vi, p1);
            uint32_t i2 = GetIndex(vi, p2);
            uint32_t i3 = GetIndex(vi, p3);

            if (i1 == i2 || i1 == i3 || i2 == i3)
            {
                dcount++;
            }
            else
            {
                m_indices.emplace_back(i1, i2, i3);
            }
        }

        if (dcount)
        {
            if (m_params.m_logger)
            {
                char scratch[512];
                snprintf(scratch,
                    sizeof(scratch),
                    "Skipped %d degenerate triangles", dcount);
                m_params.m_logger->Log(scratch);
            }
        }

        m_vertices = vi.TakeVertices();
    }

    // Create the raycast mesh
    if (!m_canceled)
    {
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
            0,
            "Building RaycastMesh");
        m_AABBTree = VHACD::AABBTree(m_vertices,
            m_indices);
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
            100,
            "RaycastMesh completed");
    }
    if (!m_canceled)
    {
        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
            0,
            "Voxelizing Input Mesh");
        m_voxelize = VHACD::Volume();
        m_voxelize.Voxelize(m_vertices,
            m_indices,
            m_params.m_resolution,
            m_params.m_fillMode,
            m_AABBTree);
        m_voxelScale = m_voxelize.GetScale();
        m_voxelHalfScale = m_voxelScale * double(0.5);
        m_voxelBmin = m_voxelize.GetBounds().GetMin();
        m_voxelBmax = m_voxelize.GetBounds().GetMax();
        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
            100,
            "Voxelization complete");
    }

    m_inputMesh.m_vertices = m_vertices;
    m_inputMesh.m_indices = m_indices;
    if (!m_canceled)
    {
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
            0,
            "Build initial ConvexHull");
        std::unique_ptr<VoxelHull> vh = std::unique_ptr<VoxelHull>(new VoxelHull(m_voxelize,
            m_params,
            this));
        if (vh->m_convexHull)
        {
            m_overallHullVolume = vh->m_convexHull->m_volume;
        }
        m_pendingHulls.push_back(std::move(vh));
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
            100,
            "Initial ConvexHull complete");
    }
}

void VHACDImpl::ScaleOutputConvexHull(ConvexHull& ch)
{
    for (uint32_t i = 0; i < ch.m_points.size(); i++)
    {
        VHACD::Vect3 p = ch.m_points[i];
        p = (p * m_scale) + m_center;
        ch.m_points[i] = p;
    }
    ch.m_volume = ComputeConvexHullVolume(ch); // get the combined volume
    VHACD::BoundsAABB b(ch.m_points);
    ch.mBmin = b.GetMin();
    ch.mBmax = b.GetMax();
    ComputeCentroid(ch.m_points,
        ch.m_triangles,
        ch.m_center);
}

void VHACDImpl::AddCostToPriorityQueue(CostTask& task)
{
    HullPair hp(task.m_hullA->m_meshId,
        task.m_hullB->m_meshId,
        task.m_concavity);
    m_hullPairQueue.push(hp);
}

void VHACDImpl::ReleaseConvexHull(ConvexHull* ch)
{
    if (ch)
    {
        delete ch;
    }
}

void jobCallback(std::unique_ptr<VoxelHull>& userPtr)
{
    userPtr->PerformPlaneSplit();
}

void computeMergeCostTask(CostTask& ptr)
{
    ptr.m_this->PerformMergeCostTask(ptr);
}

void VHACDImpl::PerformConvexDecomposition()
{
    {
        ScopedTime st("Convex Decomposition",
            m_params.m_logger);
        double maxHulls = pow(2, m_params.m_maxRecursionDepth);
        // We recursively split convex hulls until we can
        // no longer recurse further.
        Timer t;

        while (!m_pendingHulls.empty() && !m_canceled)
        {
            size_t count = m_pendingHulls.size() + m_voxelHulls.size();
            double e = t.PeekElapsedSeconds();
            if (e >= double(0.1))
            {
                t.Reset();
                double stageProgress = (double(count) * double(100.0)) / maxHulls;
                ProgressUpdate(Stages::PERFORMING_DECOMPOSITION,
                    stageProgress,
                    "Performing recursive decomposition of convex hulls");
            }
            // First we make a copy of the hulls we are processing
            std::vector<std::unique_ptr<VoxelHull>> oldList = std::move(m_pendingHulls);
            // For each hull we want to split, we either
            // immediately perform the plane split or we post it as
            // a job to be performed in a background thread
            std::vector<std::future<void>> futures(oldList.size());
            uint32_t futureCount = 0;
            for (auto& i : oldList)
            {
                if (i->IsComplete() || count > MaxConvexHullFragments)
                {
                }
                else
                {
#if !VHACD_DISABLE_THREADING
                    if (m_threadPool)
                    {
                        futures[futureCount] = m_threadPool->enqueue([&i]
                            {
                                jobCallback(i);
                            });
                        futureCount++;
                    }
                    else
#endif
                    {
                        i->PerformPlaneSplit();
                    }
                }
            }
            // Wait for any outstanding jobs to complete in the background threads
            if (futureCount)
            {
                for (uint32_t i = 0; i < futureCount; i++)
                {
                    futures[i].get();
                }
            }
            // Now, we rebuild the pending convex hulls list by
            // adding the two children to the output list if
            // we need to recurse them further
            for (auto& vh : oldList)
            {
                if (vh->IsComplete() || count > MaxConvexHullFragments)
                {
                    if (vh->m_convexHull)
                    {
                        m_voxelHulls.push_back(std::move(vh));
                    }
                }
                else
                {
                    if (vh->m_hullA)
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullA));
                    }
                    if (vh->m_hullB)
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullB));
                    }
                }
            }
        }
    }

    if (!m_canceled)
    {
        // Give each convex hull a unique guid
        m_meshId = 0;
        m_hulls.clear();

        // Build the convex hull id map
        std::vector<ConvexHull*> hulls;

        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
            0,
            "Initializing ConvexHulls");
        for (auto& vh : m_voxelHulls)
        {
            if (m_canceled)
            {
                break;
            }
            ConvexHull* ch = CopyConvexHull(*vh->m_convexHull);
            m_meshId++;
            ch->m_meshId = m_meshId;
            m_hulls[m_meshId] = ch;
            // Compute the volume of the convex hull
            ch->m_volume = ComputeConvexHullVolume(*ch);
            // Compute the AABB of the convex hull
            VHACD::BoundsAABB b = VHACD::BoundsAABB(ch->m_points).Inflate(double(0.1));
            ch->mBmin = b.GetMin();
            ch->mBmax = b.GetMax();

            ComputeCentroid(ch->m_points,
                ch->m_triangles,
                ch->m_center);

            hulls.push_back(ch);
        }
        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
            100,
            "ConvexHull initialization complete");

        m_voxelHulls.clear();

        // here we merge convex hulls as needed until the match the
        // desired maximum hull count.
        size_t hullCount = hulls.size();

        if (hullCount > m_params.m_maxConvexHulls && !m_canceled)
        {
            size_t costMatrixSize = ((hullCount * hullCount) - hullCount) >> 1;
            std::vector<CostTask> tasks;
            tasks.reserve(costMatrixSize);

            ScopedTime st("Computing the Cost Matrix",
                m_params.m_logger);
            // First thing we need to do is compute the cost matrix
            // This is computed as the volume error of any two convex hulls
            // combined
            ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                0,
                "Computing Hull Merge Cost Matrix");
            for (size_t i = 1; i < hullCount && !m_canceled; i++)
            {
                ConvexHull* chA = hulls[i];

                for (size_t j = 0; j < i && !m_canceled; j++)
                {
                    ConvexHull* chB = hulls[j];

                    CostTask t;
                    t.m_hullA = chA;
                    t.m_hullB = chB;
                    t.m_this = this;

                    if (DoFastCost(t))
                    {
                    }
                    else
                    {
                        tasks.push_back(std::move(t));
                        CostTask* task = &tasks.back();
#if !VHACD_DISABLE_THREADING
                        if (m_threadPool)
                        {
                            task->m_future = m_threadPool->enqueue([task]
                                {
                                    computeMergeCostTask(*task);
                                });
                        }
#endif
                    }
                }
            }

            if (!m_canceled)
            {
#if !VHACD_DISABLE_THREADING
                if (m_threadPool)
                {
                    for (CostTask& task : tasks)
                    {
                        task.m_future.get();
                    }

                    for (CostTask& task : tasks)
                    {
                        AddCostToPriorityQueue(task);
                    }
                }
                else
#endif
                {
                    for (CostTask& task : tasks)
                    {
                        PerformMergeCostTask(task);
                        AddCostToPriorityQueue(task);
                    }
                }
                ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                    100,
                    "Finished cost matrix");
            }

            if (!m_canceled)
            {
                ScopedTime stMerging("Merging Convex Hulls",
                    m_params.m_logger);
                Timer t;
                // Now that we know the cost to merge each hull, we can begin merging them.
                bool cancel = false;

                uint32_t maxMergeCount = uint32_t(m_hulls.size()) - m_params.m_maxConvexHulls;
                uint32_t startCount = uint32_t(m_hulls.size());

                while (!cancel
                    && m_hulls.size() > m_params.m_maxConvexHulls
                    && !m_hullPairQueue.empty()
                    && !m_canceled)
                {
                    double e = t.PeekElapsedSeconds();
                    if (e >= double(0.1))
                    {
                        t.Reset();
                        uint32_t hullsProcessed = startCount - uint32_t(m_hulls.size());
                        double stageProgress = double(hullsProcessed * 100) / double(maxMergeCount);
                        ProgressUpdate(Stages::MERGING_CONVEX_HULLS,
                            stageProgress,
                            "Merging Convex Hulls");
                    }

                    HullPair hp = m_hullPairQueue.top();
                    m_hullPairQueue.pop();

                    // It is entirely possible that the hull pair queue can
                    // have references to convex hulls that are no longer valid
                    // because they were previously merged. So we check for this
                    // and if either hull referenced in this pair no longer
                    // exists, then we skip it.

                    // Look up this pair of hulls by ID
                    ConvexHull* ch1 = GetHull(hp.m_hullA);
                    ConvexHull* ch2 = GetHull(hp.m_hullB);

                    // If both hulls are still valid, then we merge them, delete the old
                    // two hulls and recompute the cost matrix for the new combined hull
                    // we have created
                    if (ch1 && ch2)
                    {
                        // This is the convex hull which results from combining the
                        // vertices in the two source hulls
                        ConvexHull* combinedHull = ComputeCombinedConvexHull(*ch1,
                            *ch2);
                        // The two old convex hulls are going to get removed
                        RemoveHull(hp.m_hullA);
                        RemoveHull(hp.m_hullB);

                        m_meshId++;
                        combinedHull->m_meshId = m_meshId;
                        tasks.clear();
                        tasks.reserve(m_hulls.size());

                        // Compute the cost between this new merged hull
                        // and all existing convex hulls and then
                        // add that to the priority queue
                        for (auto& i : m_hulls)
                        {
                            if (m_canceled)
                            {
                                break;
                            }
                            ConvexHull* secondHull = i.second;
                            CostTask ct;
                            ct.m_hullA = combinedHull;
                            ct.m_hullB = secondHull;
                            ct.m_this = this;
                            if (DoFastCost(ct))
                            {
                            }
                            else
                            {
                                tasks.push_back(std::move(ct));
                            }
                        }
                        m_hulls[combinedHull->m_meshId] = combinedHull;
                        // See how many merge cost tasks were posted
                        // If there are 8 or more and we are running asynchronously, then do them that way.
#if !VHACD_DISABLE_THREADING
                        if (m_threadPool && tasks.size() >= 2)
                        {
                            for (CostTask& task : tasks)
                            {
                                task.m_future = m_threadPool->enqueue([&task]
                                    {
                                        computeMergeCostTask(task);
                                    });
                            }

                            for (CostTask& task : tasks)
                            {
                                task.m_future.get();
                            }
                        }
                        else
#endif
                        {
                            for (CostTask& task : tasks)
                            {
                                PerformMergeCostTask(task);
                            }
                        }

                        for (CostTask& task : tasks)
                        {
                            AddCostToPriorityQueue(task);
                        }
                    }
                }
                // Ok...once we are done, we copy the results!
                m_meshId -= 0;
                ProgressUpdate(Stages::FINALIZING_RESULTS,
                    0,
                    "Finalizing results");
                for (auto& i : m_hulls)
                {
                    if (m_canceled)
                    {
                        break;
                    }
                    ConvexHull* ch = i.second;
                    // We now must reduce the convex hull
                    if (ch->m_points.size() > m_params.m_maxNumVerticesPerCH || m_params.m_shrinkWrap)
                    {
                        ConvexHull* reduce = ComputeReducedConvexHull(*ch,
                            m_params.m_maxNumVerticesPerCH,
                            m_params.m_shrinkWrap);
                        ReleaseConvexHull(ch);
                        ch = reduce;
                    }
                    ScaleOutputConvexHull(*ch);
                    ch->m_meshId = m_meshId;
                    m_meshId++;
                    m_convexHulls.push_back(ch);
                }
                m_hulls.clear(); // since the hulls were moved into the output list, we don't need to delete them from this container
                ProgressUpdate(Stages::FINALIZING_RESULTS,
                    100,
                    "Finalized results complete");
            }
        }
        else
        {
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                0,
                "Finalizing results");
            m_meshId = 0;
            for (auto& ch : hulls)
            {
                // We now must reduce the convex hull
                if (ch->m_points.size() > m_params.m_maxNumVerticesPerCH || m_params.m_shrinkWrap)
                {
                    ConvexHull* reduce = ComputeReducedConvexHull(*ch,
                        m_params.m_maxNumVerticesPerCH,
                        m_params.m_shrinkWrap);
                    ReleaseConvexHull(ch);
                    ch = reduce;
                }
                ScaleOutputConvexHull(*ch);
                ch->m_meshId = m_meshId;
                m_meshId++;
                m_convexHulls.push_back(ch);
            }
            m_hulls.clear();
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                100,
                "Finalized results");
        }
    }
}

double VHACDImpl::ComputeConvexHullVolume(const ConvexHull& sm)
{
    double totalVolume = 0;
    VHACD::Vect3 bary(0, 0, 0);
    for (uint32_t i = 0; i < sm.m_points.size(); i++)
    {
        VHACD::Vect3 p(sm.m_points[i]);
        bary += p;
    }
    bary /= double(sm.m_points.size());

    for (uint32_t i = 0; i < sm.m_triangles.size(); i++)
    {
        uint32_t i1 = sm.m_triangles[i].mI0;
        uint32_t i2 = sm.m_triangles[i].mI1;
        uint32_t i3 = sm.m_triangles[i].mI2;

        VHACD::Vect3 ver0(sm.m_points[i1]);
        VHACD::Vect3 ver1(sm.m_points[i2]);
        VHACD::Vect3 ver2(sm.m_points[i3]);

        totalVolume += ComputeVolume4(ver0,
            ver1,
            ver2,
            bary);

    }
    totalVolume = totalVolume / double(6.0);
    return totalVolume;
}

double VHACDImpl::ComputeVolume4(const VHACD::Vect3& a,
    const VHACD::Vect3& b,
    const VHACD::Vect3& c,
    const VHACD::Vect3& d)
{
    VHACD::Vect3 ad = a - d;
    VHACD::Vect3 bd = b - d;
    VHACD::Vect3 cd = c - d;
    VHACD::Vect3 bcd = bd.Cross(cd);
    double dot = ad.Dot(bcd);
    return dot;
}

double VHACDImpl::ComputeConcavity(double volumeSeparate,
    double volumeCombined,
    double volumeMesh)
{
    return fabs(volumeSeparate - volumeCombined) / volumeMesh;
}

bool VHACDImpl::DoFastCost(CostTask& mt)
{
    bool ret = false;

    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    VHACD::BoundsAABB ch1b(ch1->mBmin,
        ch1->mBmax);
    VHACD::BoundsAABB ch2b(ch2->mBmin,
        ch2->mBmax);
    if (!ch1b.Intersects(ch2b))
    {
        VHACD::BoundsAABB b = ch1b.Union(ch2b);

        double combinedVolume = b.Volume();
        double concavity = ComputeConcavity(ch1->m_volume + ch2->m_volume,
            combinedVolume,
            m_overallHullVolume);
        HullPair hp(ch1->m_meshId,
            ch2->m_meshId,
            concavity);
        m_hullPairQueue.push(hp);
        ret = true;
    }
    return ret;
}

void VHACDImpl::PerformMergeCostTask(CostTask& mt)
{
    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    double volume1 = ch1->m_volume;
    double volume2 = ch2->m_volume;

    ConvexHull* combined = ComputeCombinedConvexHull(*ch1,
        *ch2); // Build the combined convex hull
    double combinedVolume = ComputeConvexHullVolume(*combined); // get the combined volume
    mt.m_concavity = ComputeConcavity(volume1 + volume2,
        combinedVolume,
        m_overallHullVolume);
    ReleaseConvexHull(combined);
}

IVHACD::ConvexHull* VHACDImpl::ComputeReducedConvexHull(const ConvexHull& ch,
    uint32_t maxVerts,
    bool projectHullVertices)
{
    SimpleMesh sourceConvexHull;

    sourceConvexHull.m_vertices = ch.m_points;
    sourceConvexHull.m_indices = ch.m_triangles;

    ShrinkWrap(sourceConvexHull,
        m_AABBTree,
        maxVerts,
        m_voxelScale * 4,
        projectHullVertices);

    ConvexHull* ret = new ConvexHull;

    ret->m_points = sourceConvexHull.m_vertices;
    ret->m_triangles = sourceConvexHull.m_indices;

    VHACD::BoundsAABB b = VHACD::BoundsAABB(ret->m_points).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();
    ComputeCentroid(ret->m_points,
        ret->m_triangles,
        ret->m_center);

    ret->m_volume = ComputeConvexHullVolume(*ret);

    // Return the convex hull
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::ComputeCombinedConvexHull(const ConvexHull& sm1,
    const ConvexHull& sm2)
{
    uint32_t vcount = uint32_t(sm1.m_points.size() + sm2.m_points.size()); // Total vertices from both hulls
    std::vector<VHACD::Vertex> vertices(vcount);
    auto it = std::copy(sm1.m_points.begin(),
        sm1.m_points.end(),
        vertices.begin());
    std::copy(sm2.m_points.begin(),
        sm2.m_points.end(),
        it);

    VHACD::QuickHull qh;
    qh.ComputeConvexHull(vertices,
        vcount);

    ConvexHull* ret = new ConvexHull;
    ret->m_points = qh.GetVertices();
    ret->m_triangles = qh.GetIndices();

    ret->m_volume = ComputeConvexHullVolume(*ret);

    VHACD::BoundsAABB b = VHACD::BoundsAABB(qh.GetVertices()).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();
    ComputeCentroid(ret->m_points,
        ret->m_triangles,
        ret->m_center);

    // Return the convex hull
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::GetHull(uint32_t index)
{
    ConvexHull* ret = nullptr;

    auto found = m_hulls.find(index);
    if (found != m_hulls.end())
    {
        ret = found->second;
    }

    return ret;
}

bool VHACDImpl::RemoveHull(uint32_t index)
{
    bool ret = false;
    auto found = m_hulls.find(index);
    if (found != m_hulls.end())
    {
        ret = true;
        ReleaseConvexHull(found->second);
        m_hulls.erase(found);
    }
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::CopyConvexHull(const ConvexHull& source)
{
    ConvexHull* ch = new ConvexHull;
    *ch = source;

    return ch;
}

const char* VHACDImpl::GetStageName(Stages stage) const
{
    const char* ret = "unknown";
    switch (stage)
    {
    case Stages::COMPUTE_BOUNDS_OF_INPUT_MESH:
        ret = "COMPUTE_BOUNDS_OF_INPUT_MESH";
        break;
    case Stages::REINDEXING_INPUT_MESH:
        ret = "REINDEXING_INPUT_MESH";
        break;
    case Stages::CREATE_RAYCAST_MESH:
        ret = "CREATE_RAYCAST_MESH";
        break;
    case Stages::VOXELIZING_INPUT_MESH:
        ret = "VOXELIZING_INPUT_MESH";
        break;
    case Stages::BUILD_INITIAL_CONVEX_HULL:
        ret = "BUILD_INITIAL_CONVEX_HULL";
        break;
    case Stages::PERFORMING_DECOMPOSITION:
        ret = "PERFORMING_DECOMPOSITION";
        break;
    case Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING:
        ret = "INITIALIZING_CONVEX_HULLS_FOR_MERGING";
        break;
    case Stages::COMPUTING_COST_MATRIX:
        ret = "COMPUTING_COST_MATRIX";
        break;
    case Stages::MERGING_CONVEX_HULLS:
        ret = "MERGING_CONVEX_HULLS";
        break;
    case Stages::FINALIZING_RESULTS:
        ret = "FINALIZING_RESULTS";
        break;
    case Stages::NUM_STAGES:
        // Should be unreachable, here to silence enumeration value not handled in switch warnings
        // GCC/Clang's -Wswitch
        break;
    }
    return ret;
}

void VHACDImpl::ProgressUpdate(Stages stage,
    double stageProgress,
    const char* operation)
{
    if (m_params.m_callback)
    {
        double overallProgress = (double(stage) * 100) / double(Stages::NUM_STAGES);
        const char* s = GetStageName(stage);
        m_params.m_callback->Update(overallProgress,
            stageProgress,
            s,
            operation);
    }
}

bool VHACDImpl::IsCanceled() const
{
    return m_canceled;
}