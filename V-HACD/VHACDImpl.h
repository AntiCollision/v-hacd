#pragma once

class VHACDImpl : public IVHACD, public VHACDCallbacks
{
    // Don't consider more than 100,000 convex hulls.
    static constexpr uint32_t MaxConvexHullFragments{ 100000 };
public:
    VHACDImpl() = default;

    /*
     * Overrides VHACD::IVHACD
     */
    ~VHACDImpl() override
    {
        Clean();
    }

    void Cancel() override final;

    bool Compute(const float* const points,
        const uint32_t countPoints,
        const uint32_t* const triangles,
        const uint32_t countTriangles,
        const Parameters& params) override final;

    bool Compute(const double* const points,
        const uint32_t countPoints,
        const uint32_t* const triangles,
        const uint32_t countTriangles,
        const Parameters& params) override final;

    uint32_t GetNConvexHulls() const override final;

    bool GetConvexHull(const uint32_t index,
        ConvexHull& ch) const override final;

    void Clean() override final;  // release internally allocated memory

    void Release() override final;

    // Will compute the center of mass of the convex hull decomposition results and return it
    // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
    bool ComputeCenterOfMass(double centerOfMass[3]) const override final;

    // In synchronous mode (non-multi-threaded) the state is always 'ready'
    // In asynchronous mode, this returns true if the background thread is not still actively computing
    // a new solution.  In an asynchronous config the 'IsReady' call will report any update or log
    // messages in the caller's current thread.
    bool IsReady(void) const override final;

    /**
    * At the request of LegionFu : out_look@foxmail.com
    * This method will return which convex hull is closest to the source position.
    * You can use this method to figure out, for example, which vertices in the original
    * source mesh are best associated with which convex hull.
    *
    * @param pos : The input 3d position to test against
    *
    * @return : Returns which convex hull this position is closest to.
    */
    uint32_t findNearestConvexHull(const double pos[3],
        double& distanceToHull) override final;

    // private:
    bool Compute(const std::vector<VHACD::Vertex>& points,
        const std::vector<VHACD::Triangle>& triangles,
        const Parameters& params);

    // Take the source position, normalize it, and then convert it into an index position
    uint32_t GetIndex(VHACD::VertexIndex& vi,
        const VHACD::Vertex& p);

    // This copies the input mesh while scaling the input positions
    // to fit into a normalized unit cube. It also re-indexes all of the
    // vertex positions in case they weren't clean coming in. 
    void CopyInputMesh(const std::vector<VHACD::Vertex>& points,
        const std::vector<VHACD::Triangle>& triangles);

    void ScaleOutputConvexHull(ConvexHull& ch);

    void AddCostToPriorityQueue(CostTask& task);

    void ReleaseConvexHull(ConvexHull* ch);

    void PerformConvexDecomposition();

    double ComputeConvexHullVolume(const ConvexHull& sm);

    double ComputeVolume4(const VHACD::Vect3& a,
        const VHACD::Vect3& b,
        const VHACD::Vect3& c,
        const VHACD::Vect3& d);

    double ComputeConcavity(double volumeSeparate,
        double volumeCombined,
        double volumeMesh);

    // See if we can compute the cost without having to actually merge convex hulls.
    // If the axis aligned bounding boxes (slightly inflated) of the two convex hulls
    // do not intersect, then we don't need to actually compute the merged convex hull
    // volume.
    bool DoFastCost(CostTask& mt);

    void PerformMergeCostTask(CostTask& mt);

    ConvexHull* ComputeReducedConvexHull(const ConvexHull& ch,
        uint32_t maxVerts,
        bool projectHullVertices);

    // Take the points in convex hull A and the points in convex hull B and generate
    // a new convex hull on the combined set of points.
    // Once completed, we create a SimpleMesh instance to hold the triangle mesh
    // and we compute an inflated AABB for it.
    ConvexHull* ComputeCombinedConvexHull(const ConvexHull& sm1,
        const ConvexHull& sm2);


    ConvexHull* GetHull(uint32_t index);

    bool RemoveHull(uint32_t index);

    ConvexHull* CopyConvexHull(const ConvexHull& source);

    const char* GetStageName(Stages stage) const;

    /*
     * Overrides VHACD::VHACDCallbacks
     */
    void ProgressUpdate(Stages stage,
        double stageProgress,
        const char* operation) override final;

    bool IsCanceled() const override final;

    std::atomic<bool>                                   m_canceled{ false };
    Parameters                                          m_params; // Convex decomposition parameters

    std::vector<IVHACD::ConvexHull*>                    m_convexHulls; // Finalized convex hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_voxelHulls; // completed voxel hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_pendingHulls;

    std::vector<std::unique_ptr<AABBTree>>              m_trees;
    VHACD::AABBTree                                     m_AABBTree;
    VHACD::Volume                                       m_voxelize;
    VHACD::Vect3                                        m_center;
    double                                              m_scale{ double(1.0) };
    double                                              m_recipScale{ double(1.0) };
    SimpleMesh                                          m_inputMesh; // re-indexed and normalized input mesh
    std::vector<VHACD::Vertex>                          m_vertices;
    std::vector<VHACD::Triangle>                        m_indices;

    double                                              m_overallHullVolume{ double(0.0) };
    double                                              m_voxelScale{ double(0.0) };
    double                                              m_voxelHalfScale{ double(0.0) };
    VHACD::Vect3                                        m_voxelBmin;
    VHACD::Vect3                                        m_voxelBmax;
    uint32_t                                            m_meshId{ 0 };
    std::priority_queue<HullPair>                       m_hullPairQueue;
#if !VHACD_DISABLE_THREADING
    std::unique_ptr<ThreadPool>                         m_threadPool{ nullptr };
#endif
    std::unordered_map<uint32_t, IVHACD::ConvexHull*>   m_hulls;

    double                                              m_overallProgress{ double(0.0) };
    double                                              m_stageProgress{ double(0.0) };
    double                                              m_operationProgress{ double(0.0) };
};