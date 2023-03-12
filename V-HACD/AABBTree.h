#pragma once

class AABBTree
{
public:
    AABBTree() = default;
    AABBTree(AABBTree&&) = default;
    AABBTree& operator=(AABBTree&&) = default;

    AABBTree(const std::vector<VHACD::Vertex>& vertices,
        const std::vector<VHACD::Triangle>& indices);

    bool TraceRay(const VHACD::Vect3& start,
        const VHACD::Vect3& to,
        double& outT,
        double& faceSign,
        VHACD::Vect3& hitLocation) const;

    bool TraceRay(const VHACD::Vect3& start,
        const VHACD::Vect3& dir,
        uint32_t& insideCount,
        uint32_t& outsideCount) const;

    bool TraceRay(const VHACD::Vect3& start,
        const VHACD::Vect3& dir,
        double& outT,
        double& u,
        double& v,
        double& w,
        double& faceSign,
        uint32_t& faceIndex) const;

    VHACD::Vect3 GetCenter() const;
    VHACD::Vect3 GetMinExtents() const;
    VHACD::Vect3 GetMaxExtents() const;

    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
        double maxDistance,
        VHACD::Vect3& closestPoint) const;

private:
    struct Node
    {
        union
        {
            uint32_t m_children;
            uint32_t m_numFaces{ 0 };
        };

        uint32_t* m_faces{ nullptr };
        VHACD::BoundsAABB m_extents;
    };

    struct FaceSorter
    {
        FaceSorter(const std::vector<VHACD::Vertex>& positions,
            const std::vector<VHACD::Triangle>& indices,
            uint32_t axis);

        bool operator()(uint32_t lhs, uint32_t rhs) const;

        double GetCentroid(uint32_t face) const;

        const std::vector<VHACD::Vertex>& m_vertices;
        const std::vector<VHACD::Triangle>& m_indices;
        uint32_t m_axis;
    };

    // partition the objects and return the number of objects in the lower partition
    uint32_t PartitionMedian(Node& n,
        uint32_t* faces,
        uint32_t numFaces);
    uint32_t PartitionSAH(Node& n,
        uint32_t* faces,
        uint32_t numFaces);

    void Build();

    void BuildRecursive(uint32_t nodeIndex,
        uint32_t* faces,
        uint32_t numFaces);

    void TraceRecursive(uint32_t nodeIndex,
        const VHACD::Vect3& start,
        const VHACD::Vect3& dir,
        double& outT,
        double& u,
        double& v,
        double& w,
        double& faceSign,
        uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
        const double maxDis,
        double& dis,
        double& v,
        double& w,
        uint32_t& faceIndex,
        VHACD::Vect3& closest) const;

    void GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
        const VHACD::Vect3& point,
        double& outDisSq,
        double& outV,
        double& outW,
        uint32_t& outFaceIndex,
        VHACD::Vect3& closest) const;

    VHACD::BoundsAABB CalculateFaceBounds(uint32_t* faces,
        uint32_t numFaces);

    // track the next free node
    uint32_t m_freeNode;

    const std::vector<VHACD::Vertex>* m_vertices{ nullptr };
    const std::vector<VHACD::Triangle>* m_indices{ nullptr };

    std::vector<uint32_t> m_faces;
    std::vector<Node> m_nodes;
    std::vector<VHACD::BoundsAABB> m_faceBounds;

    // stats
    uint32_t m_treeDepth{ 0 };
    uint32_t m_innerNodes{ 0 };
    uint32_t m_leafNodes{ 0 };

    uint32_t s_depth{ 0 };
};