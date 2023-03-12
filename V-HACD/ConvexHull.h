#pragma once
#include <vector>
#include <list>


class ConvexHull
{
    class ndNormalMap;

public:
    ConvexHull(const ConvexHull& source);
    ConvexHull(const std::vector<::VHACD::Vertex>& vertexCloud,
        double distTol,
        int maxVertexCount = 0x7fffffff);
    ~ConvexHull() = default;

    const std::vector<VHACD::Vect3>& GetVertexPool() const;

    const std::list<ConvexHullFace>& GetList() const { return m_list; }

private:
    void BuildHull(const std::vector<::VHACD::Vertex>& vertexCloud,
        double distTol,
        int maxVertexCount);

    void GetUniquePoints(std::vector<ConvexHullVertex>& points);
    int InitVertexArray(std::vector<ConvexHullVertex>& points,
        NodeBundle<ConvexHullAABBTreeNode>& memoryPool);

    ConvexHullAABBTreeNode* BuildTreeNew(std::vector<ConvexHullVertex>& points,
        std::vector<ConvexHullAABBTreeNode>& memoryPool) const;
    ConvexHullAABBTreeNode* BuildTreeOld(std::vector<ConvexHullVertex>& points,
        NodeBundle<ConvexHullAABBTreeNode>& memoryPool);
    ConvexHullAABBTreeNode* BuildTreeRecurse(ConvexHullAABBTreeNode* const parent,
        ConvexHullVertex* const points,
        int count,
        int baseIndex,
        NodeBundle<ConvexHullAABBTreeNode>& memoryPool) const;

    std::list<ConvexHullFace>::iterator AddFace(int i0,
        int i1,
        int i2);

    void CalculateConvexHull3D(ConvexHullAABBTreeNode* vertexTree,
        std::vector<ConvexHullVertex>& points,
        int count,
        double distTol,
        int maxVertexCount);

    int SupportVertex(ConvexHullAABBTreeNode** const tree,
        const std::vector<ConvexHullVertex>& points,
        const VHACD::Vect3& dir,
        const bool removeEntry = true) const;
    double TetrahedrumVolume(const VHACD::Vect3& p0,
        const VHACD::Vect3& p1,
        const VHACD::Vect3& p2,
        const VHACD::Vect3& p3) const;

    std::list<ConvexHullFace> m_list;
    VHACD::Vect3 m_aabbP0{ 0 };
    VHACD::Vect3 m_aabbP1{ 0 };
    double m_diag{ 0.0 };
    std::vector<VHACD::Vect3> m_points;
};

