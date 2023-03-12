#pragma once

class ConvexHullAABBTreeNode
{
#define VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE 8
public:
    ConvexHullAABBTreeNode() = default;
    ConvexHullAABBTreeNode(ConvexHullAABBTreeNode* parent);

    VHACD::Vect3 m_box[2];
    ConvexHullAABBTreeNode* m_left{ nullptr };
    ConvexHullAABBTreeNode* m_right{ nullptr };
    ConvexHullAABBTreeNode* m_parent{ nullptr };

    size_t m_count;
    std::array<size_t, VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE> m_indices;
};

ConvexHullAABBTreeNode::ConvexHullAABBTreeNode(ConvexHullAABBTreeNode* parent)
    : m_parent(parent)
{
}

