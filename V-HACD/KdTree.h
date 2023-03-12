#pragma once

class KdTreeNode;
class KdTreeFindNode;


class KdTree
{
public:
    KdTree() = default;

    const VHACD::Vertex& GetPosition(uint32_t index) const;

    uint32_t Search(const VHACD::Vect3& pos,
        double radius,
        uint32_t maxObjects,
        KdTreeFindNode* found) const;

    uint32_t Add(const VHACD::Vertex& v);

    KdTreeNode& GetNewNode(uint32_t index);

    uint32_t GetNearest(const VHACD::Vect3& pos,
        double radius,
        bool& _found) const; // returns the nearest possible neighbor's index.

    const std::vector<VHACD::Vertex>& GetVertices() const;
    std::vector<VHACD::Vertex>&& TakeVertices();

    uint32_t GetVCount() const;

private:
    KdTreeNode* m_root{ nullptr };
    NodeBundle<KdTreeNode> m_bundle;

    std::vector<VHACD::Vertex> m_vertices;
};