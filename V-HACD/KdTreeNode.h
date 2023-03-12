#pragma once

class KdTreeNode
{
public:
    KdTreeNode() = default;
    KdTreeNode(uint32_t index);

    void Add(KdTreeNode& node,
        Axes dim,
        const KdTree& iface);

    uint32_t GetIndex() const;

    void Search(Axes axis,
        const VHACD::Vect3& pos,
        double radius,
        uint32_t& count,
        uint32_t maxObjects,
        KdTreeFindNode* found,
        const KdTree& iface);

private:
    uint32_t m_index = 0;
    KdTreeNode* m_left = nullptr;
    KdTreeNode* m_right = nullptr;
};

