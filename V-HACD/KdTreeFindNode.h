#pragma once

class KdTreeNode;

class KdTreeFindNode
{
public:
    KdTreeFindNode() = default;

    KdTreeNode* m_node{ nullptr };
    double m_distance{ 0.0 };
};
