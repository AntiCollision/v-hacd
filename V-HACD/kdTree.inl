#pragma once

const VHACD::Vertex& KdTree::GetPosition(uint32_t index) const
{
    assert(index < m_vertices.size());
    return m_vertices[index];
}

uint32_t KdTree::Search(const VHACD::Vect3& pos,
    double radius,
    uint32_t maxObjects,
    KdTreeFindNode* found) const
{
    if (!m_root)
        return 0;
    uint32_t count = 0;
    m_root->Search(X_AXIS, pos, radius, count, maxObjects, found, *this);
    return count;
}

uint32_t KdTree::Add(const VHACD::Vertex& v)
{
    uint32_t ret = uint32_t(m_vertices.size());
    m_vertices.emplace_back(v);
    KdTreeNode& node = GetNewNode(ret);
    if (m_root)
    {
        m_root->Add(node,
            X_AXIS,
            *this);
    }
    else
    {
        m_root = &node;
    }
    return ret;
}

KdTreeNode& KdTree::GetNewNode(uint32_t index)
{
    KdTreeNode& node = m_bundle.GetNextNode();
    node = KdTreeNode(index);
    return node;
}

uint32_t KdTree::GetNearest(const VHACD::Vect3& pos,
    double radius,
    bool& _found) const // returns the nearest possible neighbor's index.
{
    uint32_t ret = 0;

    _found = false;
    KdTreeFindNode found;
    uint32_t count = Search(pos, radius, 1, &found);
    if (count)
    {
        KdTreeNode* node = found.m_node;
        ret = node->GetIndex();
        _found = true;
    }
    return ret;
}

const std::vector<VHACD::Vertex>& KdTree::GetVertices() const
{
    return m_vertices;
}

std::vector<VHACD::Vertex>&& KdTree::TakeVertices()
{
    return std::move(m_vertices);
}

uint32_t KdTree::GetVCount() const
{
    return uint32_t(m_vertices.size());
}