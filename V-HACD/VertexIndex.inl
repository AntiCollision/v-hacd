#pragma once

VertexIndex::VertexIndex(double granularity,
    bool snapToGrid)
    : m_snapToGrid(snapToGrid)
    , m_granularity(granularity)
{
}

VHACD::Vect3 VertexIndex::SnapToGrid(VHACD::Vect3 p)
{
    for (int i = 0; i < 3; ++i)
    {
        double m = fmod(p[i], m_granularity);
        p[i] -= m;
    }
    return p;
}

uint32_t VertexIndex::GetIndex(VHACD::Vect3 p,
    bool& newPos)
{
    uint32_t ret;

    newPos = false;

    if (m_snapToGrid)
    {
        p = SnapToGrid(p);
    }

    bool found;
    ret = m_KdTree.GetNearest(p, m_granularity, found);
    if (!found)
    {
        newPos = true;
        ret = m_KdTree.Add(VHACD::Vertex(p.GetX(), p.GetY(), p.GetZ()));
    }

    return ret;
}

const std::vector<VHACD::Vertex>& VertexIndex::GetVertices() const
{
    return m_KdTree.GetVertices();
}

std::vector<VHACD::Vertex>&& VertexIndex::TakeVertices()
{
    return std::move(m_KdTree.TakeVertices());
}

uint32_t VertexIndex::GetVCount() const
{
    return m_KdTree.GetVCount();
}