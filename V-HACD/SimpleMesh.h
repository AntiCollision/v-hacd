#pragma once

struct SimpleMesh
{
    std::vector<VHACD::Vertex> m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};