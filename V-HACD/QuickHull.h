#pragma once

//******************************************************************************************
//  ShrinkWrap helper class
//******************************************************************************************
// This is a code snippet which 'shrinkwraps' a convex hull
// to a source mesh.
//
// It is a somewhat complicated algorithm. It works as follows:
//
// * Step #1 : Compute the mean unit normal vector for each vertex in the convex hull
// * Step #2 : For each vertex in the conex hull we project is slightly outwards along the mean normal vector
// * Step #3 : We then raycast from this slightly extruded point back into the opposite direction of the mean normal vector
//             resulting in a raycast from slightly beyond the vertex in the hull into the source mesh we are trying
//             to 'shrink wrap' against
// * Step #4 : If the raycast fails we leave the original vertex alone
// * Step #5 : If the raycast hits a backface we leave the original vertex alone
// * Step #6 : If the raycast hits too far away (no more than a certain threshold distance) we live it alone
// * Step #7 : If the point we hit on the source mesh is not still within the convex hull, we reject it.
// * Step #8 : If all of the previous conditions are met, then we take the raycast hit location as the 'new position'
// * Step #9 : Once all points have been projected, if possible, we need to recompute the convex hull again based on these shrinkwrapped points
// * Step #10 : In theory that should work.. let's see...

//***********************************************************************************************
// QuickHull implementation
//***********************************************************************************************

//////////////////////////////////////////////////////////////////////////
// Quickhull base class holding the hull during construction
//////////////////////////////////////////////////////////////////////////
class QuickHull
{
public:
    uint32_t ComputeConvexHull(const std::vector<VHACD::Vertex>& vertices,
        uint32_t maxHullVertices)
    {
        m_indices.clear();

        VHACD::ConvexHull ch(vertices,
            double(0.0001),
            maxHullVertices);

        auto& vlist = ch.GetVertexPool();
        if (!vlist.empty())
        {
            size_t vcount = vlist.size();
            m_vertices.resize(vcount);
            std::copy(vlist.begin(),
                vlist.end(),
                m_vertices.begin());
        }

        for (std::list<ConvexHullFace>::const_iterator node = ch.GetList().begin(); node != ch.GetList().end(); ++node)
        {
            const VHACD::ConvexHullFace& face = *node;
            m_indices.emplace_back(face.m_index[0],
                face.m_index[1],
                face.m_index[2]);
        }

        return uint32_t(m_indices.size());
    }


    const std::vector<VHACD::Vertex>& GetVertices() const {
        return m_vertices;
    }
    const std::vector<VHACD::Triangle>& GetIndices() const
    {
        return m_indices;
    }

private:
    std::vector<VHACD::Vertex>   m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};

