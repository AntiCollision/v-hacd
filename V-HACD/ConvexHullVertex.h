#pragma once
class ConvexHullVertex : public VHACD::Vect3
{
public:
    ConvexHullVertex() = default;
    ConvexHullVertex(const ConvexHullVertex&) = default;
    ConvexHullVertex& operator=(const ConvexHullVertex& rhs) = default;
    using VHACD::Vect3::operator=;

    int m_mark{ 0 };
};