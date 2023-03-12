/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#ifndef VHACD_H
#    define VHACD_H

// Please view this slide deck which describes usage and how the algorithm works.
// https://docs.google.com/presentation/d/1OZ4mtZYrGEC8qffqb8F7Le2xzufiqvaPpRbLHKKgTIM/edit?usp=sharing

// VHACD is now a header only library.
// In just *one* of your CPP files *before* you include 'VHACD.h' you must declare
// #define ENABLE_VHACD_IMPLEMENTATION 1
// This will compile the implementation code into your project. If you don't
// have this define, you will get link errors since the implementation code will
// not be present. If you define it more than once in your code base, you will get
// link errors due to a duplicate implementation. This is the same pattern used by
// ImGui and StbLib and other popular open source libraries.

#    define VHACD_VERSION_MAJOR 4
#    define VHACD_VERSION_MINOR 1

// Changes for version 4.1
//
// Various minor tweaks mostly to the test application and some default values.

// Changes for version 4.0
//
// * The code has been significantly refactored to be cleaner and easier to maintain
//      * All OpenCL related code removed
//      * All Bullet code removed
//      * All SIMD code removed
//      * Old plane splitting code removed
// 
// * The code is now delivered as a single header file 'VHACD.h' which has both the API
// * declaration as well as the implementation.  Simply add '#define ENABLE_VHACD_IMPLEMENTATION 1'
// * to any CPP in your application prior to including 'VHACD.h'. Only do this in one CPP though.
// * If you do not have this define once, you will get link errors since the implementation code
// * will not be compiled in. If you have this define more than once, you are likely to get
// * duplicate symbol link errors.
//
// * Since the library is now delivered as a single header file, we do not provide binaries
// * or build scripts as these are not needed.
//
// * The old DebugView and test code has all been removed and replaced with a much smaller and
// * simpler test console application with some test meshes to work with.
//
// * The convex hull generation code has changed. The previous version came from Bullet. 
// * However, the new version is courtesy of Julio Jerez, the author of the Newton
// * physics engine. His new version is faster and more numerically stable.
//
// * The code can now detect if the input mesh is, itself, already a convex object and
// * can early out.
//
// * Significant performance improvements have been made to the code and it is now much
// * faster, stable, and is easier to tune than previous versions.
//
// * A bug was fixed with the shrink wrapping code (project hull vertices) that could
// * sometime produce artifacts in the results. The new version uses a 'closest point'
// * algorithm that is more reliable.
//
// * You can now select which 'fill mode' to use. For perfectly closed meshes, the default
// * behavior using a flood fill generally works fine. However, some meshes have small 
// * holes in them and therefore the flood fill will fail, treating the mesh as being
// * hollow. In these cases, you can use the 'raycast' fill option to determine which 
// * parts of the voxelized mesh are 'inside' versus being 'outside'. Finally, there
// * are some rare instances where a user might actually want the mesh to be treated as
// * hollow, in which case you can pass in 'surface' only.
// *
// * A new optional virtual interface called 'IUserProfiler' was provided.
// * This allows the user to provide an optional profiling callback interface to assist in
// * diagnosing performance issues. This change was made by Danny Couture at Epic for the UE4 integration.
// * Some profiling macros were also declared in support of this feature.
// *
// * Another new optional virtual interface called 'IUserTaskRunner' was provided.
// * This interface is used to run logical 'tasks' in a background thread. If none is provided
// * then a default implementation using std::thread will be executed.
// * This change was made by Danny Couture at Epic to speed up the voxelization step.
// *



// The history of V-HACD:
//
// The initial version was written by John W. Ratcliff and was called 'ACD'
// This version did not perform CSG operations on the source mesh, so if you 
// recursed too deeply it would produce hollow results.
//
// The next version was written by Khaled Mamou and was called 'HACD'
// In this version Khaled tried to perform a CSG operation on the source 
// mesh to produce more robust results. However, Khaled learned that the
// CSG library he was using had licensing issues so he started work on the
// next version.
//
// The next version was called 'V-HACD' because Khaled made the observation
// that plane splitting would be far easier to implement working in voxel space.
// 
// V-HACD has been integrated into UE4, Blender, and a number of other projects.
// This new release, version4, is a significant refactor of the code to fix
// some bugs, improve performance, and to make the codebase easier to maintain
// going forward.

#include <stdint.h>
#include <functional>

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

#include "Vertex.h"
#include "Triangle.h"
#include "Vector3.h"


namespace VHACD {
    typedef VHACD::Vector3<double> Vect3;

    /*
     * Out of line definitions
     */
    template <typename T>
    T clamp(const T& v, const T& lo, const T& hi)
    {
        if (v < lo)
        {
            return lo;
        }
        if (v > hi)
        {
            return hi;
        }
        return v ;
    }

	//IVHACD* CreateVHACD();      // Create a synchronous (blocking) implementation of V-HACD
	//IVHACD* CreateVHACD_ASYNC();    // Create an asynchronous (non-blocking) implementation of V-HACD

} // namespace VHACD

#if ENABLE_VHACD_IMPLEMENTATION
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "BoundsAABB.h"
#include "IVHACD.h"
#include "VScopedTime.h"
#include "Timer.h"
#include "NodeBundle.h"
#include "Googol.h"
#include "HullPlane.h"


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4100 4127 4189 4244 4456 4701 4702 4996)
#endif // _MSC_VER

#ifdef __GNUC__
#pragma GCC diagnostic push
// Minimum set of warnings used for cleanup
// #pragma GCC diagnostic warning "-Wall"
// #pragma GCC diagnostic warning "-Wextra"
// #pragma GCC diagnostic warning "-Wpedantic"
// #pragma GCC diagnostic warning "-Wold-style-cast"
// #pragma GCC diagnostic warning "-Wnon-virtual-dtor"
// #pragma GCC diagnostic warning "-Wshadow"
#endif // __GNUC__

// Scoped Timer
namespace VHACD {

/*
 * Relies on three way comparison, which std::sort doesn't use
 */
template <class T, class dCompareKey>
void Sort(T* const array, int elements)
{
    const int batchSize = 8;
    int stack[1024][2];

    stack[0][0] = 0;
    stack[0][1] = elements - 1;
    int stackIndex = 1;
    const dCompareKey comparator;
    while (stackIndex)
    {
        stackIndex--;
        int lo = stack[stackIndex][0];
        int hi = stack[stackIndex][1];
        if ((hi - lo) > batchSize)
        {
            int mid = (lo + hi) >> 1;
            if (comparator.Compare(array[lo], array[mid]) > 0)
            {
                std::swap(array[lo],
                          array[mid]);
            }
            if (comparator.Compare(array[mid], array[hi]) > 0)
            {
                std::swap(array[mid],
                          array[hi]);
            }
            if (comparator.Compare(array[lo], array[mid]) > 0)
            {
                std::swap(array[lo],
                          array[mid]);
            }
            int i = lo + 1;
            int j = hi - 1;
            const T pivot(array[mid]);
            do
            {
                while (comparator.Compare(array[i], pivot) < 0)
                {
                    i++;
                }
                while (comparator.Compare(array[j], pivot) > 0)
                {
                    j--;
                }

                if (i <= j)
                {
                    std::swap(array[i],
                              array[j]);
                    i++;
                    j--;
                }
            } while (i <= j);

            if (i < hi)
            {
                stack[stackIndex][0] = i;
                stack[stackIndex][1] = hi;
                stackIndex++;
            }
            if (lo < j)
            {
                stack[stackIndex][0] = lo;
                stack[stackIndex][1] = j;
                stackIndex++;
            }
            assert(stackIndex < int(sizeof(stack) / (2 * sizeof(stack[0][0]))));
        }
    }

    int stride = batchSize + 1;
    if (elements < stride)
    {
        stride = elements;
    }
    for (int i = 1; i < stride; ++i)
    {
        if (comparator.Compare(array[0], array[i]) > 0)
        {
            std::swap(array[0],
                      array[i]);
        }
    }

    for (int i = 1; i < elements; ++i)
    {
        int j = i;
        const T tmp(array[i]);
        for (; comparator.Compare(array[j - 1], tmp) > 0; --j)
        {
            assert(j > 0);
            array[j] = array[j - 1];
        }
        array[j] = tmp;
    }
}

/*
Maintaining comment due to attribution
Purpose:

TRIANGLE_AREA_3D computes the area of a triangle in 3D.

Modified:

22 April 1999

Author:

John Burkardt

Parameters:

Input, double X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3, the (getX,getY,getZ)
coordinates of the corners of the triangle.

Output, double TRIANGLE_AREA_3D, the area of the triangle.
*/
double ComputeArea(const VHACD::Vect3& p1,
                   const VHACD::Vect3& p2,
                   const VHACD::Vect3& p3)
{
    /*
    Find the projection of (P3-P1) onto (P2-P1).
    */
    double base = (p2 - p1).GetNorm();
    /*
    The height of the triangle is the length of (P3-P1) after its
    projection onto (P2-P1) has been subtracted.
    */
    double height;
    if (base == double(0.0))
    {
        height = double(0.0);
    }
    else
    {
        double dot = (p3 - p1).Dot(p2 - p1);
        double alpha = dot / (base * base);

        VHACD::Vect3 a = p3 - p1 - alpha * (p2 - p1);
        height = a.GetNorm();
    }

    return double(0.5) * base * height;
}

bool ComputeCentroid(const std::vector<VHACD::Vertex>& points,
                     const std::vector<VHACD::Triangle>& indices,
                     VHACD::Vect3& center)

{
    bool ret = false;
    if (points.size())
    {
        center = VHACD::Vect3(0);

        VHACD::Vect3 numerator(0);
        double denominator = 0;

        for (uint32_t i = 0; i < indices.size(); i++)
        {
            uint32_t i1 = indices[i].mI0;
            uint32_t i2 = indices[i].mI1;
            uint32_t i3 = indices[i].mI2;

            const VHACD::Vect3& p1 = points[i1];
            const VHACD::Vect3& p2 = points[i2];
            const VHACD::Vect3& p3 = points[i3];

            // Compute the average of the sum of the three positions
            VHACD::Vect3 sum = (p1 + p2 + p3) / 3;

            // Compute the area of this triangle
            double area = ComputeArea(p1,
                                      p2,
                                      p3);

            numerator += (sum * area);

            denominator += area;
        }
        double recip = 1 / denominator;
        center = numerator * recip;
        ret = true;
    }
    return ret;
}

double Determinant3x3(const std::array<VHACD::Vect3, 3>& matrix,
                      double& error)
{
    double det = double(0.0);
    error = double(0.0);

    double a01xa12 = matrix[0].GetY() * matrix[1].GetZ();
    double a02xa11 = matrix[0].GetZ() * matrix[1].GetY();
    error += (std::abs(a01xa12) + std::abs(a02xa11)) * std::abs(matrix[2].GetX());
    det += (a01xa12 - a02xa11) * matrix[2].GetX();

    double a00xa12 = matrix[0].GetX() * matrix[1].GetZ();
    double a02xa10 = matrix[0].GetZ() * matrix[1].GetX();
    error += (std::abs(a00xa12) + std::abs(a02xa10)) * std::abs(matrix[2].GetY());
    det -= (a00xa12 - a02xa10) * matrix[2].GetY();

    double a00xa11 = matrix[0].GetX() * matrix[1].GetY();
    double a01xa10 = matrix[0].GetY() * matrix[1].GetX();
    error += (std::abs(a00xa11) + std::abs(a01xa10)) * std::abs(matrix[2].GetZ());
    det += (a00xa11 - a01xa10) * matrix[2].GetZ();

    return det;
}

double ComputeMeshVolume(const std::vector<VHACD::Vertex>& vertices,
                         const std::vector<VHACD::Triangle>& indices)
{
    double volume = 0;
    for (uint32_t i = 0; i < indices.size(); i++)
    {
        const std::array<VHACD::Vect3, 3> m = {
            vertices[indices[i].mI0],
            vertices[indices[i].mI1],
            vertices[indices[i].mI2]
        };
        double placeholder;
        volume += Determinant3x3(m,
                                 placeholder);
    }

    volume *= (double(1.0) / double(6.0));
    if (volume < 0)
        volume *= -1;
    return volume;
}

/*
 * Returns index of highest set bit in x
 */
inline int dExp2(int x)
{
    int exp;
    for (exp = -1; x; x >>= 1)
    {
        exp++;
    }
    return exp;
}

/*
 * Reverses the order of the bits in v and returns the result
 * Does not put fill any of the bits higher than the highest bit in v
 * Only used to calculate index of ndNormalMap::m_normal when tessellating a triangle
 */
inline int dBitReversal(int v,
                        int base)
{
    int x = 0;
    int power = dExp2(base) - 1;
    do
    {
        x += (v & 1) << power;
        v >>= 1;
        power--;
    } while (v);
    return x;
}


Googol Determinant3x3(const std::array<VHACD::Vector3<Googol>, 3>& matrix)
{
    Googol det = double(0.0);

    Googol a01xa12 = matrix[0].GetY() * matrix[1].GetZ();
    Googol a02xa11 = matrix[0].GetZ() * matrix[1].GetY();
    det += (a01xa12 - a02xa11) * matrix[2].GetX();

    Googol a00xa12 = matrix[0].GetX() * matrix[1].GetZ();
    Googol a02xa10 = matrix[0].GetZ() * matrix[1].GetX();
    det -= (a00xa12 - a02xa10) * matrix[2].GetY();

    Googol a00xa11 = matrix[0].GetX() * matrix[1].GetY();
    Googol a01xa10 = matrix[0].GetY() * matrix[1].GetX();
    det += (a00xa11 - a01xa10) * matrix[2].GetZ();
    return det;
}
#include "ConvexHullFace.h"
#include "ConvexHullVertex.h"
#include "ConvexHullAABBTreeNode.h"
#include "ConvexHull.h"
#include "ConvexHull_nbNormalMap.h"
#include "ConvexHull.inl"

#include "Axes.h"

#include "KdTreeFindNode.h"
#include "KdTree.h"
#include "KdTreeNode.h"

#include "kdTree.inl"
#include "KdTreeNode.inl"

class VertexIndex
{
public:
    VertexIndex(double granularity,
                bool snapToGrid);

    VHACD::Vect3 SnapToGrid(VHACD::Vect3 p);

    uint32_t GetIndex(VHACD::Vect3 p,
                      bool& newPos);

    const std::vector<VHACD::Vertex>& GetVertices() const;

    std::vector<VHACD::Vertex>&& TakeVertices();

    uint32_t GetVCount() const;

    bool SaveAsObj(const char* fname,
                   uint32_t tcount,
                   uint32_t* indices)
    {
        bool ret = false;

        FILE* fph = fopen(fname, "wb");
        if (fph)
        {
            ret = true;

            const std::vector<VHACD::Vertex>& v = GetVertices();
            for (uint32_t i = 0; i < v.size(); ++i)
            {
                fprintf(fph, "v %0.9f %0.9f %0.9f\r\n",
                        v[i].mX,
                        v[i].mY,
                        v[i].mZ);
            }

            for (uint32_t i = 0; i < tcount; i++)
            {
                uint32_t i1 = *indices++;
                uint32_t i2 = *indices++;
                uint32_t i3 = *indices++;
                fprintf(fph, "f %d %d %d\r\n",
                        i1 + 1,
                        i2 + 1,
                        i3 + 1);
            }
            fclose(fph);
        }

        return ret;
    }

private:
    bool m_snapToGrid : 1;
    double m_granularity;
    KdTree m_KdTree;
};

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

/*
 * A wrapper class for 3 10 bit integers packed into a 32 bit integer
 * Layout is [PAD][X][Y][Z]
 * Pad is bits 31-30, X is 29-20, Y is 19-10, and Z is 9-0
 */
class Voxel
{
    /*
     * Specify all of them for consistency
     */
    static constexpr int VoxelBitsZStart =  0;
    static constexpr int VoxelBitsYStart = 10;
    static constexpr int VoxelBitsXStart = 20;
    static constexpr int VoxelBitMask = 0x03FF; // bits 0 through 9 inclusive
public:
    Voxel() = default;

    Voxel(uint32_t index);

    Voxel(uint32_t x,
          uint32_t y,
          uint32_t z);

    bool operator==(const Voxel &v) const;

    VHACD::Vector3<uint32_t> GetVoxel() const;

    uint32_t GetX() const;
    uint32_t GetY() const;
    uint32_t GetZ() const;

    uint32_t GetVoxelAddress() const;

private:
    uint32_t m_voxel{ 0 };
};

Voxel::Voxel(uint32_t index)
    : m_voxel(index)
{
}

Voxel::Voxel(uint32_t x,
             uint32_t y,
             uint32_t z)
    : m_voxel((x << VoxelBitsXStart) | (y << VoxelBitsYStart) | (z << VoxelBitsZStart))
{
    assert(x < 1024 && "Voxel constructed with X outside of range");
    assert(y < 1024 && "Voxel constructed with Y outside of range");
    assert(z < 1024 && "Voxel constructed with Z outside of range");
}

bool Voxel::operator==(const Voxel& v) const
{
    return m_voxel == v.m_voxel;
}

VHACD::Vector3<uint32_t> Voxel::GetVoxel() const
{
    return VHACD::Vector3<uint32_t>(GetX(), GetY(), GetZ());
}

uint32_t Voxel::GetX() const
{
    return (m_voxel >> VoxelBitsXStart) & VoxelBitMask;
}

uint32_t Voxel::GetY() const
{
    return (m_voxel >> VoxelBitsYStart) & VoxelBitMask;
}

uint32_t Voxel::GetZ() const
{
    return (m_voxel >> VoxelBitsZStart) & VoxelBitMask;
}

uint32_t Voxel::GetVoxelAddress() const
{
    return m_voxel;
}

struct SimpleMesh
{
    std::vector<VHACD::Vertex> m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};

/*======================== 0-tests ========================*/
inline bool IntersectRayAABB(const VHACD::Vect3& start,
                             const VHACD::Vect3& dir,
                             const VHACD::BoundsAABB& bounds,
                             double& t)
{
    //! calculate candidate plane on each axis
    bool inside = true;
    VHACD::Vect3 ta(double(-1.0));

    //! use unrolled loops
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (start[i] < bounds.GetMin()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMin()[i] - start[i]) / dir[i];
            inside = false;
        }
        else if (start[i] > bounds.GetMax()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMax()[i] - start[i]) / dir[i];
            inside = false;
        }
    }

    //! if point inside all planes
    if (inside)
    {
        t = double(0.0);
        return true;
    }

    //! we now have t values for each of possible intersection planes
    //! find the maximum to get the intersection point
    uint32_t taxis;
    double tmax = ta.MaxCoeff(taxis);

    if (tmax < double(0.0))
        return false;

    //! check that the intersection point lies on the plane we picked
    //! we don't test the axis of closest intersection for precision reasons

    //! no eps for now
    double eps = double(0.0);

    VHACD::Vect3 hit = start + dir * tmax;

    if ((   hit.GetX() < bounds.GetMin().GetX() - eps
         || hit.GetX() > bounds.GetMax().GetX() + eps)
        && taxis != 0)
        return false;
    if ((   hit.GetY() < bounds.GetMin().GetY() - eps
         || hit.GetY() > bounds.GetMax().GetY() + eps)
        && taxis != 1)
        return false;
    if ((   hit.GetZ() < bounds.GetMin().GetZ() - eps
         || hit.GetZ() > bounds.GetMax().GetZ() + eps)
        && taxis != 2)
        return false;

    //! output results
    t = tmax;

    return true;
}

// Moller and Trumbore's method
inline bool IntersectRayTriTwoSided(const VHACD::Vect3& p,
                                    const VHACD::Vect3& dir,
                                    const VHACD::Vect3& a,
                                    const VHACD::Vect3& b,
                                    const VHACD::Vect3& c,
                                    double& t,
                                    double& u,
                                    double& v,
                                    double& w,
                                    double& sign,
                                    VHACD::Vect3* normal)
{
    VHACD::Vect3 ab = b - a;
    VHACD::Vect3 ac = c - a;
    VHACD::Vect3 n = ab.Cross(ac);

    double d = -dir.Dot(n);
    double ood = double(1.0) / d; // No need to check for division by zero here as infinity arithmetic will save us...
    VHACD::Vect3 ap = p - a;

    t = ap.Dot(n) * ood;
    if (t < double(0.0))
    {
        return false;
    }

    VHACD::Vect3 e = -dir.Cross(ap);
    v = ac.Dot(e) * ood;
    if (v < double(0.0) || v > double(1.0)) // ...here...
    {
        return false;
    }
    w = -ab.Dot(e) * ood;
    if (w < double(0.0) || v + w > double(1.0)) // ...and here
    {
        return false;
    }

    u = double(1.0) - v - w;
    if (normal)
    {
        *normal = n;
    }

    sign = d;

    return true;
}

// RTCD 5.1.5, page 142
inline VHACD::Vect3 ClosestPointOnTriangle(const VHACD::Vect3& a,
                                           const VHACD::Vect3& b,
                                           const VHACD::Vect3& c,
                                           const VHACD::Vect3& p,
                                           double& v,
                                           double& w)
{
    VHACD::Vect3 ab = b - a;
    VHACD::Vect3 ac = c - a;
    VHACD::Vect3 ap = p - a;

    double d1 = ab.Dot(ap);
    double d2 = ac.Dot(ap);
    if (   d1 <= double(0.0)
        && d2 <= double(0.0))
    {
        v = double(0.0);
        w = double(0.0);
        return a;
    }

    VHACD::Vect3 bp = p - b;
    double d3 = ab.Dot(bp);
    double d4 = ac.Dot(bp);
    if (   d3 >= double(0.0)
        && d4 <= d3)
    {
        v = double(1.0);
        w = double(0.0);
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (   vc <= double(0.0)
        && d1 >= double(0.0)
        && d3 <= double(0.0))
    {
        v = d1 / (d1 - d3);
        w = double(0.0);
        return a + v * ab;
    }

    VHACD::Vect3 cp = p - c;
    double d5 = ab.Dot(cp);
    double d6 = ac.Dot(cp);
    if (d6 >= double(0.0) && d5 <= d6)
    {
        v = double(0.0);
        w = double(1.0);
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (   vb <= double(0.0)
        && d2 >= double(0.0)
        && d6 <= double(0.0))
    {
        v = double(0.0);
        w = d2 / (d2 - d6);
        return a + w * ac;
    }

    double va = d3 * d6 - d5 * d4;
    if (   va <= double(0.0)
        && (d4 - d3) >= double(0.0)
        && (d5 - d6) >= double(0.0))
    {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        v = double(1.0) - w;
        return b + w * (c - b);
    }

    double denom = double(1.0) / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    return a + ab * v + ac * w;
}

class AABBTree
{
public:
    AABBTree() = default;
    AABBTree(AABBTree&&) = default;
    AABBTree& operator=(AABBTree&&) = default;

    AABBTree(const std::vector<VHACD::Vertex>& vertices,
             const std::vector<VHACD::Triangle>& indices);

    bool TraceRay(const VHACD::Vect3& start,
                  const VHACD::Vect3& to,
                  double& outT,
                  double& faceSign,
                  VHACD::Vect3& hitLocation) const;

    bool TraceRay(const VHACD::Vect3& start,
                  const VHACD::Vect3& dir,
                  uint32_t& insideCount,
                  uint32_t& outsideCount) const;

    bool TraceRay(const VHACD::Vect3& start,
                  const VHACD::Vect3& dir,
                  double& outT,
                  double& u,
                  double& v,
                  double& w,
                  double& faceSign,
                  uint32_t& faceIndex) const;

    VHACD::Vect3 GetCenter() const;
    VHACD::Vect3 GetMinExtents() const;
    VHACD::Vect3 GetMaxExtents() const;

    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                       double maxDistance,
                                       VHACD::Vect3& closestPoint) const;

private:
    struct Node
    {
        union
        {
            uint32_t m_children;
            uint32_t m_numFaces{ 0 };
        };

        uint32_t* m_faces{ nullptr };
        VHACD::BoundsAABB m_extents;
    };

    struct FaceSorter
    {
        FaceSorter(const std::vector<VHACD::Vertex>& positions,
                   const std::vector<VHACD::Triangle>& indices,
                   uint32_t axis);

        bool operator()(uint32_t lhs, uint32_t rhs) const;

        double GetCentroid(uint32_t face) const;

        const std::vector<VHACD::Vertex>& m_vertices;
        const std::vector<VHACD::Triangle>& m_indices;
        uint32_t m_axis;
    };

    // partition the objects and return the number of objects in the lower partition
    uint32_t PartitionMedian(Node& n,
                             uint32_t* faces,
                             uint32_t numFaces);
    uint32_t PartitionSAH(Node& n,
                          uint32_t* faces,
                          uint32_t numFaces);

    void Build();

    void BuildRecursive(uint32_t nodeIndex,
                        uint32_t* faces,
                        uint32_t numFaces);

    void TraceRecursive(uint32_t nodeIndex,
                        const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                       const double maxDis,
                                       double& dis,
                                       double& v,
                                       double& w,
                                       uint32_t& faceIndex,
                                       VHACD::Vect3& closest) const;

    void GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                  const VHACD::Vect3& point,
                                                  double& outDisSq,
                                                  double& outV,
                                                  double& outW,
                                                  uint32_t& outFaceIndex,
                                                  VHACD::Vect3& closest) const;

    VHACD::BoundsAABB CalculateFaceBounds(uint32_t* faces,
                                          uint32_t numFaces);

    // track the next free node
    uint32_t m_freeNode;

    const std::vector<VHACD::Vertex>* m_vertices{ nullptr };
    const std::vector<VHACD::Triangle>* m_indices{ nullptr };

    std::vector<uint32_t> m_faces;
    std::vector<Node> m_nodes;
    std::vector<VHACD::BoundsAABB> m_faceBounds;

    // stats
    uint32_t m_treeDepth{ 0 };
    uint32_t m_innerNodes{ 0 };
    uint32_t m_leafNodes{ 0 };

    uint32_t s_depth{ 0 };
};

AABBTree::FaceSorter::FaceSorter(const std::vector<VHACD::Vertex>& positions,
                                 const std::vector<VHACD::Triangle>& indices,
                                 uint32_t axis)
    : m_vertices(positions)
    , m_indices(indices)
    , m_axis(axis)
{
}

inline bool AABBTree::FaceSorter::operator()(uint32_t lhs,
                                             uint32_t rhs) const
{
    double a = GetCentroid(lhs);
    double b = GetCentroid(rhs);

    if (a == b)
    {
        return lhs < rhs;
    }
    else
    {
        return a < b;
    }
}

inline double AABBTree::FaceSorter::GetCentroid(uint32_t face) const
{
    const VHACD::Vect3& a = m_vertices[m_indices[face].mI0];
    const VHACD::Vect3& b = m_vertices[m_indices[face].mI1];
    const VHACD::Vect3& c = m_vertices[m_indices[face].mI2];

    return (a[m_axis] + b[m_axis] + c[m_axis]) / double(3.0);
}

AABBTree::AABBTree(const std::vector<VHACD::Vertex>& vertices,
                   const std::vector<VHACD::Triangle>& indices)
    : m_vertices(&vertices)
    , m_indices(&indices)
{
    Build();
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
                        const VHACD::Vect3& to,
                        double& outT,
                        double& faceSign,
                        VHACD::Vect3& hitLocation) const
{
    VHACD::Vect3 dir = to - start;
    double distance = dir.Normalize();
    double u, v, w;
    uint32_t faceIndex;
    bool hit = TraceRay(start,
                        dir,
                        outT,
                        u,
                        v,
                        w,
                        faceSign,
                        faceIndex);
    if (hit)
    {
        hitLocation = start + dir * outT;
    }

    if (hit && outT > distance)
    {
        hit = false;
    }
    return hit;
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        uint32_t& insideCount,
                        uint32_t& outsideCount) const
{
    double outT, u, v, w, faceSign;
    uint32_t faceIndex;
    bool hit = TraceRay(start,
                        dir,
                        outT,
                        u,
                        v,
                        w,
                        faceSign,
                        faceIndex);
    if (hit)
    {
        if (faceSign >= 0)
        {
            insideCount++;
        }
        else
        {
            outsideCount++;
        }
    }
    return hit;
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const
{
    outT = FLT_MAX;
    TraceRecursive(0,
                   start,
                   dir,
                   outT,
                   u,
                   v,
                   w,
                   faceSign,
                   faceIndex);
    return (outT != FLT_MAX);
}

VHACD::Vect3 AABBTree::GetCenter() const
{
    return m_nodes[0].m_extents.GetCenter();
}

VHACD::Vect3 AABBTree::GetMinExtents() const
{
    return m_nodes[0].m_extents.GetMin();
}

VHACD::Vect3 AABBTree::GetMaxExtents() const
{
    return m_nodes[0].m_extents.GetMax();
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                             double maxDistance,
                                             VHACD::Vect3& closestPoint) const
{
    double dis, v, w;
    uint32_t faceIndex;
    bool hit = GetClosestPointWithinDistance(point,
                                             maxDistance,
                                             dis,
                                             v,
                                             w,
                                             faceIndex,
                                             closestPoint);
    return hit;
}

// partition faces around the median face
uint32_t AABBTree::PartitionMedian(Node& n,
                                   uint32_t* faces,
                                   uint32_t numFaces)
{
    FaceSorter predicate(*m_vertices,
                         *m_indices,
                         n.m_extents.GetSize().LongestAxis());
    std::nth_element(faces,
                     faces + numFaces / 2,
                     faces + numFaces,
                     predicate);

    return numFaces / 2;
}

// partition faces based on the surface area heuristic
uint32_t AABBTree::PartitionSAH(Node&,
                                uint32_t* faces,
                                uint32_t numFaces)
{
    uint32_t bestAxis = 0;
    uint32_t bestIndex = 0;
    double bestCost = FLT_MAX;

    for (uint32_t a = 0; a < 3; ++a)
    {
        // sort faces by centroids
        FaceSorter predicate(*m_vertices,
                             *m_indices,
                             a);
        std::sort(faces,
                  faces + numFaces,
                  predicate);

        // two passes over data to calculate upper and lower bounds
        std::vector<double> cumulativeLower(numFaces);
        std::vector<double> cumulativeUpper(numFaces);

        VHACD::BoundsAABB lower;
        VHACD::BoundsAABB upper;

        for (uint32_t i = 0; i < numFaces; ++i)
        {
            lower.Union(m_faceBounds[faces[i]]);
            upper.Union(m_faceBounds[faces[numFaces - i - 1]]);

            cumulativeLower[i] = lower.SurfaceArea();
            cumulativeUpper[numFaces - i - 1] = upper.SurfaceArea();
        }

        double invTotalSA = double(1.0) / cumulativeUpper[0];

        // test all split positions
        for (uint32_t i = 0; i < numFaces - 1; ++i)
        {
            double pBelow = cumulativeLower[i] * invTotalSA;
            double pAbove = cumulativeUpper[i] * invTotalSA;

            double cost = double(0.125) + (pBelow * i + pAbove * (numFaces - i));
            if (cost <= bestCost)
            {
                bestCost = cost;
                bestIndex = i;
                bestAxis = a;
            }
        }
    }

    // re-sort by best axis
    FaceSorter predicate(*m_vertices,
                         *m_indices,
                         bestAxis);
    std::sort(faces,
              faces + numFaces,
              predicate);

    return bestIndex + 1;
}

void AABBTree::Build()
{
    const uint32_t numFaces = uint32_t(m_indices->size());

    // build initial list of faces
    m_faces.reserve(numFaces);

    // calculate bounds of each face and store
    m_faceBounds.reserve(numFaces);

    std::vector<VHACD::BoundsAABB> stack;
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        VHACD::BoundsAABB top = CalculateFaceBounds(&i,
                                                    1);

        m_faces.push_back(i);
        m_faceBounds.push_back(top);
    }

    m_nodes.reserve(uint32_t(numFaces * double(1.5)));

    // allocate space for all the nodes
    m_freeNode = 1;

    // start building
    BuildRecursive(0,
                   m_faces.data(),
                   numFaces);

    assert(s_depth == 0);
}

void AABBTree::BuildRecursive(uint32_t nodeIndex,
                              uint32_t* faces,
                              uint32_t numFaces)
{
    const uint32_t kMaxFacesPerLeaf = 6;

    // if we've run out of nodes allocate some more
    if (nodeIndex >= m_nodes.size())
    {
        uint32_t s = std::max(uint32_t(double(1.5) * m_nodes.size()), 512U);
        m_nodes.resize(s);
    }

    // a reference to the current node, need to be careful here as this reference may become invalid if array is resized
    Node& n = m_nodes[nodeIndex];

    // track max tree depth
    ++s_depth;
    m_treeDepth = std::max(m_treeDepth, s_depth);

    n.m_extents = CalculateFaceBounds(faces,
                                      numFaces);

    // calculate bounds of faces and add node
    if (numFaces <= kMaxFacesPerLeaf)
    {
        n.m_faces = faces;
        n.m_numFaces = numFaces;

        ++m_leafNodes;
    }
    else
    {
        ++m_innerNodes;

        // face counts for each branch
        const uint32_t leftCount = PartitionMedian(n, faces, numFaces);
        // const uint32_t leftCount = PartitionSAH(n, faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // alloc 2 nodes
        m_nodes[nodeIndex].m_children = m_freeNode;

        // allocate two nodes
        m_freeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(m_nodes[nodeIndex].m_children + 0, faces, leftCount);
        BuildRecursive(m_nodes[nodeIndex].m_children + 1, faces + leftCount, rightCount);
    }

    --s_depth;
}

void AABBTree::TraceRecursive(uint32_t nodeIndex,
                              const VHACD::Vect3& start,
                              const VHACD::Vect3& dir,
                              double& outT,
                              double& outU,
                              double& outV,
                              double& outW,
                              double& faceSign,
                              uint32_t& faceIndex) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        double dist[2] = { FLT_MAX, FLT_MAX };

        IntersectRayAABB(start,
                         dir,
                         leftChild.m_extents,
                         dist[0]);
        IntersectRayAABB(start,
                         dir,
                         rightChild.m_extents,
                         dist[1]);

        uint32_t closest = 0;
        uint32_t furthest = 1;

        if (dist[1] < dist[0])
        {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
        {
            TraceRecursive(node.m_children + closest,
                           start,
                           dir,
                           outT,
                           outU,
                           outV,
                           outW,
                           faceSign,
                           faceIndex);
        }

        if (dist[furthest] < outT)
        {
            TraceRecursive(node.m_children + furthest,
                           start,
                           dir,
                           outT,
                           outU,
                           outV,
                           outW,
                           faceSign,
                           faceIndex);
        }
    }
    else
    {
        double t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];
            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s, NULL))
            {
                if (t < outT)
                {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                             const double maxDis,
                                             double& dis,
                                             double& v,
                                             double& w,
                                             uint32_t& faceIndex,
                                             VHACD::Vect3& closest) const
{
    dis = maxDis;
    faceIndex = uint32_t(~0);
    double disSq = dis * dis;

    GetClosestPointWithinDistanceSqRecursive(0,
                                             point,
                                             disSq,
                                             v,
                                             w,
                                             faceIndex,
                                             closest);
    dis = sqrt(disSq);

    return (faceIndex < (~(static_cast<unsigned int>(0))));
}

void AABBTree::GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                        const VHACD::Vect3& point,
                                                        double& outDisSq,
                                                        double& outV,
                                                        double& outW,
                                                        uint32_t& outFaceIndex,
                                                        VHACD::Vect3& closestPoint) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == nullptr)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        // double dist[2] = { FLT_MAX, FLT_MAX };
        VHACD::Vect3 lp = leftChild.m_extents.ClosestPoint(point);
        VHACD::Vect3 rp = rightChild.m_extents.ClosestPoint(point);


        uint32_t closest = 0;
        uint32_t furthest = 1;
        double dcSq = (point - lp).GetNormSquared();
        double dfSq = (point - rp).GetNormSquared();
        if (dfSq < dcSq)
        {
            closest = 1;
            furthest = 0;
            std::swap(dfSq, dcSq);
        }

        if (dcSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + closest,
                                                     point,
                                                     outDisSq,
                                                     outV,
                                                     outW,
                                                     outFaceIndex,
                                                     closestPoint);
        }

        if (dfSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + furthest,
                                                     point,
                                                     outDisSq,
                                                     outV,
                                                     outW,
                                                     outFaceIndex,
                                                     closestPoint);
        }
    }
    else
    {

        double v, w;
        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];

            VHACD::Vect3 cp = ClosestPointOnTriangle(a, b, c, point, v, w);
            double disSq = (cp - point).GetNormSquared();

            if (disSq < outDisSq)
            {
                closestPoint = cp;
                outDisSq = disSq;
                outV = v;
                outW = w;
                outFaceIndex = node.m_faces[i];
            }
        }
    }
}

VHACD::BoundsAABB AABBTree::CalculateFaceBounds(uint32_t* faces,
                                                uint32_t numFaces)
{
    VHACD::Vect3 minExtents( FLT_MAX);
    VHACD::Vect3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        VHACD::Vect3 a = (*m_vertices)[(*m_indices)[faces[i]].mI0];
        VHACD::Vect3 b = (*m_vertices)[(*m_indices)[faces[i]].mI1];
        VHACD::Vect3 c = (*m_vertices)[(*m_indices)[faces[i]].mI2];

        minExtents = a.CWiseMin(minExtents);
        maxExtents = a.CWiseMax(maxExtents);

        minExtents = b.CWiseMin(minExtents);
        maxExtents = b.CWiseMax(maxExtents);

        minExtents = c.CWiseMin(minExtents);
        maxExtents = c.CWiseMax(maxExtents);
    }

    return VHACD::BoundsAABB(minExtents,
                             maxExtents);
}

enum class VoxelValue : uint8_t
{
    PRIMITIVE_UNDEFINED = 0,
    PRIMITIVE_OUTSIDE_SURFACE_TOWALK = 1,
    PRIMITIVE_OUTSIDE_SURFACE = 2,
    PRIMITIVE_INSIDE_SURFACE = 3,
    PRIMITIVE_ON_SURFACE = 4
};

class Volume
{
public:
    void Voxelize(const std::vector<VHACD::Vertex>& points,
                  const std::vector<VHACD::Triangle>& triangles,
                  const size_t dim,
                  FillMode fillMode,
                  const AABBTree& aabbTree);

    void RaycastFill(const AABBTree& aabbTree);

    void SetVoxel(const size_t i,
                  const size_t j,
                  const size_t k,
                  VoxelValue value);

    VoxelValue& GetVoxel(const size_t i,
                         const size_t j,
                         const size_t k);

    const VoxelValue& GetVoxel(const size_t i,
                               const size_t j,
                               const size_t k) const;

    const std::vector<Voxel>& GetSurfaceVoxels() const;
    const std::vector<Voxel>& GetInteriorVoxels() const;

    double GetScale() const;
    const VHACD::BoundsAABB& GetBounds() const;
    const VHACD::Vector3<uint32_t>& GetDimensions() const;

    VHACD::BoundsAABB m_bounds;
    double m_scale{ 1.0 };
    VHACD::Vector3<uint32_t> m_dim{ 0 };
    size_t m_numVoxelsOnSurface{ 0 };
    size_t m_numVoxelsInsideSurface{ 0 };
    size_t m_numVoxelsOutsideSurface{ 0 };
    std::vector<VoxelValue> m_data;
private:

    void MarkOutsideSurface(const size_t i0,
                            const size_t j0,
                            const size_t k0,
                            const size_t i1,
                            const size_t j1,
                            const size_t k1);
    void FillOutsideSurface();

    void FillInsideSurface();

    std::vector<VHACD::Voxel> m_surfaceVoxels;
    std::vector<VHACD::Voxel> m_interiorVoxels;
};

bool PlaneBoxOverlap(const VHACD::Vect3& normal,
                     const VHACD::Vect3& vert,
                     const VHACD::Vect3& maxbox)
{
    int32_t q;
    VHACD::Vect3 vmin;
    VHACD::Vect3 vmax;
    double v;
    for (q = 0; q < 3; q++)
    {
        v = vert[q];
        if (normal[q] > double(0.0))
        {
            vmin[q] = -maxbox[q] - v;
            vmax[q] =  maxbox[q] - v;
        }
        else
        {
            vmin[q] =  maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if (normal.Dot(vmin) > double(0.0))
        return false;
    if (normal.Dot(vmax) >= double(0.0))
        return true;
    return false;
}

bool AxisTest(double  a, double  b, double fa, double fb,
              double v0, double v1, double v2, double v3,
              double boxHalfSize1,  double boxHalfSize2)
{
    double p0 = a * v0 + b * v1;
    double p1 = a * v2 + b * v3;

    double min = std::min(p0, p1);
    double max = std::max(p0, p1);

    double rad = fa * boxHalfSize1 + fb * boxHalfSize2;
    if (min > rad || max < -rad)
    {
        return false;
    }

    return true;
}

bool TriBoxOverlap(const VHACD::Vect3& boxCenter,
                   const VHACD::Vect3& boxHalfSize,
                   const VHACD::Vect3& triVer0,
                   const VHACD::Vect3& triVer1,
                   const VHACD::Vect3& triVer2)
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-direction) */
    /*       this gives 3x3=9 more tests */

    VHACD::Vect3 v0 = triVer0 - boxCenter;
    VHACD::Vect3 v1 = triVer1 - boxCenter;
    VHACD::Vect3 v2 = triVer2 - boxCenter;
    VHACD::Vect3 e0 = v1 - v0;
    VHACD::Vect3 e1 = v2 - v1;
    VHACD::Vect3 e2 = v0 - v2;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    double fex = fabs(e0[0]);
    double fey = fabs(e0[1]);
    double fez = fabs(e0[2]);

    /*
     * These should use Get*() instead of subscript for consistency, but the function calls are long enough already
     */
    if (!AxisTest( e0[2], -e0[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e0[2],  e0[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest( e0[1], -e0[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    fex = fabs(e1[0]);
    fey = fabs(e1[1]);
    fez = fabs(e1[2]);

    if (!AxisTest( e1[2], -e1[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e1[2],  e1[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest( e1[1], -e1[0], fey, fex, v0[0], v0[1], v1[0], v1[1], boxHalfSize[0], boxHalfSize[2])) return 0; // Z0

    fex = fabs(e2[0]);
    fey = fabs(e2[1]);
    fez = fabs(e2[2]);

    if (!AxisTest( e2[2], -e2[1], fez, fey, v0[1], v0[2], v1[1], v1[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X2
    if (!AxisTest(-e2[2],  e2[0], fez, fex, v0[0], v0[2], v1[0], v1[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y1
    if (!AxisTest( e2[1], -e2[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    double min = std::min({v0.GetX(), v1.GetX(), v2.GetX()});
    double max = std::max({v0.GetX(), v1.GetX(), v2.GetX()});
    if (min > boxHalfSize[0] || max < -boxHalfSize[0])
        return false;

    /* test in 1-direction */
    min = std::min({v0.GetY(), v1.GetY(), v2.GetY()});
    max = std::max({v0.GetY(), v1.GetY(), v2.GetY()});
    if (min > boxHalfSize[1] || max < -boxHalfSize[1])
        return false;

    /* test in getZ-direction */
    min = std::min({v0.GetZ(), v1.GetZ(), v2.GetZ()});
    max = std::max({v0.GetZ(), v1.GetZ(), v2.GetZ()});
    if (min > boxHalfSize[2] || max < -boxHalfSize[2])
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    VHACD::Vect3 normal = e0.Cross(e1);

    if (!PlaneBoxOverlap(normal, v0, boxHalfSize))
        return false;
    return true; /* box and triangle overlaps */
}

void Volume::Voxelize(const std::vector<VHACD::Vertex>& points,
                      const std::vector<VHACD::Triangle>& indices,
                      const size_t dimensions,
                      FillMode fillMode,
                      const AABBTree& aabbTree)
{
    double a = std::pow(dimensions, 0.33);
    size_t dim = a * double(1.5);
    dim = std::max(dim, size_t(32));

    if (points.size() == 0)
    {
        return;
    }

    m_bounds = BoundsAABB(points);

    VHACD::Vect3 d = m_bounds.GetSize();
    double r;
    // Equal comparison is important here to avoid taking the last branch when d[0] == d[1] with d[2] being the smallest
    // dimension. That would lead to dimensions in i and j to be a lot bigger than expected and make the amount of
    // voxels in the volume totally unmanageable.
    if (d[0] >= d[1] && d[0] >= d[2])
    {
        r = d[0];
        m_dim[0] = uint32_t(dim);
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[0]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[0]));
    }
    else if (d[1] >= d[0] && d[1] >= d[2])
    {
        r = d[1];
        m_dim[1] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[1]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[1]));
    }
    else
    {
        r = d[2];
        m_dim[2] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[2]));
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[2]));
    }

    m_scale = r / (dim - 1);
    double invScale = (dim - 1) / r;

    m_data = std::vector<VoxelValue>(m_dim[0] * m_dim[1] * m_dim[2],
                                     VoxelValue::PRIMITIVE_UNDEFINED);
    m_numVoxelsOnSurface = 0;
    m_numVoxelsInsideSurface = 0;
    m_numVoxelsOutsideSurface = 0;

    VHACD::Vect3 p[3];
    VHACD::Vect3 boxcenter;
    VHACD::Vect3 pt;
    const VHACD::Vect3 boxhalfsize(double(0.5));
    for (size_t t = 0; t < indices.size(); ++t)
    {
        size_t i0, j0, k0;
        size_t i1, j1, k1;
        VHACD::Vector3<uint32_t> tri = indices[t];
        for (int32_t c = 0; c < 3; ++c)
        {
            pt = points[tri[c]];

            p[c] = (pt - m_bounds.GetMin()) * invScale;

            size_t i = static_cast<size_t>(p[c][0] + double(0.5));
            size_t j = static_cast<size_t>(p[c][1] + double(0.5));
            size_t k = static_cast<size_t>(p[c][2] + double(0.5));

            assert(i < m_dim[0] && j < m_dim[1] && k < m_dim[2]);

            if (c == 0)
            {
                i0 = i1 = i;
                j0 = j1 = j;
                k0 = k1 = k;
            }
            else
            {
                i0 = std::min(i0, i);
                j0 = std::min(j0, j);
                k0 = std::min(k0, k);

                i1 = std::max(i1, i);
                j1 = std::max(j1, j);
                k1 = std::max(k1, k);
            }
        }
        if (i0 > 0)
            --i0;
        if (j0 > 0)
            --j0;
        if (k0 > 0)
            --k0;
        if (i1 < m_dim[0])
            ++i1;
        if (j1 < m_dim[1])
            ++j1;
        if (k1 < m_dim[2])
            ++k1;
        for (size_t i_id = i0; i_id < i1; ++i_id)
        {
            boxcenter[0] = uint32_t(i_id);
            for (size_t j_id = j0; j_id < j1; ++j_id)
            {
                boxcenter[1] = uint32_t(j_id);
                for (size_t k_id = k0; k_id < k1; ++k_id)
                {
                    boxcenter[2] = uint32_t(k_id);
                    bool res = TriBoxOverlap(boxcenter,
                                             boxhalfsize,
                                             p[0],
                                             p[1],
                                             p[2]);
                    VoxelValue& value = GetVoxel(i_id,
                                                 j_id,
                                                 k_id);
                    if (   res
                        && value == VoxelValue::PRIMITIVE_UNDEFINED)
                    {
                        value = VoxelValue::PRIMITIVE_ON_SURFACE;
                        ++m_numVoxelsOnSurface;
                        m_surfaceVoxels.emplace_back(uint32_t(i_id),
                                                     uint32_t(j_id),
                                                     uint32_t(k_id));
                    }
                }
            }
        }
    }

    if (fillMode == FillMode::SURFACE_ONLY)
    {
        const size_t i0_local = m_dim[0];
        const size_t j0_local = m_dim[1];
        const size_t k0_local = m_dim[2];
        for (size_t i_id = 0; i_id < i0_local; ++i_id)
        {
            for (size_t j_id = 0; j_id < j0_local; ++j_id)
            {
                for (size_t k_id = 0; k_id < k0_local; ++k_id)
                {
                    const VoxelValue& voxel = GetVoxel(i_id,
                                                       j_id,
                                                       k_id);
                    if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                    {
                        SetVoxel(i_id,
                                 j_id,
                                 k_id,
                                 VoxelValue::PRIMITIVE_OUTSIDE_SURFACE);
                    }
                }
            }
        }
    }
    else if (fillMode == FillMode::FLOOD_FILL)
    {
        /*
         * Marking the outside edges of the voxel cube to be outside surfaces to walk
         */
        MarkOutsideSurface(0,            0,            0,            m_dim[0], m_dim[1], 1);
        MarkOutsideSurface(0,            0,            m_dim[2] - 1, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0,            0,            0,            m_dim[0], 1,        m_dim[2]);
        MarkOutsideSurface(0,            m_dim[1] - 1, 0,            m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0,            0,            0,            1,        m_dim[1], m_dim[2]);
        MarkOutsideSurface(m_dim[0] - 1, 0,            0,            m_dim[0], m_dim[1], m_dim[2]);
        FillOutsideSurface();
        FillInsideSurface();
    }
    else if (fillMode == FillMode::RAYCAST_FILL)
    {
        RaycastFill(aabbTree);
    }
}

void Volume::RaycastFill(const AABBTree& aabbTree)
{
    const uint32_t i0 = m_dim[0];
    const uint32_t j0 = m_dim[1];
    const uint32_t k0 = m_dim[2];

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };
    m_numVoxelsInsideSurface = 0;
    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& voxel = GetVoxel(i, j, k);
                if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                {
                    VHACD::Vect3 start = VHACD::Vect3(i, j, k) * m_scale + m_bounds.GetMin();

                    uint32_t insideCount = 0;
                    uint32_t outsideCount = 0;

                    VHACD::Vect3 directions[6] = {
                        VHACD::Vect3( 1,  0,  0),
                        VHACD::Vect3(-1,  0,  0), // this was 1, 0, 0 in the original code, but looks wrong
                        VHACD::Vect3( 0,  1,  0),
                        VHACD::Vect3( 0, -1,  0),
                        VHACD::Vect3( 0,  0,  1),
                        VHACD::Vect3( 0,  0, -1)
                    };

                    for (uint32_t r = 0; r < 6; r++)
                    {
                        aabbTree.TraceRay(start,
                                          directions[r],
                                          insideCount,
                                          outsideCount);
                        // Early out if we hit the outside of the mesh
                        if (outsideCount)
                        {
                            break;
                        }
                        // Early out if we accumulated 3 inside hits
                        if (insideCount >= 3)
                        {
                            break;
                        }
                    }

                    if (outsideCount == 0 && insideCount >= 3)
                    {
                        voxel = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                        temp.emplace_back(i, j, k);
                        count++;
                        m_numVoxelsInsideSurface++;
                    }
                    else
                    {
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;
                    }
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

void Volume::SetVoxel(const size_t i,
                      const size_t j,
                      const size_t k,
                      VoxelValue value)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);

    m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]] = value;
}

VoxelValue& Volume::GetVoxel(const size_t i,
                             const size_t j,
                             const size_t k)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const VoxelValue& Volume::GetVoxel(const size_t i,
                                   const size_t j,
                                   const size_t k) const
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const std::vector<Voxel>& Volume::GetSurfaceVoxels() const
{
    return m_surfaceVoxels;
}

const std::vector<Voxel>& Volume::GetInteriorVoxels() const
{
    return m_interiorVoxels;
}

double Volume::GetScale() const
{
    return m_scale;
}

const VHACD::BoundsAABB& Volume::GetBounds() const
{
    return m_bounds;
}

const VHACD::Vector3<uint32_t>& Volume::GetDimensions() const
{
    return m_dim;
}

void Volume::MarkOutsideSurface(const size_t i0,
                                const size_t j0,
                                const size_t k0,
                                const size_t i1,
                                const size_t j1,
                                const size_t k1)
{
    for (size_t i = i0; i < i1; ++i)
    {
        for (size_t j = j0; j < j1; ++j)
        {
            for (size_t k = k0; k < k1; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
                }
            }
        }
    }
}

inline void WalkForward(int64_t start,
                        int64_t end,
                        VoxelValue* ptr,
                        int64_t stride,
                        int64_t maxDistance)
{
    for (int64_t i = start, count = 0;
         count < maxDistance && i < end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
         ++i, ptr += stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

inline void WalkBackward(int64_t start,
                         int64_t end,
                         VoxelValue* ptr,
                         int64_t stride,
                         int64_t maxDistance)
{
    for (int64_t i = start, count = 0;
         count < maxDistance && i >= end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
         --i, ptr -= stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

void Volume::FillOutsideSurface()
{
    size_t voxelsWalked = 0;
    const int64_t i0 = m_dim[0];
    const int64_t j0 = m_dim[1];
    const int64_t k0 = m_dim[2];

    // Avoid striding too far in each direction to stay in L1 cache as much as possible.
    // The cache size required for the walk is roughly (4 * walkDistance * 64) since
    // the k direction doesn't count as it's walking byte per byte directly in a cache lines.
    // ~16k is required for a walk distance of 64 in each directions.
    const size_t walkDistance = 64;

    // using the stride directly instead of calling GetVoxel for each iterations saves
    // a lot of multiplications and pipeline stalls due to data dependencies on imul.
    const size_t istride = &GetVoxel(1, 0, 0) - &GetVoxel(0, 0, 0);
    const size_t jstride = &GetVoxel(0, 1, 0) - &GetVoxel(0, 0, 0);
    const size_t kstride = &GetVoxel(0, 0, 1) - &GetVoxel(0, 0, 0);

    // It might seem counter intuitive to go over the whole voxel range multiple times
    // but since we do the run in memory order, it leaves us with far fewer cache misses
    // than a BFS algorithm and it has the additional benefit of not requiring us to
    // store and manipulate a fifo for recursion that might become huge when the number
    // of voxels is large.
    // This will outperform the BFS algorithm by several orders of magnitude in practice.
    do
    {
        voxelsWalked = 0;
        for (int64_t i = 0; i < i0; ++i)
        {
            for (int64_t j = 0; j < j0; ++j)
            {
                for (int64_t k = 0; k < k0; ++k)
                {
                    VoxelValue& voxel = GetVoxel(i, j, k);
                    if (voxel == VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK)
                    {
                        voxelsWalked++;
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;

                        // walk in each direction to mark other voxel that should be walked.
                        // this will generate a 3d pattern that will help the overall
                        // algorithm converge faster while remaining cache friendly.
                        WalkForward(k + 1, k0, &voxel + kstride, kstride, walkDistance);
                        WalkBackward(k - 1, 0, &voxel - kstride, kstride, walkDistance);

                        WalkForward(j + 1, j0, &voxel + jstride, jstride, walkDistance);
                        WalkBackward(j - 1, 0, &voxel - jstride, jstride, walkDistance);

                        WalkForward(i + 1, i0, &voxel + istride, istride, walkDistance);
                        WalkBackward(i - 1, 0, &voxel - istride, istride, walkDistance);
                    }
                }
            }
        }

        m_numVoxelsOutsideSurface += voxelsWalked;
    } while (voxelsWalked != 0);
}

void Volume::FillInsideSurface()
{
    const uint32_t i0 = uint32_t(m_dim[0]);
    const uint32_t j0 = uint32_t(m_dim[1]);
    const uint32_t k0 = uint32_t(m_dim[2]);

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };

    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                    temp.emplace_back(i, j, k);
                    count++;
                    ++m_numVoxelsInsideSurface;
                }
            }
        }
    }

    if ( count )
    {
        m_interiorVoxels = std::move(temp);
    }
}

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
                               uint32_t maxHullVertices);

    const std::vector<VHACD::Vertex>& GetVertices() const;
    const std::vector<VHACD::Triangle>& GetIndices() const;

private:
    std::vector<VHACD::Vertex>   m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};

uint32_t QuickHull::ComputeConvexHull(const std::vector<VHACD::Vertex>& vertices,
                                      uint32_t maxHullVertices)
{
    m_indices.clear();

    VHACD::ConvexHull ch(vertices,
                         double(0.0001),
                         maxHullVertices);

    auto& vlist = ch.GetVertexPool();
    if ( !vlist.empty() )
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

const std::vector<VHACD::Vertex>& QuickHull::GetVertices() const
{
    return m_vertices;
}

const std::vector<VHACD::Triangle>& QuickHull::GetIndices() const
{
    return m_indices;
}

//******************************************************************************************
// Implementation of the ShrinkWrap function
//******************************************************************************************

void ShrinkWrap(SimpleMesh& sourceConvexHull,
                const AABBTree& aabbTree,
                uint32_t maxHullVertexCount,
                double distanceThreshold,
                bool doShrinkWrap)
{
    std::vector<VHACD::Vertex> verts; // New verts for the new convex hull
    verts.reserve(sourceConvexHull.m_vertices.size());
    // Examine each vertex and see if it is within the voxel distance.
    // If it is, then replace the point with the shrinkwrapped / projected point
    for (uint32_t j = 0; j < sourceConvexHull.m_vertices.size(); j++)
    {
        VHACD::Vertex& p = sourceConvexHull.m_vertices[j];
        if (doShrinkWrap)
        {
            VHACD::Vect3 closest;
            if (aabbTree.GetClosestPointWithinDistance(p, distanceThreshold, closest))
            {
                p = closest;
            }
        }
        verts.emplace_back(p);
    }
    // Final step is to recompute the convex hull
    VHACD::QuickHull qh;
    uint32_t tcount = qh.ComputeConvexHull(verts,
                                            maxHullVertexCount);
    if (tcount)
    {
        sourceConvexHull.m_vertices = qh.GetVertices();
        sourceConvexHull.m_indices = qh.GetIndices();
    }
}

//********************************************************************************************************************

#if !VHACD_DISABLE_THREADING

//********************************************************************************************************************
// Definition of the ThreadPool
//********************************************************************************************************************

class ThreadPool {
 public:
    ThreadPool();
    ThreadPool(int worker);
    ~ThreadPool();
    template<typename F, typename... Args>
    auto enqueue(F&& f, Args&& ... args)
#ifndef __cpp_lib_is_invocable
        -> std::future< typename std::result_of< F( Args... ) >::type>;
#else
        -> std::future< typename std::invoke_result_t<F, Args...>>;
#endif
 private:
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> tasks;
    std::mutex task_mutex;
    std::condition_variable cv;
    bool closed;
    int count;
};

ThreadPool::ThreadPool()
    : ThreadPool(1)
{
}

ThreadPool::ThreadPool(int worker)
    : closed(false)
    , count(0)
{
    workers.reserve(worker);
    for(int i=0; i<worker; i++) 
    {
        workers.emplace_back(
            [this]
            {
                std::unique_lock<std::mutex> lock(this->task_mutex);
                while(true) 
                {
                    while (this->tasks.empty()) 
                    {
                        if (this->closed) 
                        {
                            return;
                        }
                        this->cv.wait(lock);
                    }
                    auto task = this->tasks.front();
                    this->tasks.pop_front();
                    lock.unlock();
                    task();
                    lock.lock();
                }
            }
        );
    }
}

template<typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&& ... args)
#ifndef __cpp_lib_is_invocable
    -> std::future< typename std::result_of< F( Args... ) >::type>
#else
    -> std::future< typename std::invoke_result_t<F, Args...>>
#endif
{

#ifndef __cpp_lib_is_invocable
    using return_type = typename std::result_of< F( Args... ) >::type;
#else
    using return_type = typename std::invoke_result_t< F, Args... >;
#endif
    auto task = std::make_shared<std::packaged_task<return_type()> > (
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
    auto result = task->get_future();

    {
        std::unique_lock<std::mutex> lock(task_mutex);
        if (!closed) 
        {
            tasks.emplace_back([task]
            { 
                (*task)();
            });
            cv.notify_one();
        }
    }

    return result;
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(task_mutex);
        closed = true;
    }
    cv.notify_all();
    for (auto && worker : workers) 
    {
        worker.join();
    }
}
#endif

enum class Stages
{
    COMPUTE_BOUNDS_OF_INPUT_MESH,
    REINDEXING_INPUT_MESH,
    CREATE_RAYCAST_MESH,
    VOXELIZING_INPUT_MESH,
    BUILD_INITIAL_CONVEX_HULL,
    PERFORMING_DECOMPOSITION,
    INITIALIZING_CONVEX_HULLS_FOR_MERGING,
    COMPUTING_COST_MATRIX,
    MERGING_CONVEX_HULLS,
    FINALIZING_RESULTS,
    NUM_STAGES
};

class VHACDCallbacks
{
public:
    virtual void ProgressUpdate(Stages stage,
                                double stageProgress,
                                const char *operation) = 0;
    virtual bool IsCanceled() const = 0;

    virtual ~VHACDCallbacks() = default;
};

enum class SplitAxis
{
    X_AXIS_NEGATIVE,
    X_AXIS_POSITIVE,
    Y_AXIS_NEGATIVE,
    Y_AXIS_POSITIVE,
    Z_AXIS_NEGATIVE,
    Z_AXIS_POSITIVE,
};

// This class represents a collection of voxels, the convex hull
// which surrounds them, and a triangle mesh representation of those voxels
class VoxelHull
{
public:

    // This method constructs a new VoxelHull based on a plane split of the parent
    // convex hull
    VoxelHull(const VoxelHull& parent,
              SplitAxis axis,
              uint32_t splitLoc);

    // Here we construct the initial convex hull around the
    // entire voxel set
    VoxelHull(Volume& voxels,
              const IVHACD::Parameters &params,
              VHACDCallbacks *callbacks);

    ~VoxelHull() = default;

    // Helper method to refresh the min/max voxel bounding region
    void MinMaxVoxelRegion(const Voxel &v);

    void BuildRaycastMesh();

    // We now compute the convex hull relative to a triangle mesh generated 
    // from the voxels
    void ComputeConvexHull();

    // Returns true if this convex hull should be considered done
    bool IsComplete();

    
    // Convert a voxel position into it's correct double precision location
    VHACD::Vect3 GetPoint(const int32_t x,
                                 const int32_t y,
                                 const int32_t z,
                                 const double scale,
                                 const VHACD::Vect3& bmin) const;

    // Sees if we have already got an index for this voxel position.
    // If the voxel position has already been indexed, we just return
    // that index value.
    // If not, then we convert it into the floating point position and
    // add it to the index map
    uint32_t GetVertexIndex(const VHACD::Vector3<uint32_t>& p);

    // This method will convert the voxels into an actual indexed triangle mesh of boxes
    // This serves two purposes.
    // The primary purpose is so that when we compute a convex hull it considered all of the points
    // for each voxel, not just the center point. If you don't do this, then the hulls don't fit the
    // mesh accurately enough.
    // The second reason we convert it into a triangle mesh is so that we can do raycasting against it
    // to search for the best splitting plane fairly quickly. That algorithm will be discussed in the 
    // method which computes the best splitting plane.
    void BuildVoxelMesh();

    // Convert a single voxel position into an actual 3d box mesh comprised
    // of 12 triangles
    void AddVoxelBox(const Voxel &v);
    
    // Add the triangle represented by these 3 indices into the 'box' set of vertices
    // to the output mesh
    void AddTri(const std::array<VHACD::Vector3<uint32_t>, 8>& box,
                uint32_t i1,
                uint32_t i2,
                uint32_t i3);

    // Here we convert from voxel space to a 3d position, index it, and add
    // the triangle positions and indices for the output mesh
    void AddTriangle(const VHACD::Vector3<uint32_t>& p1,
                     const VHACD::Vector3<uint32_t>& p2,
                     const VHACD::Vector3<uint32_t>& p3);

    // When computing the split plane, we start by simply 
    // taking the midpoint of the longest side. However,
    // we can also search the surface and look for the greatest
    // spot of concavity and use that as the split location.
    // This will make the convex decomposition more efficient
    // as it will tend to cut across the greatest point of
    // concavity on the surface.
    SplitAxis ComputeSplitPlane(uint32_t& location);

    VHACD::Vect3 GetPosition(const VHACD::Vector3<int32_t>& ip) const;

    double Raycast(const VHACD::Vector3<int32_t>& p1,
                   const VHACD::Vector3<int32_t>& p2) const;

    bool FindConcavity(uint32_t idx,
                       uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityX(uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityY(uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityZ(uint32_t& splitLoc);

    // This operation is performed in a background thread.
    // It splits the voxels by a plane
    void PerformPlaneSplit();

    // Used only for debugging. Saves the voxelized mesh to disk
    // Optionally saves the original source mesh as well for comparison
    void SaveVoxelMesh(const SimpleMesh& inputMesh,
                       bool saveVoxelMesh,
                       bool saveSourceMesh);

    void SaveOBJ(const char* fname,
                 const VoxelHull* h);

    void SaveOBJ(const char* fname);

private:
    void WriteOBJ(FILE* fph,
                  const std::vector<VHACD::Vertex>& vertices,
                  const std::vector<VHACD::Triangle>& indices,
                  uint32_t baseIndex);
public:

    SplitAxis               m_axis{ SplitAxis::X_AXIS_NEGATIVE };
    Volume*                 m_voxels{ nullptr }; // The voxelized data set
    double                  m_voxelScale{ 0 };   // Size of a single voxel
    double                  m_voxelScaleHalf{ 0 }; // 1/2 of the size of a single voxel
    VHACD::BoundsAABB       m_voxelBounds;
    VHACD::Vect3            m_voxelAdjust;       // Minimum coordinates of the voxel space, with adjustment
    uint32_t                m_depth{ 0 };        // How deep in the recursion of the binary tree this hull is
    uint32_t                m_index{ 0 };        // Each convex hull is given a unique id to distinguish it from the others
    double                  m_volumeError{ 0 };  // The percentage error from the convex hull volume vs. the voxel volume
    double                  m_voxelVolume{ 0 };  // The volume of the voxels
    double                  m_hullVolume{ 0 };   // The volume of the enclosing convex hull

    std::unique_ptr<IVHACD::ConvexHull> m_convexHull{ nullptr }; // The convex hull which encloses this set of voxels.
    std::vector<Voxel>                  m_surfaceVoxels;     // The voxels which are on the surface of the source mesh.
    std::vector<Voxel>                  m_newSurfaceVoxels;  // Voxels which are on the surface as a result of a plane split
    std::vector<Voxel>                  m_interiorVoxels;    // Voxels which are part of the interior of the hull

    std::unique_ptr<VoxelHull>          m_hullA{ nullptr }; // hull resulting from one side of the plane split
    std::unique_ptr<VoxelHull>          m_hullB{ nullptr }; // hull resulting from the other side of the plane split

    // Defines the coordinates this convex hull comprises within the voxel volume
    // of the entire source
    VHACD::Vector3<uint32_t>                    m_1{ 0 };
    VHACD::Vector3<uint32_t>                    m_2{ 0 };
    AABBTree                                    m_AABBTree;
    std::unordered_map<uint32_t, uint32_t>      m_voxelIndexMap; // Maps from a voxel coordinate space into a vertex index space
    std::vector<VHACD::Vertex>                  m_vertices;
    std::vector<VHACD::Triangle>                m_indices;
    static uint32_t                             m_voxelHullCount;
    IVHACD::Parameters                          m_params;
    VHACDCallbacks*                             m_callbacks{ nullptr };
};

uint32_t VoxelHull::m_voxelHullCount = 0;

VoxelHull::VoxelHull(const VoxelHull& parent,
                     SplitAxis axis,
                     uint32_t splitLoc)
    : m_axis(axis)
    , m_voxels(parent.m_voxels)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale * double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    , m_depth(parent.m_depth + 1)
    , m_index(++m_voxelHullCount)
    , m_1(parent.m_1)
    , m_2(parent.m_2)
    , m_params(parent.m_params)
{
    // Default copy the voxel region from the parent, but values will
    // be adjusted next based on the split axis and location
    switch ( m_axis )
    {
        case SplitAxis::X_AXIS_NEGATIVE:
            m_2.GetX() = splitLoc;
            break;
        case SplitAxis::X_AXIS_POSITIVE:
            m_1.GetX() = splitLoc + 1;
            break;
        case SplitAxis::Y_AXIS_NEGATIVE:
            m_2.GetY() = splitLoc;
            break;
        case SplitAxis::Y_AXIS_POSITIVE:
            m_1.GetY() = splitLoc + 1;
            break;
        case SplitAxis::Z_AXIS_NEGATIVE:
            m_2.GetZ() = splitLoc;
            break;
        case SplitAxis::Z_AXIS_POSITIVE:
            m_1.GetZ() = splitLoc + 1;
            break;
    }

    // First, we copy all of the interior voxels from our parent
    // which intersect our region
    for (auto& i : parent.m_interiorVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            bool newSurface = false;
            switch ( m_axis )
            {
                case SplitAxis::X_AXIS_NEGATIVE:
                    if ( v.GetX() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::X_AXIS_POSITIVE:
                    if ( v.GetX() == m_1.GetX() )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Y_AXIS_NEGATIVE:
                    if ( v.GetY() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Y_AXIS_POSITIVE:
                    if ( v.GetY() == m_1.GetY() )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Z_AXIS_NEGATIVE:
                    if ( v.GetZ() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Z_AXIS_POSITIVE:
                    if ( v.GetZ() == m_1.GetZ() )
                    {
                        newSurface = true;
                    }
                    break;
            }
            // If his interior voxels lie directly on the split plane then
            // these become new surface voxels for our patch
            if ( newSurface )
            {
                m_newSurfaceVoxels.push_back(i);
            }
            else
            {
                m_interiorVoxels.push_back(i);
            }
        }
    }
    // Next we copy all of the surface voxels which intersect our region
    for (auto& i : parent.m_surfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_surfaceVoxels.push_back(i);
        }
    }
    // Our parent's new surface voxels become our new surface voxels so long as they intersect our region
    for (auto& i : parent.m_newSurfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_newSurfaceVoxels.push_back(i);
        }
    }

    // Recompute the min-max bounding box which would be different after the split occurs
    m_1 = VHACD::Vector3<uint32_t>(0x7FFFFFFF);
    m_2 = VHACD::Vector3<uint32_t>(0);
    for (auto& i : m_surfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_newSurfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_interiorVoxels)
    {
        MinMaxVoxelRegion(i);
    }

    BuildVoxelMesh();
    BuildRaycastMesh(); // build a raycast mesh of the voxel mesh
    ComputeConvexHull();
}

VoxelHull::VoxelHull(Volume& voxels,
                     const IVHACD::Parameters& params,
                     VHACDCallbacks* callbacks)
    : m_voxels(&voxels)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale * double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    , m_index(++m_voxelHullCount)
    // Here we get a copy of all voxels which lie on the surface mesh
    , m_surfaceVoxels(m_voxels->GetSurfaceVoxels())
    // Now we get a copy of all voxels which are considered part of the 'interior' of the source mesh
    , m_interiorVoxels(m_voxels->GetInteriorVoxels())
    , m_2(m_voxels->GetDimensions() - 1)
    , m_params(params)
    , m_callbacks(callbacks)
{
    BuildVoxelMesh();
    BuildRaycastMesh(); // build a raycast mesh of the voxel mesh
    ComputeConvexHull();
}

void VoxelHull::MinMaxVoxelRegion(const Voxel& v)
{
    VHACD::Vector3<uint32_t> x = v.GetVoxel();
    m_1 = m_1.CWiseMin(x);
    m_2 = m_2.CWiseMax(x);
}

void VoxelHull::BuildRaycastMesh()
{
    // Create a raycast mesh representation of the voxelized surface mesh
    if ( !m_indices.empty() )
    {
        m_AABBTree = AABBTree(m_vertices,
                              m_indices);
    }
}

void VoxelHull::ComputeConvexHull()
{
    if ( !m_vertices.empty() )
    {
        // we compute the convex hull as follows...
        VHACD::QuickHull qh;
        uint32_t tcount = qh.ComputeConvexHull(m_vertices,
                                               uint32_t(m_vertices.size()));
        if ( tcount )
        {
            m_convexHull = std::unique_ptr<IVHACD::ConvexHull>(new IVHACD::ConvexHull);

            m_convexHull->m_points = qh.GetVertices();
            m_convexHull->m_triangles = qh.GetIndices();

            VHACD::ComputeCentroid(m_convexHull->m_points,
                                   m_convexHull->m_triangles,
                                   m_convexHull->m_center);
            m_convexHull->m_volume = VHACD::ComputeMeshVolume(m_convexHull->m_points,
                                                              m_convexHull->m_triangles);
        }
    }
    if ( m_convexHull )
    {
        m_hullVolume = m_convexHull->m_volume;
    }
    // This is the volume of a single voxel
    double singleVoxelVolume = m_voxelScale * m_voxelScale * m_voxelScale;
    size_t voxelCount = m_interiorVoxels.size() + m_newSurfaceVoxels.size() + m_surfaceVoxels.size();
    m_voxelVolume = singleVoxelVolume * double(voxelCount);
    double diff = fabs(m_hullVolume - m_voxelVolume);
    m_volumeError = (diff * 100) / m_voxelVolume;
}

bool VoxelHull::IsComplete()
{
    bool ret = false;
    if ( m_convexHull == nullptr )
    {
        ret = true;
    }
    else if ( m_volumeError < m_params.m_minimumVolumePercentErrorAllowed )
    {
        ret = true;
    }
    else if ( m_depth > m_params.m_maxRecursionDepth )
    {
        ret = true;
    }
    else
    {
        // We compute the voxel width on all 3 axes and see if they are below the min threshold size
        VHACD::Vector3<uint32_t> d = m_2 - m_1;
        if ( d.GetX() <= m_params.m_minEdgeLength &&
             d.GetY() <= m_params.m_minEdgeLength &&
             d.GetZ() <= m_params.m_minEdgeLength )
        {
            ret = true;
        }
    }
    return ret;
}

VHACD::Vect3 VoxelHull::GetPoint(const int32_t x,
                                 const int32_t y,
                                 const int32_t z,
                                 const double scale,
                                 const VHACD::Vect3& bmin) const
{
    return VHACD::Vect3(x * scale + bmin.GetX(),
                        y * scale + bmin.GetY(),
                        z * scale + bmin.GetZ());
}

uint32_t VoxelHull::GetVertexIndex(const VHACD::Vector3<uint32_t>& p)
{
    uint32_t ret = 0;
    uint32_t address = (p.GetX() << 20) | (p.GetY() << 10) | p.GetZ();
    auto found = m_voxelIndexMap.find(address);
    if ( found != m_voxelIndexMap.end() )
    {
        ret = found->second;
    }
    else
    {
        VHACD::Vect3 vertex = GetPoint(p.GetX(),
                                       p.GetY(),
                                       p.GetZ(),
                                       m_voxelScale,
                                       m_voxelAdjust);
        ret = uint32_t(m_voxelIndexMap.size());
        m_voxelIndexMap[address] = ret;
        m_vertices.emplace_back(vertex);
    }
    return ret;
}

void VoxelHull::BuildVoxelMesh()
{
    // When we build the triangle mesh we do *not* need the interior voxels, only the ones
    // which lie upon the logical surface of the mesh.
    // Each time we perform a plane split, voxels which are along the splitting plane become
    // 'new surface voxels'.

    for (auto& i : m_surfaceVoxels)
    {
        AddVoxelBox(i);
    }
    for (auto& i : m_newSurfaceVoxels)
    {
        AddVoxelBox(i);
    }
}

void VoxelHull::AddVoxelBox(const Voxel &v)
{
    // The voxel position of the upper left corner of the box
    VHACD::Vector3<uint32_t> bmin(v.GetX(),
                                  v.GetY(),
                                  v.GetZ());
    // The voxel position of the lower right corner of the box
    VHACD::Vector3<uint32_t> bmax(bmin.GetX() + 1,
                                  bmin.GetY() + 1,
                                  bmin.GetZ() + 1);

    // Build the set of 8 voxel positions representing
    // the coordinates of the box
    std::array<VHACD::Vector3<uint32_t>, 8> box{{
        { bmin.GetX(), bmin.GetY(), bmin.GetZ() },
        { bmax.GetX(), bmin.GetY(), bmin.GetZ() },
        { bmax.GetX(), bmax.GetY(), bmin.GetZ() },
        { bmin.GetX(), bmax.GetY(), bmin.GetZ() },
        { bmin.GetX(), bmin.GetY(), bmax.GetZ() },
        { bmax.GetX(), bmin.GetY(), bmax.GetZ() },
        { bmax.GetX(), bmax.GetY(), bmax.GetZ() },
        { bmin.GetX(), bmax.GetY(), bmax.GetZ() }
    }};

    // Now add the 12 triangles comprising the 3d box
    AddTri(box, 2, 1, 0);
    AddTri(box, 3, 2, 0);

    AddTri(box, 7, 2, 3);
    AddTri(box, 7, 6, 2);

    AddTri(box, 5, 1, 2);
    AddTri(box, 5, 2, 6);

    AddTri(box, 5, 4, 1);
    AddTri(box, 4, 0, 1);

    AddTri(box, 4, 6, 7);
    AddTri(box, 4, 5, 6);

    AddTri(box, 4, 7, 0);
    AddTri(box, 7, 3, 0);
}

void VoxelHull::AddTri(const std::array<VHACD::Vector3<uint32_t>, 8>& box,
                       uint32_t i1,
                       uint32_t i2,
                       uint32_t i3)
{
    AddTriangle(box[i1], box[i2], box[i3]);
}

void VoxelHull::AddTriangle(const VHACD::Vector3<uint32_t>& p1,
                            const VHACD::Vector3<uint32_t>& p2,
                            const VHACD::Vector3<uint32_t>& p3)
{
    uint32_t i1 = GetVertexIndex(p1);
    uint32_t i2 = GetVertexIndex(p2);
    uint32_t i3 = GetVertexIndex(p3);

    m_indices.emplace_back(i1, i2, i3);
}

SplitAxis VoxelHull::ComputeSplitPlane(uint32_t& location)
{
    SplitAxis ret = SplitAxis::X_AXIS_NEGATIVE;

    VHACD::Vector3<uint32_t> d = m_2 - m_1;

    if ( d.GetX() >= d.GetY() && d.GetX() >= d.GetZ() )
    {
        ret = SplitAxis::X_AXIS_NEGATIVE;
        location = (m_2.GetX() + 1 + m_1.GetX()) / 2;
        uint32_t edgeLoc;
        if ( m_params.m_findBestPlane && FindConcavityX(edgeLoc) )
        {
            location = edgeLoc;
        }
    }
    else if ( d.GetY() >= d.GetX() && d.GetY() >= d.GetZ() )
    {
        ret = SplitAxis::Y_AXIS_NEGATIVE;
        location = (m_2.GetY() + 1 + m_1.GetY()) / 2;
        uint32_t edgeLoc;
        if ( m_params.m_findBestPlane && FindConcavityY(edgeLoc) )
        {
            location = edgeLoc;
        }
    }
    else
    {
        ret = SplitAxis::Z_AXIS_NEGATIVE;
        location = (m_2.GetZ() + 1 + m_1.GetZ()) / 2;
        uint32_t edgeLoc;
        if ( m_params.m_findBestPlane && FindConcavityZ(edgeLoc) )
        {
            location = edgeLoc;
        }
    }

    return ret;
}

VHACD::Vect3 VoxelHull::GetPosition(const VHACD::Vector3<int32_t>& ip) const
{
    return GetPoint(ip.GetX(),
                    ip.GetY(),
                    ip.GetZ(),
                    m_voxelScale,
                    m_voxelAdjust);
}

double VoxelHull::Raycast(const VHACD::Vector3<int32_t>& p1,
                          const VHACD::Vector3<int32_t>& p2) const
{
    double ret;
    VHACD::Vect3 from = GetPosition(p1);
    VHACD::Vect3 to = GetPosition(p2);

    double outT;
    double faceSign;
    VHACD::Vect3 hitLocation;
    if (m_AABBTree.TraceRay(from, to, outT, faceSign, hitLocation))
    {
        ret = (from - hitLocation).GetNorm();
    }
    else
    {
        ret = 0; // if it doesn't hit anything, just assign it to zero.
    }

    return ret;
}

bool VoxelHull::FindConcavity(uint32_t idx,
                              uint32_t& splitLoc)
{
    bool ret = false;

    int32_t d = (m_2[idx] - m_1[idx]) + 1; // The length of the getX axis in voxel space

    uint32_t idx1;
    uint32_t idx2;
    uint32_t idx3;
    switch (idx)
    {
        case 0: // X
            idx1 = 0;
            idx2 = 1;
            idx3 = 2;
            break;
        case 1: // Y
            idx1 = 1;
            idx2 = 0;
            idx3 = 2;
            break;
        case 2:
            idx1 = 2;
            idx2 = 1;
            idx3 = 0;
            break;
        default:
            /*
                * To silence uninitialized variable warnings
                */
            idx1 = 0;
            idx2 = 0;
            idx3 = 0;
            assert(0 && "findConcavity::idx must be 0, 1, or 2");
            break;
    }

    // We will compute the edge error on the XY plane and the XZ plane
    // searching for the greatest location of concavity
    std::vector<double> edgeError1 = std::vector<double>(d);
    std::vector<double> edgeError2 = std::vector<double>(d);

    // Counter of number of voxel samples on the XY plane we have accumulated
    uint32_t index1 = 0;

    // Compute Edge Error on the XY plane
    for (uint32_t i0 = m_1[idx1]; i0 <= m_2[idx1]; i0++)
    {
        double errorTotal = 0;
        // We now perform a raycast from the sides inward on the XY plane to
        // determine the total error (distance of the surface from the sides)
        // along this getX position.
        for (uint32_t i1 = m_1[idx2]; i1 <= m_2[idx2]; i1++)
        {
            VHACD::Vector3<int32_t> p1;
            VHACD::Vector3<int32_t> p2;
            switch (idx)
            {
                case 0:
                {
                    p1 = VHACD::Vector3<int32_t>(i0, i1, m_1.GetZ() - 2);
                    p2 = VHACD::Vector3<int32_t>(i0, i1, m_2.GetZ() + 2);
                    break;
                }
                case 1:
                {
                    p1 = VHACD::Vector3<int32_t>(i1, i0, m_1.GetZ() - 2);
                    p2 = VHACD::Vector3<int32_t>(i1, i0, m_2.GetZ() + 2);
                    break;
                }
                case 2:
                {
                    p1 = VHACD::Vector3<int32_t>(m_1.GetX() - 2, i1, i0);
                    p2 = VHACD::Vector3<int32_t>(m_2.GetX() + 2, i1, i0);
                    break;
                }
            }

            double e1 = Raycast(p1, p2);
            double e2 = Raycast(p2, p1);

            errorTotal = errorTotal + e1 + e2;
        }
        // The total amount of edge error along this voxel location
        edgeError1[index1] = errorTotal;
        index1++;
    }

    // Compute edge error along the XZ plane
    uint32_t index2 = 0;

    for (uint32_t i0 = m_1[idx1]; i0 <= m_2[idx1]; i0++)
    {
        double errorTotal = 0;

        for (uint32_t i1 = m_1[idx3]; i1 <= m_2[idx3]; i1++)
        {
            VHACD::Vector3<int32_t> p1;
            VHACD::Vector3<int32_t> p2;
            switch (idx)
            {
                case 0:
                {
                    p1 = VHACD::Vector3<int32_t>(i0, m_1.GetY() - 2, i1);
                    p2 = VHACD::Vector3<int32_t>(i0, m_2.GetY() + 2, i1);
                    break;
                }
                case 1:
                {
                    p1 = VHACD::Vector3<int32_t>(m_1.GetX() - 2, i0, i1);
                    p2 = VHACD::Vector3<int32_t>(m_2.GetX() + 2, i0, i1);
                    break;
                }
                case 2:
                {
                    p1 = VHACD::Vector3<int32_t>(i1, m_1.GetY() - 2, i0);
                    p2 = VHACD::Vector3<int32_t>(i1, m_2.GetY() + 2, i0);
                    break;
                }
            }

            double e1 = Raycast(p1, p2); // raycast from one side to the interior
            double e2 = Raycast(p2, p1); // raycast from the other side to the interior

            errorTotal = errorTotal + e1 + e2;
        }
        edgeError2[index2] = errorTotal;
        index2++;
    }


    // we now compute the first derivative to find the greatest spot of concavity on the XY plane
    double maxDiff = 0;
    uint32_t maxC = 0;
    for (uint32_t x = 1; x < index1; x++)
    {
        if ( edgeError1[x] > 0 &&  edgeError1[x - 1] > 0 )
        {
            double diff = abs(edgeError1[x] - edgeError1[x - 1]);
            if ( diff > maxDiff )
            {
                maxDiff = diff;
                maxC = x-1;
            }
        }
    }

    // Now see if there is a greater concavity on the XZ plane
    for (uint32_t x = 1; x < index2; x++)
    {
        if ( edgeError2[x] > 0 && edgeError2[x - 1] > 0 )
        {
            double diff = abs(edgeError2[x] - edgeError2[x - 1]);
            if ( diff > maxDiff )
            {
                maxDiff = diff;
                maxC = x - 1;
            }
        }
    }

    splitLoc = maxC + m_1[idx1];

    // we do not allow an edge split if it is too close to the ends
    if (    splitLoc > (m_1[idx1] + 4)
         && splitLoc < (m_2[idx1] - 4) )
    {
        ret = true;
    }

    return ret;
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityX(uint32_t& splitLoc)
{
    return FindConcavity(0, splitLoc);
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityY(uint32_t& splitLoc)
{
    return FindConcavity(1, splitLoc);
}

// Finding the greatest area of concavity..
bool VoxelHull::FindConcavityZ(uint32_t &splitLoc)
{
    return FindConcavity(2, splitLoc);
}

void VoxelHull::PerformPlaneSplit()
{
    if ( IsComplete() )
    {
    }
    else
    {
        uint32_t splitLoc;
        SplitAxis axis = ComputeSplitPlane(splitLoc);
        switch ( axis )
        {
            case SplitAxis::X_AXIS_NEGATIVE:
            case SplitAxis::X_AXIS_POSITIVE:
                // Split on the getX axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_POSITIVE, splitLoc));
                break;
            case SplitAxis::Y_AXIS_NEGATIVE:
            case SplitAxis::Y_AXIS_POSITIVE:
                // Split on the 1 axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_POSITIVE, splitLoc));
                break;
            case SplitAxis::Z_AXIS_NEGATIVE:
            case SplitAxis::Z_AXIS_POSITIVE:
                // Split on the getZ axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_POSITIVE, splitLoc));
                break;
        }
    }
}

void VoxelHull::SaveVoxelMesh(const SimpleMesh &inputMesh,
                              bool saveVoxelMesh,
                              bool saveSourceMesh)
{
    char scratch[512];
    snprintf(scratch,
             sizeof(scratch),
             "voxel-mesh-%03d.obj",
             m_index);
    FILE *fph = fopen(scratch,
                      "wb");
    if ( fph )
    {
        uint32_t baseIndex = 1;
        if ( saveVoxelMesh )
        {
            WriteOBJ(fph,
                     m_vertices,
                     m_indices,
                     baseIndex);
            baseIndex += uint32_t(m_vertices.size());
        }
        if ( saveSourceMesh )
        {
            WriteOBJ(fph,
                     inputMesh.m_vertices,
                     inputMesh.m_indices,
                     baseIndex);
        }
        fclose(fph);
    }
}

void VoxelHull::SaveOBJ(const char* fname,
                        const VoxelHull* h)
{
    FILE *fph = fopen(fname,"wb");
    if ( fph )
    {
        uint32_t baseIndex = 1;
        WriteOBJ(fph,
                 m_vertices,
                 m_indices,
                 baseIndex);

        baseIndex += uint32_t(m_vertices.size());

        WriteOBJ(fph,
                 h->m_vertices,
                 h->m_indices,
                 baseIndex);
        fclose(fph);
    }
}

void VoxelHull::SaveOBJ(const char *fname)
{
    FILE *fph = fopen(fname, "wb");
    if ( fph )
    {
        printf("Saving '%s' with %d vertices and %d triangles\n",
                fname,
                uint32_t(m_vertices.size()),
                uint32_t(m_indices.size()));
        WriteOBJ(fph,
                 m_vertices,
                 m_indices,
                 1);
        fclose(fph);
    }
}

void VoxelHull::WriteOBJ(FILE* fph,
                         const std::vector<VHACD::Vertex>& vertices,
                         const std::vector<VHACD::Triangle>& indices,
                         uint32_t baseIndex)
{
    if (!fph)
    {
        return;
    }

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const VHACD::Vertex& v = vertices[i];
        fprintf(fph, "v %0.9f %0.9f %0.9f\n",
                v.mX,
                v.mY,
                v.mZ);
    }

    for (size_t i = 0; i < indices.size(); ++i)
    {
        const VHACD::Triangle& t = indices[i];
        fprintf(fph, "f %d %d %d\n",
                t.mI0 + baseIndex,
                t.mI1 + baseIndex,
                t.mI2 + baseIndex);
    }
}

class VHACDImpl;

// This class represents a single task to compute the volume error
// of two convex hulls combined
class CostTask
{
public:
    VHACDImpl*          m_this{ nullptr };
    IVHACD::ConvexHull* m_hullA{ nullptr };
    IVHACD::ConvexHull* m_hullB{ nullptr };
    double              m_concavity{ 0 }; // concavity of the two combined
    std::future<void>   m_future;
};

class HullPair
{
public:
    HullPair() = default;
    HullPair(uint32_t hullA,
             uint32_t hullB,
             double concavity);

    bool operator<(const HullPair &h) const;

    uint32_t    m_hullA{ 0 };
    uint32_t    m_hullB{ 0 };
    double      m_concavity{ 0 };
};

HullPair::HullPair(uint32_t hullA,
                   uint32_t hullB,
                   double concavity)
    : m_hullA(hullA)
    , m_hullB(hullB)
    , m_concavity(concavity)
{
}

bool HullPair::operator<(const HullPair &h) const
{
    return m_concavity > h.m_concavity ? true : false;
}

// void jobCallback(void* userPtr);

class VHACDImpl : public IVHACD, public VHACDCallbacks
{
    // Don't consider more than 100,000 convex hulls.
    static constexpr uint32_t MaxConvexHullFragments{ 100000 };
public:
    VHACDImpl() = default;

    /*
     * Overrides VHACD::IVHACD
     */
    ~VHACDImpl() override
    {
        Clean();
    }

    void Cancel() override final;

    bool Compute(const float* const points,
                 const uint32_t countPoints,
                 const uint32_t* const triangles,
                 const uint32_t countTriangles,
                 const Parameters& params) override final;

    bool Compute(const double* const points,
                 const uint32_t countPoints,
                 const uint32_t* const triangles,
                 const uint32_t countTriangles,
                 const Parameters& params) override final;

    uint32_t GetNConvexHulls() const override final;

    bool GetConvexHull(const uint32_t index,
                       ConvexHull& ch) const override final;

    void Clean() override final;  // release internally allocated memory

    void Release() override final;

    // Will compute the center of mass of the convex hull decomposition results and return it
    // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
    bool ComputeCenterOfMass(double centerOfMass[3]) const override final;

    // In synchronous mode (non-multi-threaded) the state is always 'ready'
    // In asynchronous mode, this returns true if the background thread is not still actively computing
    // a new solution.  In an asynchronous config the 'IsReady' call will report any update or log
    // messages in the caller's current thread.
    bool IsReady(void) const override final;

    /**
    * At the request of LegionFu : out_look@foxmail.com
    * This method will return which convex hull is closest to the source position.
    * You can use this method to figure out, for example, which vertices in the original
    * source mesh are best associated with which convex hull.
    * 
    * @param pos : The input 3d position to test against
    * 
    * @return : Returns which convex hull this position is closest to.
    */
    uint32_t findNearestConvexHull(const double pos[3],
                                   double& distanceToHull) override final;

// private:
    bool Compute(const std::vector<VHACD::Vertex>& points,
                 const std::vector<VHACD::Triangle>& triangles,
                 const Parameters& params);

    // Take the source position, normalize it, and then convert it into an index position
    uint32_t GetIndex(VHACD::VertexIndex& vi,
                      const VHACD::Vertex& p);

    // This copies the input mesh while scaling the input positions
    // to fit into a normalized unit cube. It also re-indexes all of the
    // vertex positions in case they weren't clean coming in. 
    void CopyInputMesh(const std::vector<VHACD::Vertex>& points,
                       const std::vector<VHACD::Triangle>& triangles);

    void ScaleOutputConvexHull(ConvexHull &ch);

    void AddCostToPriorityQueue(CostTask& task);

    void ReleaseConvexHull(ConvexHull* ch);

    void PerformConvexDecomposition();

    double ComputeConvexHullVolume(const ConvexHull& sm);

    double ComputeVolume4(const VHACD::Vect3& a,
                          const VHACD::Vect3& b,
                          const VHACD::Vect3& c,
                          const VHACD::Vect3& d);

    double ComputeConcavity(double volumeSeparate,
                            double volumeCombined,
                            double volumeMesh);

    // See if we can compute the cost without having to actually merge convex hulls.
    // If the axis aligned bounding boxes (slightly inflated) of the two convex hulls
    // do not intersect, then we don't need to actually compute the merged convex hull
    // volume.
    bool DoFastCost(CostTask& mt);

    void PerformMergeCostTask(CostTask& mt);

    ConvexHull* ComputeReducedConvexHull(const ConvexHull& ch,
                                         uint32_t maxVerts,
                                         bool projectHullVertices);

    // Take the points in convex hull A and the points in convex hull B and generate
    // a new convex hull on the combined set of points.
    // Once completed, we create a SimpleMesh instance to hold the triangle mesh
    // and we compute an inflated AABB for it.
    ConvexHull* ComputeCombinedConvexHull(const ConvexHull& sm1,
                                          const ConvexHull& sm2);


    ConvexHull* GetHull(uint32_t index);

    bool RemoveHull(uint32_t index);

    ConvexHull* CopyConvexHull(const ConvexHull& source);

    const char* GetStageName(Stages stage) const;

    /*
     * Overrides VHACD::VHACDCallbacks
     */
    void ProgressUpdate(Stages stage,
                        double stageProgress,
                        const char* operation) override final;

    bool IsCanceled() const override final;

    std::atomic<bool>                                   m_canceled{ false };
    Parameters                                          m_params; // Convex decomposition parameters

    std::vector<IVHACD::ConvexHull*>                    m_convexHulls; // Finalized convex hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_voxelHulls; // completed voxel hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_pendingHulls;

    std::vector<std::unique_ptr<AABBTree>>              m_trees;
    VHACD::AABBTree                                     m_AABBTree;
    VHACD::Volume                                       m_voxelize;
    VHACD::Vect3                                        m_center;
    double                                              m_scale{ double(1.0) };
    double                                              m_recipScale{ double(1.0) };
    SimpleMesh                                          m_inputMesh; // re-indexed and normalized input mesh
    std::vector<VHACD::Vertex>                          m_vertices;
    std::vector<VHACD::Triangle>                        m_indices;

    double                                              m_overallHullVolume{ double(0.0) };
    double                                              m_voxelScale{ double(0.0) };
    double                                              m_voxelHalfScale{ double(0.0) };
    VHACD::Vect3                                        m_voxelBmin;
    VHACD::Vect3                                        m_voxelBmax;
    uint32_t                                            m_meshId{ 0 };
    std::priority_queue<HullPair>                       m_hullPairQueue;
#if !VHACD_DISABLE_THREADING
    std::unique_ptr<ThreadPool>                         m_threadPool{ nullptr };
#endif
    std::unordered_map<uint32_t, IVHACD::ConvexHull*>   m_hulls;

    double                                              m_overallProgress{ double(0.0) };
    double                                              m_stageProgress{ double(0.0) };
    double                                              m_operationProgress{ double(0.0) };
};

void VHACDImpl::Cancel()
{
    m_canceled = true;
}

bool VHACDImpl::Compute(const float* const points,
                        const uint32_t countPoints,
                        const uint32_t* const triangles,
                        const uint32_t countTriangles,
                        const Parameters& params)
{
    std::vector<VHACD::Vertex> v;
    v.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        v.emplace_back(points[i * 3 + 0],
                       points[i * 3 + 1],
                       points[i * 3 + 2]);
    }

    std::vector<VHACD::Triangle> t;
    t.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        t.emplace_back(triangles[i * 3 + 0],
                       triangles[i * 3 + 1],
                       triangles[i * 3 + 2]);
    }

    return Compute(v, t, params);
}

bool VHACDImpl::Compute(const double* const points,
                        const uint32_t countPoints,
                        const uint32_t* const triangles,
                        const uint32_t countTriangles,
                        const Parameters& params)
{
    std::vector<VHACD::Vertex> v;
    v.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        v.emplace_back(points[i * 3 + 0],
                       points[i * 3 + 1],
                       points[i * 3 + 2]);
    }

    std::vector<VHACD::Triangle> t;
    t.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        t.emplace_back(triangles[i * 3 + 0],
                       triangles[i * 3 + 1],
                       triangles[i * 3 + 2]);
    }

    return Compute(v, t, params);
}

uint32_t VHACDImpl::GetNConvexHulls() const
{
    return uint32_t(m_convexHulls.size());
}

bool VHACDImpl::GetConvexHull(const uint32_t index,
                              ConvexHull& ch) const
{
    bool ret = false;

    if ( index < uint32_t(m_convexHulls.size() ))
    {
        ch = *m_convexHulls[index];
        ret = true;
    }

    return ret;
}

void VHACDImpl::Clean()
{
#if !VHACD_DISABLE_THREADING
    m_threadPool = nullptr;
#endif

    m_trees.clear();

    for (auto& ch : m_convexHulls)
    {
        ReleaseConvexHull(ch);
    }
    m_convexHulls.clear();

    for (auto& ch : m_hulls)
    {
        ReleaseConvexHull(ch.second);
    }
    m_hulls.clear();

    m_voxelHulls.clear();

    m_pendingHulls.clear();

    m_vertices.clear();
    m_indices.clear();
}

void VHACDImpl::Release()
{
    delete this;
}

bool VHACDImpl::ComputeCenterOfMass(double centerOfMass[3]) const
{
    bool ret = false;

    return ret;
}

bool VHACDImpl::IsReady() const
{
    return true;
}

uint32_t VHACDImpl::findNearestConvexHull(const double pos[3],
                                          double& distanceToHull)
{
    uint32_t ret = 0; // The default return code is zero

    uint32_t hullCount = GetNConvexHulls();
    distanceToHull = 0;
    // First, make sure that we have valid and completed results
    if ( hullCount )
    {
        // See if we already have AABB trees created for each convex hull
        if ( m_trees.empty() )
        {
            // For each convex hull, we generate an AABB tree for fast closest point queries
            for (uint32_t i = 0; i < hullCount; i++)
            {
                VHACD::IVHACD::ConvexHull ch;
                GetConvexHull(i,ch);
                // Pass the triangle mesh to create an AABB tree instance based on it.
                m_trees.emplace_back(new AABBTree(ch.m_points,
                                                  ch.m_triangles));
            }
        }
        // We now compute the closest point to each convex hull and save the nearest one
        double closest = 1e99;
        for (uint32_t i = 0; i < hullCount; i++)
        {
            std::unique_ptr<AABBTree>& t = m_trees[i];
            if ( t )
            {
                VHACD::Vect3 closestPoint;
                VHACD::Vect3 position(pos[0],
                                      pos[1],
                                      pos[2]);
                if ( t->GetClosestPointWithinDistance(position, 1e99, closestPoint))
                {
                    VHACD::Vect3 d = position - closestPoint;
                    double distanceSquared = d.GetNormSquared();
                    if ( distanceSquared < closest )
                    {
                        closest = distanceSquared;
                        ret = i;
                    }
                }
            }
        }
        distanceToHull = sqrt(closest); // compute the distance to the nearest convex hull
    }

    return ret;
}

bool VHACDImpl::Compute(const std::vector<VHACD::Vertex>& points,
                        const std::vector<VHACD::Triangle>& triangles,
                        const Parameters& params)
{
    bool ret = false;

    m_params = params;
    m_canceled = false;

    Clean(); // release any previous results
#if !VHACD_DISABLE_THREADING
    if ( m_params.m_asyncACD )
    {
        m_threadPool = std::unique_ptr<ThreadPool>(new ThreadPool(8));
    }
#endif
    CopyInputMesh(points,
                  triangles);
    if ( !m_canceled )
    {
        // We now recursively perform convex decomposition until complete
        PerformConvexDecomposition();
    }

    if ( m_canceled )
    {
        Clean();
        ret = false;
        if ( m_params.m_logger )
        {
            m_params.m_logger->Log("VHACD operation canceled before it was complete.");
        }
    }
    else
    {
        ret = true;
    }
#if !VHACD_DISABLE_THREADING
    m_threadPool = nullptr;
#endif
    return ret;
}

uint32_t VHACDImpl::GetIndex(VHACD::VertexIndex& vi,
                             const VHACD::Vertex& p)
{
    VHACD::Vect3 pos = (VHACD::Vect3(p) - m_center) * m_recipScale;
    bool newPos;
    uint32_t ret = vi.GetIndex(pos,
                               newPos);
    return ret;
}

void VHACDImpl::CopyInputMesh(const std::vector<VHACD::Vertex>& points,
                              const std::vector<VHACD::Triangle>& triangles)
{
    m_vertices.clear();
    m_indices.clear();
    m_indices.reserve(triangles.size());

    // First we must find the bounding box of this input vertices and normalize them into a unit-cube
    VHACD::Vect3 bmin( FLT_MAX);
    VHACD::Vect3 bmax(-FLT_MAX);
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
                   0,
                   "ComputingBounds");
    for (uint32_t i = 0; i < points.size(); i++)
    {
        const VHACD::Vertex& p = points[i];

        bmin = bmin.CWiseMin(p);
        bmax = bmax.CWiseMax(p);
    }
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
                   100,
                   "ComputingBounds");

    m_center = (bmax + bmin) * double(0.5);

    VHACD::Vect3 scale = bmax - bmin;
    m_scale = scale.MaxCoeff();

    m_recipScale = m_scale > double(0.0) ? double(1.0) / m_scale : double(0.0);

    {
        VHACD::VertexIndex vi = VHACD::VertexIndex(double(0.001), false);

        uint32_t dcount = 0;

        for (uint32_t i = 0; i < triangles.size() && !m_canceled; ++i)
        {
            const VHACD::Triangle& t = triangles[i];
            const VHACD::Vertex& p1 = points[t.mI0];
            const VHACD::Vertex& p2 = points[t.mI1];
            const VHACD::Vertex& p3 = points[t.mI2];

            uint32_t i1 = GetIndex(vi, p1);
            uint32_t i2 = GetIndex(vi, p2);
            uint32_t i3 = GetIndex(vi, p3);

            if ( i1 == i2 || i1 == i3 || i2 == i3 )
            {
                dcount++;
            }
            else
            {
                m_indices.emplace_back(i1, i2, i3);
            }
        }

        if ( dcount )
        {
            if ( m_params.m_logger )
            {
                char scratch[512];
                snprintf(scratch,
                         sizeof(scratch),
                         "Skipped %d degenerate triangles", dcount);
                m_params.m_logger->Log(scratch);
            }
        }

        m_vertices = vi.TakeVertices();
    }

    // Create the raycast mesh
    if ( !m_canceled )
    {
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
                       0,
                       "Building RaycastMesh");
        m_AABBTree = VHACD::AABBTree(m_vertices,
                                     m_indices);
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
                       100,
                       "RaycastMesh completed");
    }
    if ( !m_canceled )
    {
        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
                        0,
                        "Voxelizing Input Mesh");
        m_voxelize = VHACD::Volume();
        m_voxelize.Voxelize(m_vertices,
                            m_indices,
                            m_params.m_resolution,
                            m_params.m_fillMode,
                            m_AABBTree);
        m_voxelScale = m_voxelize.GetScale();
        m_voxelHalfScale = m_voxelScale * double(0.5);
        m_voxelBmin = m_voxelize.GetBounds().GetMin();
        m_voxelBmax = m_voxelize.GetBounds().GetMax();
        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
                       100,
                       "Voxelization complete");
    }

    m_inputMesh.m_vertices = m_vertices;
    m_inputMesh.m_indices = m_indices;
    if ( !m_canceled )
    {
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
                        0,
                        "Build initial ConvexHull");
        std::unique_ptr<VoxelHull> vh = std::unique_ptr<VoxelHull>(new VoxelHull(m_voxelize,
                                                                                 m_params,
                                                                                 this));
        if ( vh->m_convexHull )
        {
            m_overallHullVolume = vh->m_convexHull->m_volume;
        }
        m_pendingHulls.push_back(std::move(vh));
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
                       100,
                       "Initial ConvexHull complete");
    }
}

void VHACDImpl::ScaleOutputConvexHull(ConvexHull& ch)
{
    for (uint32_t i = 0; i < ch.m_points.size(); i++)
    {
        VHACD::Vect3 p = ch.m_points[i];
        p = (p * m_scale) + m_center;
        ch.m_points[i] = p;
    }
    ch.m_volume = ComputeConvexHullVolume(ch); // get the combined volume
    VHACD::BoundsAABB b(ch.m_points);
    ch.mBmin = b.GetMin();
    ch.mBmax = b.GetMax();
    ComputeCentroid(ch.m_points,
                    ch.m_triangles,
                    ch.m_center);
}

void VHACDImpl::AddCostToPriorityQueue(CostTask& task)
{
    HullPair hp(task.m_hullA->m_meshId,
                task.m_hullB->m_meshId,
                task.m_concavity);
    m_hullPairQueue.push(hp);
}

void VHACDImpl::ReleaseConvexHull(ConvexHull* ch)
{
    if ( ch )
    {
        delete ch;
    }
}

void jobCallback(std::unique_ptr<VoxelHull>& userPtr)
{
    userPtr->PerformPlaneSplit();
}

void computeMergeCostTask(CostTask& ptr)
{
    ptr.m_this->PerformMergeCostTask(ptr);
}

void VHACDImpl::PerformConvexDecomposition()
{
    {
        ScopedTime st("Convex Decomposition",
                      m_params.m_logger);
        double maxHulls = pow(2, m_params.m_maxRecursionDepth);
        // We recursively split convex hulls until we can
        // no longer recurse further.
        Timer t;

        while ( !m_pendingHulls.empty() && !m_canceled )
        {
            size_t count = m_pendingHulls.size() + m_voxelHulls.size();
            double e = t.PeekElapsedSeconds();
            if ( e >= double(0.1) )
            {
                t.Reset();
                double stageProgress = (double(count) * double(100.0)) / maxHulls;
                ProgressUpdate(Stages::PERFORMING_DECOMPOSITION,
                               stageProgress,
                               "Performing recursive decomposition of convex hulls");
            }
            // First we make a copy of the hulls we are processing
            std::vector<std::unique_ptr<VoxelHull>> oldList = std::move(m_pendingHulls);
            // For each hull we want to split, we either
            // immediately perform the plane split or we post it as
            // a job to be performed in a background thread
            std::vector<std::future<void>> futures(oldList.size());
            uint32_t futureCount = 0;
            for (auto& i : oldList)
            {
                if ( i->IsComplete() || count > MaxConvexHullFragments )
                {
                }
                else
                {
#if !VHACD_DISABLE_THREADING
                    if ( m_threadPool )
                    {
                        futures[futureCount] = m_threadPool->enqueue([&i]
                        {
                            jobCallback(i);
                        });
                        futureCount++;
                    }
                    else
#endif
                    {
                        i->PerformPlaneSplit();
                    }
                }
            }
            // Wait for any outstanding jobs to complete in the background threads
            if ( futureCount )
            {
                for (uint32_t i = 0; i < futureCount; i++)
                {
                    futures[i].get();
                }
            }
            // Now, we rebuild the pending convex hulls list by
            // adding the two children to the output list if
            // we need to recurse them further
            for (auto& vh : oldList)
            {
                if ( vh->IsComplete() || count > MaxConvexHullFragments )
                {
                    if ( vh->m_convexHull )
                    {
                        m_voxelHulls.push_back(std::move(vh));
                    }
                }
                else
                {
                    if ( vh->m_hullA )
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullA));
                    }
                    if ( vh->m_hullB )
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullB));
                    }
                }
            }
        }
    }

    if ( !m_canceled )
    {
        // Give each convex hull a unique guid
        m_meshId = 0;
        m_hulls.clear();

        // Build the convex hull id map
        std::vector<ConvexHull*> hulls;

        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
                       0,
                       "Initializing ConvexHulls");
        for (auto& vh : m_voxelHulls)
        {
            if ( m_canceled )
            {
                break;
            }
            ConvexHull* ch = CopyConvexHull(*vh->m_convexHull);
            m_meshId++;
            ch->m_meshId = m_meshId;
            m_hulls[m_meshId] = ch;
            // Compute the volume of the convex hull
            ch->m_volume = ComputeConvexHullVolume(*ch);
            // Compute the AABB of the convex hull
            VHACD::BoundsAABB b = VHACD::BoundsAABB(ch->m_points).Inflate(double(0.1));
            ch->mBmin = b.GetMin();
            ch->mBmax = b.GetMax();

            ComputeCentroid(ch->m_points,
                            ch->m_triangles,
                            ch->m_center);

            hulls.push_back(ch);
        }
        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
                        100,
                        "ConvexHull initialization complete");

        m_voxelHulls.clear();

        // here we merge convex hulls as needed until the match the
        // desired maximum hull count.
        size_t hullCount = hulls.size();

        if ( hullCount > m_params.m_maxConvexHulls && !m_canceled)
        {
            size_t costMatrixSize = ((hullCount * hullCount) - hullCount) >> 1;
            std::vector<CostTask> tasks;
            tasks.reserve(costMatrixSize);

            ScopedTime st("Computing the Cost Matrix",
                          m_params.m_logger);
            // First thing we need to do is compute the cost matrix
            // This is computed as the volume error of any two convex hulls
            // combined
            ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                           0,
                           "Computing Hull Merge Cost Matrix");
            for (size_t i = 1; i < hullCount && !m_canceled; i++)
            {
                ConvexHull* chA = hulls[i];

                for (size_t j = 0; j < i && !m_canceled; j++)
                {
                    ConvexHull* chB = hulls[j];

                    CostTask t;
                    t.m_hullA = chA;
                    t.m_hullB = chB;
                    t.m_this = this;

                    if ( DoFastCost(t) )
                    {
                    }
                    else
                    {
                        tasks.push_back(std::move(t));
                        CostTask* task = &tasks.back();
#if !VHACD_DISABLE_THREADING
                        if ( m_threadPool )
                        {
                            task->m_future = m_threadPool->enqueue([task]
                            {
                                computeMergeCostTask(*task);
                            });
                        }
#endif
                    }
                }
            }

            if ( !m_canceled )
            {
#if !VHACD_DISABLE_THREADING
                if ( m_threadPool )
                {
                    for (CostTask& task : tasks)
                    {
                        task.m_future.get();
                    }

                    for (CostTask& task : tasks)
                    {
                        AddCostToPriorityQueue(task);
                    }
                }
                else
#endif
                {
                    for (CostTask& task : tasks)
                    {
                        PerformMergeCostTask(task);
                        AddCostToPriorityQueue(task);
                    }
                }
                ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                               100,
                               "Finished cost matrix");
            }

            if ( !m_canceled )
            {
                ScopedTime stMerging("Merging Convex Hulls",
                                     m_params.m_logger);
                Timer t;
                // Now that we know the cost to merge each hull, we can begin merging them.
                bool cancel = false;

                uint32_t maxMergeCount = uint32_t(m_hulls.size()) - m_params.m_maxConvexHulls;
                uint32_t startCount = uint32_t(m_hulls.size());

                while (    !cancel
                        && m_hulls.size() > m_params.m_maxConvexHulls
                        && !m_hullPairQueue.empty()
                        && !m_canceled)
                {
                    double e = t.PeekElapsedSeconds();
                    if ( e >= double(0.1) )
                    {
                        t.Reset();
                        uint32_t hullsProcessed = startCount - uint32_t(m_hulls.size() );
                        double stageProgress = double(hullsProcessed * 100) / double(maxMergeCount);
                        ProgressUpdate(Stages::MERGING_CONVEX_HULLS,
                                       stageProgress,
                                       "Merging Convex Hulls");
                    }

                    HullPair hp = m_hullPairQueue.top();
                    m_hullPairQueue.pop();

                    // It is entirely possible that the hull pair queue can
                    // have references to convex hulls that are no longer valid
                    // because they were previously merged. So we check for this
                    // and if either hull referenced in this pair no longer
                    // exists, then we skip it.

                    // Look up this pair of hulls by ID
                    ConvexHull* ch1 = GetHull(hp.m_hullA);
                    ConvexHull* ch2 = GetHull(hp.m_hullB);

                    // If both hulls are still valid, then we merge them, delete the old
                    // two hulls and recompute the cost matrix for the new combined hull
                    // we have created
                    if ( ch1 && ch2 )
                    {
                        // This is the convex hull which results from combining the
                        // vertices in the two source hulls
                        ConvexHull* combinedHull = ComputeCombinedConvexHull(*ch1,
                                                                                *ch2);
                        // The two old convex hulls are going to get removed
                        RemoveHull(hp.m_hullA);
                        RemoveHull(hp.m_hullB);

                        m_meshId++;
                        combinedHull->m_meshId = m_meshId;
                        tasks.clear();
                        tasks.reserve(m_hulls.size());

                        // Compute the cost between this new merged hull
                        // and all existing convex hulls and then
                        // add that to the priority queue
                        for (auto& i : m_hulls)
                        {
                            if ( m_canceled )
                            {
                                break;
                            }
                            ConvexHull* secondHull = i.second;
                            CostTask ct;
                            ct.m_hullA = combinedHull;
                            ct.m_hullB = secondHull;
                            ct.m_this = this;
                            if ( DoFastCost(ct) )
                            {
                            }
                            else
                            {
                                tasks.push_back(std::move(ct));
                            }
                        }
                        m_hulls[combinedHull->m_meshId] = combinedHull;
                        // See how many merge cost tasks were posted
                        // If there are 8 or more and we are running asynchronously, then do them that way.
#if !VHACD_DISABLE_THREADING
                        if ( m_threadPool && tasks.size() >= 2)
                        {
                            for (CostTask& task : tasks)
                            {
                                task.m_future = m_threadPool->enqueue([&task]
                                {
                                    computeMergeCostTask(task);
                                });
                            }

                            for (CostTask& task : tasks)
                            {
                                task.m_future.get();
                            }
                        }
                        else
#endif
                        {
                            for (CostTask& task : tasks)
                            {
                                PerformMergeCostTask(task);
                            }
                        }

                        for (CostTask& task : tasks)
                        {
                            AddCostToPriorityQueue(task);
                        }
                    }
                }
                // Ok...once we are done, we copy the results!
                m_meshId -= 0;
                ProgressUpdate(Stages::FINALIZING_RESULTS,
                               0,
                               "Finalizing results");
                for (auto& i : m_hulls)
                {
                    if ( m_canceled )
                    {
                        break;
                    }
                    ConvexHull* ch = i.second;
                    // We now must reduce the convex hull
                    if ( ch->m_points.size() > m_params.m_maxNumVerticesPerCH || m_params.m_shrinkWrap)
                    {
                        ConvexHull* reduce = ComputeReducedConvexHull(*ch,
                                                                      m_params.m_maxNumVerticesPerCH,
                                                                      m_params.m_shrinkWrap);
                        ReleaseConvexHull(ch);
                        ch = reduce;
                    }
                    ScaleOutputConvexHull(*ch);
                    ch->m_meshId = m_meshId;
                    m_meshId++;
                    m_convexHulls.push_back(ch);
                }
                m_hulls.clear(); // since the hulls were moved into the output list, we don't need to delete them from this container
                ProgressUpdate(Stages::FINALIZING_RESULTS,
                               100,
                               "Finalized results complete");
            }
        }
        else
        {
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                           0,
                           "Finalizing results");
            m_meshId = 0;
            for (auto& ch : hulls)
            {
                // We now must reduce the convex hull
                if ( ch->m_points.size() > m_params.m_maxNumVerticesPerCH  || m_params.m_shrinkWrap )
                {
                    ConvexHull* reduce = ComputeReducedConvexHull(*ch,
                                                                  m_params.m_maxNumVerticesPerCH,
                                                                  m_params.m_shrinkWrap);
                    ReleaseConvexHull(ch);
                    ch = reduce;
                }
                ScaleOutputConvexHull(*ch);
                ch->m_meshId = m_meshId;
                m_meshId++;
                m_convexHulls.push_back(ch);
            }
            m_hulls.clear();
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                           100,
                           "Finalized results");
        }
    }
}

double VHACDImpl::ComputeConvexHullVolume(const ConvexHull& sm)
{
    double totalVolume = 0;
    VHACD::Vect3 bary(0, 0, 0);
    for (uint32_t i = 0; i < sm.m_points.size(); i++)
    {
        VHACD::Vect3 p(sm.m_points[i]);
        bary += p;
    }
    bary /= double(sm.m_points.size());

    for (uint32_t i = 0; i < sm.m_triangles.size(); i++)
    {
        uint32_t i1 = sm.m_triangles[i].mI0;
        uint32_t i2 = sm.m_triangles[i].mI1;
        uint32_t i3 = sm.m_triangles[i].mI2;

        VHACD::Vect3 ver0(sm.m_points[i1]);
        VHACD::Vect3 ver1(sm.m_points[i2]);
        VHACD::Vect3 ver2(sm.m_points[i3]);

        totalVolume += ComputeVolume4(ver0,
                                      ver1,
                                      ver2,
                                      bary);

    }
    totalVolume = totalVolume / double(6.0);
    return totalVolume;
}

double VHACDImpl::ComputeVolume4(const VHACD::Vect3& a,
                                 const VHACD::Vect3& b,
                                 const VHACD::Vect3& c,
                                 const VHACD::Vect3& d)
{
    VHACD::Vect3 ad = a - d;
    VHACD::Vect3 bd = b - d;
    VHACD::Vect3 cd = c - d;
    VHACD::Vect3 bcd = bd.Cross(cd);
    double dot = ad.Dot(bcd);
    return dot;
}

double VHACDImpl::ComputeConcavity(double volumeSeparate,
                                   double volumeCombined,
                                   double volumeMesh)
{
    return fabs(volumeSeparate - volumeCombined) / volumeMesh;
}

bool VHACDImpl::DoFastCost(CostTask& mt)
{
    bool ret = false;

    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    VHACD::BoundsAABB ch1b(ch1->mBmin,
                           ch1->mBmax);
    VHACD::BoundsAABB ch2b(ch2->mBmin,
                           ch2->mBmax);
    if (!ch1b.Intersects(ch2b))
    {
        VHACD::BoundsAABB b = ch1b.Union(ch2b);

        double combinedVolume = b.Volume();
        double concavity = ComputeConcavity(ch1->m_volume + ch2->m_volume,
                                            combinedVolume,
                                            m_overallHullVolume);
        HullPair hp(ch1->m_meshId,
                    ch2->m_meshId,
                    concavity);
        m_hullPairQueue.push(hp);
        ret = true;
    }
    return ret;
}

void VHACDImpl::PerformMergeCostTask(CostTask& mt)
{
    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    double volume1 = ch1->m_volume;
    double volume2 = ch2->m_volume;

    ConvexHull* combined = ComputeCombinedConvexHull(*ch1,
                                                     *ch2); // Build the combined convex hull
    double combinedVolume = ComputeConvexHullVolume(*combined); // get the combined volume
    mt.m_concavity = ComputeConcavity(volume1 + volume2,
                                      combinedVolume,
                                      m_overallHullVolume);
    ReleaseConvexHull(combined);
}

IVHACD::ConvexHull* VHACDImpl::ComputeReducedConvexHull(const ConvexHull& ch,
                                                        uint32_t maxVerts,
                                                        bool projectHullVertices)
{
    SimpleMesh sourceConvexHull;

    sourceConvexHull.m_vertices = ch.m_points;
    sourceConvexHull.m_indices = ch.m_triangles;

    ShrinkWrap(sourceConvexHull,
               m_AABBTree,
               maxVerts,
               m_voxelScale * 4,
               projectHullVertices);

    ConvexHull *ret = new ConvexHull;

    ret->m_points = sourceConvexHull.m_vertices;
    ret->m_triangles = sourceConvexHull.m_indices;

    VHACD::BoundsAABB b = VHACD::BoundsAABB(ret->m_points).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();
    ComputeCentroid(ret->m_points,
                    ret->m_triangles,
                    ret->m_center);

    ret->m_volume = ComputeConvexHullVolume(*ret);

    // Return the convex hull
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::ComputeCombinedConvexHull(const ConvexHull& sm1,
                                                         const ConvexHull& sm2)
{
    uint32_t vcount = uint32_t(sm1.m_points.size() + sm2.m_points.size()); // Total vertices from both hulls
    std::vector<VHACD::Vertex> vertices(vcount);
    auto it = std::copy(sm1.m_points.begin(),
                        sm1.m_points.end(),
                        vertices.begin());
    std::copy(sm2.m_points.begin(),
                sm2.m_points.end(),
                it);

    VHACD::QuickHull qh;
    qh.ComputeConvexHull(vertices,
                         vcount);

    ConvexHull* ret = new ConvexHull;
    ret->m_points = qh.GetVertices();
    ret->m_triangles = qh.GetIndices();

    ret->m_volume = ComputeConvexHullVolume(*ret);

    VHACD::BoundsAABB b = VHACD::BoundsAABB(qh.GetVertices()).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();
    ComputeCentroid(ret->m_points,
                    ret->m_triangles,
                    ret->m_center);

    // Return the convex hull
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::GetHull(uint32_t index)
{
    ConvexHull* ret = nullptr;

    auto found = m_hulls.find(index);
    if ( found != m_hulls.end() )
    {
        ret = found->second;
    }

    return ret;
}

bool VHACDImpl::RemoveHull(uint32_t index)
{
    bool ret = false;
    auto found = m_hulls.find(index);
    if ( found != m_hulls.end() )
    {
        ret = true;
        ReleaseConvexHull(found->second);
        m_hulls.erase(found);
    }
    return ret;
}

IVHACD::ConvexHull* VHACDImpl::CopyConvexHull(const ConvexHull& source)
{
    ConvexHull *ch = new ConvexHull;
    *ch = source;

    return ch;
}

const char* VHACDImpl::GetStageName(Stages stage) const
{
    const char *ret = "unknown";
    switch ( stage )
    {
        case Stages::COMPUTE_BOUNDS_OF_INPUT_MESH:
            ret = "COMPUTE_BOUNDS_OF_INPUT_MESH";
            break;
        case Stages::REINDEXING_INPUT_MESH:
            ret = "REINDEXING_INPUT_MESH";
            break;
        case Stages::CREATE_RAYCAST_MESH:
            ret = "CREATE_RAYCAST_MESH";
            break;
        case Stages::VOXELIZING_INPUT_MESH:
            ret = "VOXELIZING_INPUT_MESH";
            break;
        case Stages::BUILD_INITIAL_CONVEX_HULL:
            ret = "BUILD_INITIAL_CONVEX_HULL";
            break;
        case Stages::PERFORMING_DECOMPOSITION:
            ret = "PERFORMING_DECOMPOSITION";
            break;
        case Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING:
            ret = "INITIALIZING_CONVEX_HULLS_FOR_MERGING";
            break;
        case Stages::COMPUTING_COST_MATRIX:
            ret = "COMPUTING_COST_MATRIX";
            break;
        case Stages::MERGING_CONVEX_HULLS:
            ret = "MERGING_CONVEX_HULLS";
            break;
        case Stages::FINALIZING_RESULTS:
            ret = "FINALIZING_RESULTS";
            break;
        case Stages::NUM_STAGES:
            // Should be unreachable, here to silence enumeration value not handled in switch warnings
            // GCC/Clang's -Wswitch
            break;
    }
    return ret;
}

void VHACDImpl::ProgressUpdate(Stages stage,
                               double stageProgress,
                               const char* operation)
{
    if ( m_params.m_callback )
    {
        double overallProgress = (double(stage) * 100) / double(Stages::NUM_STAGES);
        const char *s = GetStageName(stage);
        m_params.m_callback->Update(overallProgress,
                                    stageProgress,
                                    s,
                                    operation);
    }
}

bool VHACDImpl::IsCanceled() const
{
    return m_canceled;
}

IVHACD* CreateVHACD(void)
{
    VHACDImpl *ret = new VHACDImpl;
    return static_cast< IVHACD *>(ret);
}

IVHACD* CreateVHACD(void);

#if !VHACD_DISABLE_THREADING

class LogMessage
{
public:
    double  m_overallProgress{ double(-1.0) };
    double  m_stageProgress{ double(-1.0) };
    std::string m_stage;
    std::string m_operation;
};

class VHACDAsyncImpl : public VHACD::IVHACD,
                       public VHACD::IVHACD::IUserCallback,
                       VHACD::IVHACD::IUserLogger,
                       VHACD::IVHACD::IUserTaskRunner
{
public:
    VHACDAsyncImpl() = default;

    ~VHACDAsyncImpl() override;

    void Cancel() override final;

    bool Compute(const float* const points,
                 const uint32_t countPoints,
                 const uint32_t* const triangles,
                 const uint32_t countTriangles,
                 const Parameters& params) override final;

    bool Compute(const double* const points,
                 const uint32_t countPoints,
                 const uint32_t* const triangles,
                 const uint32_t countTriangles,
                 const Parameters& params) override final;

    bool GetConvexHull(const uint32_t index,
                       VHACD::IVHACD::ConvexHull& ch) const override final;

    uint32_t GetNConvexHulls() const override final;

    void Clean() override final; // release internally allocated memory

    void Release() override final; // release IVHACD

    // Will compute the center of mass of the convex hull decomposition results and return it
    // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
    bool ComputeCenterOfMass(double centerOfMass[3]) const override;

    bool IsReady() const override final;

    /**
    * At the request of LegionFu : out_look@foxmail.com
    * This method will return which convex hull is closest to the source position.
    * You can use this method to figure out, for example, which vertices in the original
    * source mesh are best associated with which convex hull.
    *
    * @param pos : The input 3d position to test against
    *
    * @return : Returns which convex hull this position is closest to.
    */
    uint32_t findNearestConvexHull(const double pos[3],
                                   double& distanceToHull) override final;

    void Update(const double overallProgress,
                const double stageProgress,
                const char* const stage,
                const char *operation) override final;

    void Log(const char* const msg) override final;

    void* StartTask(std::function<void()> func) override;

    void JoinTask(void* Task) override;

    bool Compute(const Parameters params);

    bool ComputeNow(const std::vector<VHACD::Vertex>& points,
                    const std::vector<VHACD::Triangle>& triangles,
                    const Parameters& _desc);

    // As a convenience for the calling application we only send it update and log messages from it's own main
    // thread.  This reduces the complexity burden on the caller by making sure it only has to deal with log
    // messages in it's main application thread.
    void ProcessPendingMessages() const;

private:
    VHACD::VHACDImpl                m_VHACD;
    std::vector<VHACD::Vertex>      m_vertices;
    std::vector<VHACD::Triangle>    m_indices;
    VHACD::IVHACD::IUserCallback*   m_callback{ nullptr };
    VHACD::IVHACD::IUserLogger*     m_logger{ nullptr };
    VHACD::IVHACD::IUserTaskRunner* m_taskRunner{ nullptr };
    void*                           m_task{ nullptr };
    std::atomic<bool>               m_running{ false };
    std::atomic<bool>               m_cancel{ false };

    // Thread safe caching mechanism for messages and update status.
    // This is so that caller always gets messages in his own thread
    // Member variables are marked as 'mutable' since the message dispatch function
    // is called from const query methods.
    mutable std::mutex              m_messageMutex;
    mutable std::vector<LogMessage> m_messages;
    mutable std::atomic<bool>       m_haveMessages{ false };
};

VHACDAsyncImpl::~VHACDAsyncImpl()
{
    Cancel();
}

void VHACDAsyncImpl::Cancel()
{
    m_cancel = true;
    m_VHACD.Cancel();

    if (m_task)
    {
        m_taskRunner->JoinTask(m_task); // Wait for the thread to fully exit before we delete the instance
        m_task = nullptr;
    }
    m_cancel = false; // clear the cancel semaphore
}

bool VHACDAsyncImpl::Compute(const float* const points,
                             const uint32_t countPoints,
                             const uint32_t* const triangles,
                             const uint32_t countTriangles,
                             const Parameters& params)
{
    m_vertices.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        m_vertices.emplace_back(points[i * 3 + 0],
                                points[i * 3 + 1],
                                points[i * 3 + 2]);
    }

    m_indices.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        m_indices.emplace_back(triangles[i * 3 + 0],
                               triangles[i * 3 + 1],
                               triangles[i * 3 + 2]);
    }

    return Compute(params);
}

bool VHACDAsyncImpl::Compute(const double* const points,
                             const uint32_t countPoints,
                             const uint32_t* const triangles,
                             const uint32_t countTriangles,
                             const Parameters& params)
{
    // We need to copy the input vertices and triangles into our own buffers so we can operate
    // on them safely from the background thread.
    // Can't be local variables due to being asynchronous
    m_vertices.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        m_vertices.emplace_back(points[i * 3 + 0],
                                points[i * 3 + 1],
                                points[i * 3 + 2]);
    }

    m_indices.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        m_indices.emplace_back(triangles[i * 3 + 0],
                               triangles[i * 3 + 1],
                               triangles[i * 3 + 2]);
    }

    return Compute(params);
}

bool VHACDAsyncImpl::GetConvexHull(const uint32_t index,
                                   VHACD::IVHACD::ConvexHull& ch) const
{
    return m_VHACD.GetConvexHull(index,
                                 ch);
}

uint32_t VHACDAsyncImpl::GetNConvexHulls() const
{
    ProcessPendingMessages();
    return m_VHACD.GetNConvexHulls();
}

void VHACDAsyncImpl::Clean()
{
    Cancel();
    m_VHACD.Clean();
}

void VHACDAsyncImpl::Release()
{
    delete this;
}

bool VHACDAsyncImpl::ComputeCenterOfMass(double centerOfMass[3]) const
{
    bool ret = false;

    centerOfMass[0] = 0;
    centerOfMass[1] = 0;
    centerOfMass[2] = 0;

    if (IsReady())
    {
        ret = m_VHACD.ComputeCenterOfMass(centerOfMass);
    }
    return ret;
}

bool VHACDAsyncImpl::IsReady() const
{
    ProcessPendingMessages();
    return !m_running;
}

uint32_t VHACDAsyncImpl::findNearestConvexHull(const double pos[3],
                                               double& distanceToHull)
{
    uint32_t ret = 0; // The default return code is zero

    distanceToHull = 0;
    // First, make sure that we have valid and completed results
    if (IsReady() )
    {
        ret = m_VHACD.findNearestConvexHull(pos,distanceToHull);
    }

    return ret;
}

void VHACDAsyncImpl::Update(const double overallProgress,
                            const double stageProgress,
                            const char* const stage,
                            const char* operation)
{
    m_messageMutex.lock();
    LogMessage m;
    m.m_operation = std::string(operation);
    m.m_overallProgress = overallProgress;
    m.m_stageProgress = stageProgress;
    m.m_stage = std::string(stage);
    m_messages.push_back(m);
    m_haveMessages = true;
    m_messageMutex.unlock();
}

void VHACDAsyncImpl::Log(const char* const msg)
{
    m_messageMutex.lock();
    LogMessage m;
    m.m_operation = std::string(msg);
    m_haveMessages = true;
    m_messages.push_back(m);
    m_messageMutex.unlock();
}

void* VHACDAsyncImpl::StartTask(std::function<void()> func)
{
    return new std::thread(func);
}

void VHACDAsyncImpl::JoinTask(void* Task)
{
    std::thread* t = static_cast<std::thread*>(Task);
    t->join();
    delete t;
}

bool VHACDAsyncImpl::Compute(Parameters params)
{
    Cancel(); // if we previously had a solution running; cancel it.

    m_taskRunner = params.m_taskRunner ? params.m_taskRunner : this;
    params.m_taskRunner = m_taskRunner;

    m_running = true;
    m_task = m_taskRunner->StartTask([this, params]() {
        ComputeNow(m_vertices,
                   m_indices,
                   params);
        // If we have a user provided callback and the user did *not* call 'cancel' we notify him that the
        // task is completed. However..if the user selected 'cancel' we do not send a completed notification event.
        if (params.m_callback && !m_cancel)
        {
            params.m_callback->NotifyVHACDComplete();
        }
        m_running = false;
    });
    return true;
}

bool VHACDAsyncImpl::ComputeNow(const std::vector<VHACD::Vertex>& points,
                                const std::vector<VHACD::Triangle>& triangles,
                                const Parameters& _desc)
{
    uint32_t ret = 0;

    Parameters desc;
    m_callback = _desc.m_callback;
    m_logger = _desc.m_logger;

    desc = _desc;
    // Set our intercepting callback interfaces if non-null
    desc.m_callback = _desc.m_callback ? this : nullptr;
    desc.m_logger = _desc.m_logger ? this : nullptr;

    // If not task runner provided, then use the default one
    if (desc.m_taskRunner == nullptr)
    {
        desc.m_taskRunner = this;
    }

    bool ok = m_VHACD.Compute(points,
                              triangles,
                              desc);
    if (ok)
    {
        ret = m_VHACD.GetNConvexHulls();
    }

    return ret ? true : false;
}

void VHACDAsyncImpl::ProcessPendingMessages() const
{
    if (m_cancel)
    {
        return;
    }
    if ( m_haveMessages )
    {
        m_messageMutex.lock();
        for (auto& i : m_messages)
        {
            if ( i.m_overallProgress == -1 )
            {
                if ( m_logger )
                {
                    m_logger->Log(i.m_operation.c_str());
                }
            }
            else if ( m_callback )
            {
                m_callback->Update(i.m_overallProgress,
                                   i.m_stageProgress,
                                   i.m_stage.c_str(),
                                   i.m_operation.c_str());
            }
        }
        m_messages.clear();
        m_haveMessages = false;
        m_messageMutex.unlock();
    }
}

IVHACD* CreateVHACD_ASYNC()
{
    VHACDAsyncImpl* m = new VHACDAsyncImpl;
    return static_cast<IVHACD*>(m);
}
#endif

} // namespace VHACD

#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif // __GNUC__

#endif // ENABLE_VHACD_IMPLEMENTATION

#endif // VHACD_H
