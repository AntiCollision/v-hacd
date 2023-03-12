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

#include "VertexIndex.h"
#include "VertexIndex.inl"

#include "Voxel.h"
#include "Voxel.inl"

#include "SimpleMesh.h"

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
#include "AABBTree.h"
#include "AABBTree.inl"
#include "VoxelValue.h"
#include "Volume.h"
#include "Volume.inl"

#include "QuickHull.h"


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
#include "ThreadPool.h"
#endif

#include "Stages.h"
#include "SplitAxis.h"
#include "VHACDCallbakcs.h"

#include "VoxelHull.h"
#include "VoxelHull.inl"

#include "CostTask.h"
#include "HullPair.h"

#include "VHACDImpl.h"
#include "VHACDImpl.inl"

IVHACD* CreateVHACD(void)
{
    VHACDImpl *ret = new VHACDImpl;
    return static_cast< IVHACD *>(ret);
}

#if !VHACD_DISABLE_THREADING

#include "LogMessage.h"
#include "VHACDAsyncImpl.h"
#include "VHACDAsyncImpl.inl"

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
