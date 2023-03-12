#pragma once


ConvexHull::ConvexHull(const std::vector<::VHACD::Vertex>& vertexCloud,
    double distTol,
    int maxVertexCount)
{
    if (vertexCloud.size() >= 4)
    {
        BuildHull(vertexCloud,
            distTol,
            maxVertexCount);
    }
}

const std::vector<VHACD::Vect3>& ConvexHull::GetVertexPool() const
{
    return m_points;
}

void ConvexHull::BuildHull(const std::vector<::VHACD::Vertex>& vertexCloud,
    double distTol,
    int maxVertexCount)
{
    size_t treeCount = vertexCloud.size() / (VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE >> 1);
    treeCount = std::max(treeCount, size_t(4)) * 2;

    std::vector<ConvexHullVertex> points(vertexCloud.size());
    /*
     * treePool provides a memory pool for the AABB tree
     * Each node is either a leaf or non-leaf node
     * Non-leaf nodes have up to 8 vertices
     * Vertices are specified by the m_indices array and are accessed via the points array
     *
     * Later on in ConvexHull::SupportVertex, the tree is used directly
     * It differentiates between ConvexHullAABBTreeNode and ConvexHull3DPointCluster by whether the m_left and m_right
     * pointers are null or not
     *
     * Pointers have to be stable
     */
    NodeBundle<ConvexHullAABBTreeNode> treePool;
    for (size_t i = 0; i < vertexCloud.size(); ++i)
    {
        points[i] = VHACD::Vect3(vertexCloud[i]);
    }
    int count = InitVertexArray(points,
        treePool);

    if (m_points.size() >= 4)
    {
        CalculateConvexHull3D(&treePool.GetFirstNode(),
            points,
            count,
            distTol,
            maxVertexCount);
    }
}

void ConvexHull::GetUniquePoints(std::vector<ConvexHullVertex>& points)
{
    class CompareVertex
    {
    public:
        int Compare(const ConvexHullVertex& elementA, const ConvexHullVertex& elementB) const
        {
            for (int i = 0; i < 3; i++)
            {
                if (elementA[i] < elementB[i])
                {
                    return -1;
                }
                else if (elementA[i] > elementB[i])
                {
                    return 1;
                }
            }
            return 0;
        }
    };

    int count = int(points.size());
    Sort<ConvexHullVertex, CompareVertex>(points.data(),
        count);

    int indexCount = 0;
    CompareVertex compareVertex;
    for (int i = 1; i < count; ++i)
    {
        for (; i < count; ++i)
        {
            if (compareVertex.Compare(points[indexCount], points[i]))
            {
                indexCount++;
                points[indexCount] = points[i];
                break;
            }
        }
    }
    points.resize(indexCount + 1);
}


ConvexHullAABBTreeNode* ConvexHull::BuildTreeRecurse(ConvexHullAABBTreeNode* const parent,
    ConvexHullVertex* const points,
    int count,
    int baseIndex,
    NodeBundle<ConvexHullAABBTreeNode>& memoryPool) const
{
    ConvexHullAABBTreeNode* tree = nullptr;

    assert(count);
    VHACD::Vect3 minP(double(1.0e15));
    VHACD::Vect3 maxP(-double(1.0e15));
    if (count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
    {
        ConvexHullAABBTreeNode& clump = memoryPool.GetNextNode();

        clump.m_count = count;
        for (int i = 0; i < count; ++i)
        {
            clump.m_indices[i] = i + baseIndex;

            const VHACD::Vect3& p = points[i];
            minP = minP.CWiseMin(p);
            maxP = maxP.CWiseMax(p);
        }

        clump.m_left = nullptr;
        clump.m_right = nullptr;
        tree = &clump;
    }
    else
    {
        VHACD::Vect3 median(0);
        VHACD::Vect3 varian(0);
        for (int i = 0; i < count; ++i)
        {
            const VHACD::Vect3& p = points[i];
            minP = minP.CWiseMin(p);
            maxP = maxP.CWiseMax(p);
            median += p;
            varian += p.CWiseMul(p);
        }

        varian = varian * double(count) - median.CWiseMul(median);
        int index = 0;
        double maxVarian = double(-1.0e10);
        for (int i = 0; i < 3; ++i)
        {
            if (varian[i] > maxVarian)
            {
                index = i;
                maxVarian = varian[i];
            }
        }
        VHACD::Vect3 center(median * (double(1.0) / double(count)));

        double test = center[index];

        int i0 = 0;
        int i1 = count - 1;
        do
        {
            for (; i0 <= i1; i0++)
            {
                double val = points[i0][index];
                if (val > test)
                {
                    break;
                }
            }

            for (; i1 >= i0; i1--)
            {
                double val = points[i1][index];
                if (val < test)
                {
                    break;
                }
            }

            if (i0 < i1)
            {
                std::swap(points[i0],
                    points[i1]);
                i0++;
                i1--;
            }
        } while (i0 <= i1);

        if (i0 == 0)
        {
            i0 = count / 2;
        }
        if (i0 >= (count - 1))
        {
            i0 = count / 2;
        }

        tree = &memoryPool.GetNextNode();

        assert(i0);
        assert(count - i0);

        tree->m_left = BuildTreeRecurse(tree,
            points,
            i0,
            baseIndex,
            memoryPool);
        tree->m_right = BuildTreeRecurse(tree,
            &points[i0],
            count - i0,
            i0 + baseIndex,
            memoryPool);
    }

    assert(tree);
    tree->m_parent = parent;
    /*
     * WARNING: Changing the compiler conversion of 1.0e-3f changes the results of the convex decomposition
     * Inflate the tree's bounding box slightly
     */
    tree->m_box[0] = minP - VHACD::Vect3(double(1.0e-3f));
    tree->m_box[1] = maxP + VHACD::Vect3(double(1.0e-3f));
    return tree;
}

ConvexHullAABBTreeNode* ConvexHull::BuildTreeOld(std::vector<ConvexHullVertex>& points,
    NodeBundle<ConvexHullAABBTreeNode>& memoryPool)
{
    GetUniquePoints(points);
    int count = int(points.size());
    if (count < 4)
    {
        return nullptr;
    }
    return BuildTreeRecurse(nullptr,
        points.data(),
        count,
        0,
        memoryPool);
}

ConvexHullAABBTreeNode* ConvexHull::BuildTreeNew(std::vector<ConvexHullVertex>& points,
    std::vector<ConvexHullAABBTreeNode>& memoryPool) const
{
    class dCluster
    {
    public:
        VHACD::Vect3 m_sum{ double(0.0) };
        VHACD::Vect3 m_sum2{ double(0.0) };
        int m_start{ 0 };
        int m_count{ 0 };
    };

    dCluster firstCluster;
    firstCluster.m_count = int(points.size());

    for (int i = 0; i < firstCluster.m_count; ++i)
    {
        const VHACD::Vect3& p = points[i];
        firstCluster.m_sum += p;
        firstCluster.m_sum2 += p.CWiseMul(p);
    }

    int baseCount = 0;
    const int clusterSize = 16;

    if (firstCluster.m_count > clusterSize)
    {
        dCluster spliteStack[128];
        spliteStack[0] = firstCluster;
        size_t stack = 1;

        while (stack)
        {
            stack--;
            dCluster cluster(spliteStack[stack]);

            const VHACD::Vect3 origin(cluster.m_sum * (double(1.0) / cluster.m_count));
            const VHACD::Vect3 variance2(cluster.m_sum2 * (double(1.0) / cluster.m_count) - origin.CWiseMul(origin));
            double maxVariance2 = variance2.MaxCoeff();

            if ((cluster.m_count <= clusterSize)
                || (stack > (sizeof(spliteStack) / sizeof(spliteStack[0]) - 4))
                || (maxVariance2 < 1.e-4f))
            {
                // no sure if this is beneficial,
                // the array is so small that seem too much overhead
                //int maxIndex = 0;
                //double min_x = 1.0e20f;
                //for (int i = 0; i < cluster.m_count; ++i)
                //{
                //	if (points[cluster.m_start + i].getX() < min_x)
                //	{
                //		maxIndex = i;
                //		min_x = points[cluster.m_start + i].getX();
                //	}
                //}
                //Swap(points[cluster.m_start], points[cluster.m_start + maxIndex]);
                //
                //for (int i = 2; i < cluster.m_count; ++i)
                //{
                //	int j = i;
                //	ConvexHullVertex tmp(points[cluster.m_start + i]);
                //	for (; points[cluster.m_start + j - 1].getX() > tmp.getX(); --j)
                //	{
                //		assert(j > 0);
                //		points[cluster.m_start + j] = points[cluster.m_start + j - 1];
                //	}
                //	points[cluster.m_start + j] = tmp;
                //}

                int count = cluster.m_count;
                for (int i = cluster.m_count - 1; i > 0; --i)
                {
                    for (int j = i - 1; j >= 0; --j)
                    {
                        VHACD::Vect3 error(points[cluster.m_start + j] - points[cluster.m_start + i]);
                        double mag2 = error.Dot(error);
                        if (mag2 < double(1.0e-6))
                        {
                            points[cluster.m_start + j] = points[cluster.m_start + i];
                            count--;
                            break;
                        }
                    }
                }

                assert(baseCount <= cluster.m_start);
                for (int i = 0; i < count; ++i)
                {
                    points[baseCount] = points[cluster.m_start + i];
                    baseCount++;
                }
            }
            else
            {
                const int firstSortAxis = variance2.LongestAxis();
                double axisVal = origin[firstSortAxis];

                int i0 = 0;
                int i1 = cluster.m_count - 1;

                const int start = cluster.m_start;
                while (i0 < i1)
                {
                    while ((points[start + i0][firstSortAxis] <= axisVal)
                        && (i0 < i1))
                    {
                        ++i0;
                    };

                    while ((points[start + i1][firstSortAxis] > axisVal)
                        && (i0 < i1))
                    {
                        --i1;
                    }

                    assert(i0 <= i1);
                    if (i0 < i1)
                    {
                        std::swap(points[start + i0],
                            points[start + i1]);
                        ++i0;
                        --i1;
                    }
                }

                while ((points[start + i0][firstSortAxis] <= axisVal)
                    && (i0 < cluster.m_count))
                {
                    ++i0;
                };

#ifdef _DEBUG
                for (int i = 0; i < i0; ++i)
                {
                    assert(points[start + i][firstSortAxis] <= axisVal);
                }

                for (int i = i0; i < cluster.m_count; ++i)
                {
                    assert(points[start + i][firstSortAxis] > axisVal);
                }
#endif

                VHACD::Vect3 xc(0);
                VHACD::Vect3 x2c(0);
                for (int i = 0; i < i0; ++i)
                {
                    const VHACD::Vect3& x = points[start + i];
                    xc += x;
                    x2c += x.CWiseMul(x);
                }

                dCluster cluster_i1(cluster);
                cluster_i1.m_start = start + i0;
                cluster_i1.m_count = cluster.m_count - i0;
                cluster_i1.m_sum -= xc;
                cluster_i1.m_sum2 -= x2c;
                spliteStack[stack] = cluster_i1;
                assert(cluster_i1.m_count > 0);
                stack++;

                dCluster cluster_i0(cluster);
                cluster_i0.m_start = start;
                cluster_i0.m_count = i0;
                cluster_i0.m_sum = xc;
                cluster_i0.m_sum2 = x2c;
                assert(cluster_i0.m_count > 0);
                spliteStack[stack] = cluster_i0;
                stack++;
            }
        }
    }

    points.resize(baseCount);
    if (baseCount < 4)
    {
        return nullptr;
    }

    VHACD::Vect3 sum(0);
    VHACD::Vect3 sum2(0);
    VHACD::Vect3 minP(double(1.0e15));
    VHACD::Vect3 maxP(double(-1.0e15));
    class dTreeBox
    {
    public:
        VHACD::Vect3 m_min;
        VHACD::Vect3 m_max;
        VHACD::Vect3 m_sum;
        VHACD::Vect3 m_sum2;
        ConvexHullAABBTreeNode* m_parent;
        ConvexHullAABBTreeNode** m_child;
        int m_start;
        int m_count;
    };

    for (int i = 0; i < baseCount; ++i)
    {
        const VHACD::Vect3& p = points[i];
        sum += p;
        sum2 += p.CWiseMul(p);
        minP = minP.CWiseMin(p);
        maxP = maxP.CWiseMax(p);
    }

    dTreeBox treeBoxStack[128];
    treeBoxStack[0].m_start = 0;
    treeBoxStack[0].m_count = baseCount;
    treeBoxStack[0].m_sum = sum;
    treeBoxStack[0].m_sum2 = sum2;
    treeBoxStack[0].m_min = minP;
    treeBoxStack[0].m_max = maxP;
    treeBoxStack[0].m_child = nullptr;
    treeBoxStack[0].m_parent = nullptr;

    int stack = 1;
    ConvexHullAABBTreeNode* root = nullptr;
    while (stack)
    {
        stack--;
        dTreeBox box(treeBoxStack[stack]);
        if (box.m_count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
        {
            assert(memoryPool.size() != memoryPool.capacity()
                && "memoryPool is going to be reallocated, pointers will be invalid");
            memoryPool.emplace_back();
            ConvexHullAABBTreeNode& clump = memoryPool.back();

            clump.m_count = box.m_count;
            for (int i = 0; i < box.m_count; ++i)
            {
                clump.m_indices[i] = i + box.m_start;
            }
            clump.m_box[0] = box.m_min;
            clump.m_box[1] = box.m_max;

            if (box.m_child)
            {
                *box.m_child = &clump;
            }

            if (!root)
            {
                root = &clump;
            }
        }
        else
        {
            const VHACD::Vect3 origin(box.m_sum * (double(1.0) / box.m_count));
            const VHACD::Vect3 variance2(box.m_sum2 * (double(1.0) / box.m_count) - origin.CWiseMul(origin));

            int firstSortAxis = 0;
            if ((variance2.GetY() >= variance2.GetX()) && (variance2.GetY() >= variance2.GetZ()))
            {
                firstSortAxis = 1;
            }
            else if ((variance2.GetZ() >= variance2.GetX()) && (variance2.GetZ() >= variance2.GetY()))
            {
                firstSortAxis = 2;
            }
            double axisVal = origin[firstSortAxis];

            int i0 = 0;
            int i1 = box.m_count - 1;

            const int start = box.m_start;
            while (i0 < i1)
            {
                while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < i1))
                {
                    ++i0;
                };

                while ((points[start + i1][firstSortAxis] > axisVal) && (i0 < i1))
                {
                    --i1;
                }

                assert(i0 <= i1);
                if (i0 < i1)
                {
                    std::swap(points[start + i0],
                        points[start + i1]);
                    ++i0;
                    --i1;
                }
            }

            while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < box.m_count))
            {
                ++i0;
            };

#ifdef _DEBUG
            for (int i = 0; i < i0; ++i)
            {
                assert(points[start + i][firstSortAxis] <= axisVal);
            }

            for (int i = i0; i < box.m_count; ++i)
            {
                assert(points[start + i][firstSortAxis] > axisVal);
            }
#endif

            assert(memoryPool.size() != memoryPool.capacity()
                && "memoryPool is going to be reallocated, pointers will be invalid");
            memoryPool.emplace_back();
            ConvexHullAABBTreeNode& node = memoryPool.back();

            node.m_box[0] = box.m_min;
            node.m_box[1] = box.m_max;
            if (box.m_child)
            {
                *box.m_child = &node;
            }

            if (!root)
            {
                root = &node;
            }

            {
                VHACD::Vect3 xc(0);
                VHACD::Vect3 x2c(0);
                VHACD::Vect3 p0(double(1.0e15));
                VHACD::Vect3 p1(double(-1.0e15));
                for (int i = i0; i < box.m_count; ++i)
                {
                    const VHACD::Vect3& p = points[start + i];
                    xc += p;
                    x2c += p.CWiseMul(p);
                    p0 = p0.CWiseMin(p);
                    p1 = p1.CWiseMax(p);
                }

                dTreeBox cluster_i1(box);
                cluster_i1.m_start = start + i0;
                cluster_i1.m_count = box.m_count - i0;
                cluster_i1.m_sum = xc;
                cluster_i1.m_sum2 = x2c;
                cluster_i1.m_min = p0;
                cluster_i1.m_max = p1;
                cluster_i1.m_parent = &node;
                cluster_i1.m_child = &node.m_right;
                treeBoxStack[stack] = cluster_i1;
                assert(cluster_i1.m_count > 0);
                stack++;
            }

            {
                VHACD::Vect3 xc(0);
                VHACD::Vect3 x2c(0);
                VHACD::Vect3 p0(double(1.0e15));
                VHACD::Vect3 p1(double(-1.0e15));
                for (int i = 0; i < i0; ++i)
                {
                    const VHACD::Vect3& p = points[start + i];
                    xc += p;
                    x2c += p.CWiseMul(p);
                    p0 = p0.CWiseMin(p);
                    p1 = p1.CWiseMax(p);
                }

                dTreeBox cluster_i0(box);
                cluster_i0.m_start = start;
                cluster_i0.m_count = i0;
                cluster_i0.m_min = p0;
                cluster_i0.m_max = p1;
                cluster_i0.m_sum = xc;
                cluster_i0.m_sum2 = x2c;
                cluster_i0.m_parent = &node;
                cluster_i0.m_child = &node.m_left;
                assert(cluster_i0.m_count > 0);
                treeBoxStack[stack] = cluster_i0;
                stack++;
            }
        }
    }

    return root;
}

int ConvexHull::SupportVertex(ConvexHullAABBTreeNode** const treePointer,
    const std::vector<ConvexHullVertex>& points,
    const VHACD::Vect3& dirPlane,
    const bool removeEntry) const
{
#define VHACD_STACK_DEPTH_3D 64
    double aabbProjection[VHACD_STACK_DEPTH_3D];
    ConvexHullAABBTreeNode* stackPool[VHACD_STACK_DEPTH_3D];

    VHACD::Vect3 dir(dirPlane);

    int index = -1;
    int stack = 1;
    stackPool[0] = *treePointer;
    aabbProjection[0] = double(1.0e20);
    double maxProj = double(-1.0e20);
    int ix = (dir[0] > double(0.0)) ? 1 : 0;
    int iy = (dir[1] > double(0.0)) ? 1 : 0;
    int iz = (dir[2] > double(0.0)) ? 1 : 0;
    while (stack)
    {
        stack--;
        double boxSupportValue = aabbProjection[stack];
        if (boxSupportValue > maxProj)
        {
            ConvexHullAABBTreeNode* me = stackPool[stack];

            /*
             * If the node is not a leaf node...
             */
            if (me->m_left && me->m_right)
            {
                const VHACD::Vect3 leftSupportPoint(me->m_left->m_box[ix].GetX(),
                    me->m_left->m_box[iy].GetY(),
                    me->m_left->m_box[iz].GetZ());
                double leftSupportDist = leftSupportPoint.Dot(dir);

                const VHACD::Vect3 rightSupportPoint(me->m_right->m_box[ix].GetX(),
                    me->m_right->m_box[iy].GetY(),
                    me->m_right->m_box[iz].GetZ());
                double rightSupportDist = rightSupportPoint.Dot(dir);

                /*
                 * ...push the shorter side first
                 * So we can explore the tree in the larger side first
                 */
                if (rightSupportDist >= leftSupportDist)
                {
                    aabbProjection[stack] = leftSupportDist;
                    stackPool[stack] = me->m_left;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                    aabbProjection[stack] = rightSupportDist;
                    stackPool[stack] = me->m_right;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                }
                else
                {
                    aabbProjection[stack] = rightSupportDist;
                    stackPool[stack] = me->m_right;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                    aabbProjection[stack] = leftSupportDist;
                    stackPool[stack] = me->m_left;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                }
            }
            /*
             * If it is a node...
             */
            else
            {
                ConvexHullAABBTreeNode* cluster = me;
                for (size_t i = 0; i < cluster->m_count; ++i)
                {
                    const ConvexHullVertex& p = points[cluster->m_indices[i]];
                    assert(p.GetX() >= cluster->m_box[0].GetX());
                    assert(p.GetX() <= cluster->m_box[1].GetX());
                    assert(p.GetY() >= cluster->m_box[0].GetY());
                    assert(p.GetY() <= cluster->m_box[1].GetY());
                    assert(p.GetZ() >= cluster->m_box[0].GetZ());
                    assert(p.GetZ() <= cluster->m_box[1].GetZ());
                    if (!p.m_mark)
                    {
                        //assert(p.m_w == double(0.0f));
                        double dist = p.Dot(dir);
                        if (dist > maxProj)
                        {
                            maxProj = dist;
                            index = cluster->m_indices[i];
                        }
                    }
                    else if (removeEntry)
                    {
                        cluster->m_indices[i] = cluster->m_indices[cluster->m_count - 1];
                        cluster->m_count = cluster->m_count - 1;
                        i--;
                    }
                }

                if (cluster->m_count == 0)
                {
                    ConvexHullAABBTreeNode* const parent = cluster->m_parent;
                    if (parent)
                    {
                        ConvexHullAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
                        assert(sibling != cluster);
                        ConvexHullAABBTreeNode* const grandParent = parent->m_parent;
                        if (grandParent)
                        {
                            sibling->m_parent = grandParent;
                            if (grandParent->m_right == parent)
                            {
                                grandParent->m_right = sibling;
                            }
                            else
                            {
                                grandParent->m_left = sibling;
                            }
                        }
                        else
                        {
                            sibling->m_parent = nullptr;
                            *treePointer = sibling;
                        }
                    }
                }
            }
        }
    }

    assert(index != -1);
    return index;
}

double ConvexHull::TetrahedrumVolume(const VHACD::Vect3& p0,
    const VHACD::Vect3& p1,
    const VHACD::Vect3& p2,
    const VHACD::Vect3& p3) const
{
    const VHACD::Vect3 p1p0(p1 - p0);
    const VHACD::Vect3 p2p0(p2 - p0);
    const VHACD::Vect3 p3p0(p3 - p0);
    return p3p0.Dot(p1p0.Cross(p2p0));
}

int ConvexHull::InitVertexArray(std::vector<ConvexHullVertex>& points,
    NodeBundle<ConvexHullAABBTreeNode>& memoryPool)
    //                                 std::vector<ConvexHullAABBTreeNode>& memoryPool)
{
#if 1
    ConvexHullAABBTreeNode* tree = BuildTreeOld(points,
        memoryPool);
#else
    ConvexHullAABBTreeNode* tree = BuildTreeNew(points, (char**)&memoryPool, maxMemSize);
#endif
    int count = int(points.size());
    if (count < 4)
    {
        m_points.resize(0);
        return 0;
    }

    m_points.resize(count);
    m_aabbP0 = tree->m_box[0];
    m_aabbP1 = tree->m_box[1];

    VHACD::Vect3 boxSize(tree->m_box[1] - tree->m_box[0]);
    m_diag = boxSize.GetNorm();
    const ndNormalMap& normalMap = ndNormalMap::GetNormalMap();

    int index0 = SupportVertex(&tree,
        points,
        normalMap.m_normal[0]);
    m_points[0] = points[index0];
    points[index0].m_mark = 1;

    bool validTetrahedrum = false;
    VHACD::Vect3 e1(double(0.0));
    for (int i = 1; i < normalMap.m_count; ++i)
    {
        int index = SupportVertex(&tree,
            points,
            normalMap.m_normal[i]);
        assert(index >= 0);

        e1 = points[index] - m_points[0];
        double error2 = e1.GetNormSquared();
        if (error2 > (double(1.0e-4) * m_diag * m_diag))
        {
            m_points[1] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
            break;
        }
    }
    if (!validTetrahedrum)
    {
        m_points.resize(0);
        assert(0);
        return count;
    }

    validTetrahedrum = false;
    VHACD::Vect3 e2(double(0.0));
    VHACD::Vect3 normal(double(0.0));
    for (int i = 2; i < normalMap.m_count; ++i)
    {
        int index = SupportVertex(&tree,
            points,
            normalMap.m_normal[i]);
        assert(index >= 0);
        e2 = points[index] - m_points[0];
        normal = e1.Cross(e2);
        double error2 = normal.GetNorm();
        if (error2 > (double(1.0e-4) * m_diag * m_diag))
        {
            m_points[2] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
            break;
        }
    }

    if (!validTetrahedrum)
    {
        m_points.resize(0);
        assert(0);
        return count;
    }

    // find the largest possible tetrahedron
    validTetrahedrum = false;
    VHACD::Vect3 e3(double(0.0));

    index0 = SupportVertex(&tree,
        points,
        normal);
    e3 = points[index0] - m_points[0];
    double err2 = normal.Dot(e3);
    if (fabs(err2) > (double(1.0e-6) * m_diag * m_diag))
    {
        // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
        m_points[3] = points[index0];
        points[index0].m_mark = 1;
        validTetrahedrum = true;
    }
    if (!validTetrahedrum)
    {
        VHACD::Vect3 n(-normal);
        int index = SupportVertex(&tree,
            points,
            n);
        e3 = points[index] - m_points[0];
        double error2 = normal.Dot(e3);
        if (fabs(error2) > (double(1.0e-6) * m_diag * m_diag))
        {
            // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
            m_points[3] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
        }
    }
    if (!validTetrahedrum)
    {
        for (int i = 3; i < normalMap.m_count; ++i)
        {
            int index = SupportVertex(&tree,
                points,
                normalMap.m_normal[i]);
            assert(index >= 0);

            //make sure the volume of the fist tetrahedral is no negative
            e3 = points[index] - m_points[0];
            double error2 = normal.Dot(e3);
            if (fabs(error2) > (double(1.0e-6) * m_diag * m_diag))
            {
                // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
                m_points[3] = points[index];
                points[index].m_mark = 1;
                validTetrahedrum = true;
                break;
            }
        }
    }
    if (!validTetrahedrum)
    {
        // the points do not form a convex hull
        m_points.resize(0);
        return count;
    }

    m_points.resize(4);
    double volume = TetrahedrumVolume(m_points[0],
        m_points[1],
        m_points[2],
        m_points[3]);
    if (volume > double(0.0))
    {
        std::swap(m_points[2],
            m_points[3]);
    }
    assert(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0));
    return count;
}

std::list<ConvexHullFace>::iterator ConvexHull::AddFace(int i0,
    int i1,
    int i2)
{
    ConvexHullFace face;
    face.m_index[0] = i0;
    face.m_index[1] = i1;
    face.m_index[2] = i2;

    std::list<ConvexHullFace>::iterator node = m_list.emplace(m_list.end(), face);
    return node;
}

void ConvexHull::CalculateConvexHull3D(ConvexHullAABBTreeNode* vertexTree,
    std::vector<ConvexHullVertex>& points,
    int count,
    double distTol,
    int maxVertexCount)
{
    distTol = fabs(distTol) * m_diag;
    std::list<ConvexHullFace>::iterator f0Node = AddFace(0, 1, 2);
    std::list<ConvexHullFace>::iterator f1Node = AddFace(0, 2, 3);
    std::list<ConvexHullFace>::iterator f2Node = AddFace(2, 1, 3);
    std::list<ConvexHullFace>::iterator f3Node = AddFace(1, 0, 3);

    ConvexHullFace& f0 = *f0Node;
    ConvexHullFace& f1 = *f1Node;
    ConvexHullFace& f2 = *f2Node;
    ConvexHullFace& f3 = *f3Node;

    f0.m_twin[0] = f3Node;
    f0.m_twin[1] = f2Node;
    f0.m_twin[2] = f1Node;

    f1.m_twin[0] = f0Node;
    f1.m_twin[1] = f2Node;
    f1.m_twin[2] = f3Node;

    f2.m_twin[0] = f0Node;
    f2.m_twin[1] = f3Node;
    f2.m_twin[2] = f1Node;

    f3.m_twin[0] = f0Node;
    f3.m_twin[1] = f1Node;
    f3.m_twin[2] = f2Node;

    std::list<std::list<ConvexHullFace>::iterator> boundaryFaces;
    boundaryFaces.push_back(f0Node);
    boundaryFaces.push_back(f1Node);
    boundaryFaces.push_back(f2Node);
    boundaryFaces.push_back(f3Node);

    m_points.resize(count);

    count -= 4;
    maxVertexCount -= 4;
    int currentIndex = 4;

    /*
     * Some are iterators into boundaryFaces, others into m_list
     */
    std::vector<std::list<ConvexHullFace>::iterator> stack;
    std::vector<std::list<ConvexHullFace>::iterator> coneList;
    std::vector<std::list<ConvexHullFace>::iterator> deleteList;

    stack.reserve(1024 + count);
    coneList.reserve(1024 + count);
    deleteList.reserve(1024 + count);

    while (boundaryFaces.size() && count && (maxVertexCount > 0))
    {
        // my definition of the optimal convex hull of a given vertex count,
        // is the convex hull formed by a subset of the input vertex that minimizes the volume difference
        // between the perfect hull formed from all input vertex and the hull of the sub set of vertex.
        // When using a priority heap this algorithms will generate the an optimal of a fix vertex count.
        // Since all Newton's tools do not have a limit on the point count of a convex hull, I can use either a stack or a queue.
        // a stack maximize construction speed, a Queue tend to maximize the volume of the generated Hull approaching a perfect Hull.
        // For now we use a queue.
        // For general hulls it does not make a difference if we use a stack, queue, or a priority heap.
        // perfect optimal hull only apply for when build hull of a limited vertex count.
        //
        // Also when building Hulls of a limited vertex count, this function runs in constant time.
        // yes that is correct, it does not makes a difference if you build a N point hull from 100 vertex
        // or from 100000 vertex input array.

        // using a queue (some what slower by better hull when reduced vertex count is desired)
        bool isvalid;
        std::list<ConvexHullFace>::iterator faceNode = boundaryFaces.back();
        ConvexHullFace& face = *faceNode;
        HullPlane planeEquation(face.GetPlaneEquation(m_points, isvalid));

        int index = 0;
        double dist = 0;
        VHACD::Vect3 p;
        if (isvalid)
        {
            index = SupportVertex(&vertexTree,
                points,
                planeEquation);
            p = points[index];
            dist = planeEquation.Evalue(p);
        }

        if (isvalid
            && (dist >= distTol)
            && (face.Evalue(m_points, p) < double(0.0)))
        {
            stack.push_back(faceNode);

            deleteList.clear();
            while (stack.size())
            {
                std::list<ConvexHullFace>::iterator node1 = stack.back();
                ConvexHullFace& face1 = *node1;

                stack.pop_back();

                if (!face1.m_mark && (face1.Evalue(m_points, p) < double(0.0)))
                {
#ifdef _DEBUG
                    for (const auto node : deleteList)
                    {
                        assert(node != node1);
                    }
#endif

                    deleteList.push_back(node1);
                    face1.m_mark = 1;
                    for (std::list<ConvexHullFace>::iterator& twinNode : face1.m_twin)
                    {
                        ConvexHullFace& twinFace = *twinNode;
                        if (!twinFace.m_mark)
                        {
                            stack.push_back(twinNode);
                        }
                    }
                }
            }

            m_points[currentIndex] = points[index];
            points[index].m_mark = 1;

            coneList.clear();
            for (std::list<ConvexHullFace>::iterator node1 : deleteList)
            {
                ConvexHullFace& face1 = *node1;
                assert(face1.m_mark == 1);
                for (std::size_t j0 = 0; j0 < face1.m_twin.size(); ++j0)
                {
                    std::list<ConvexHullFace>::iterator twinNode = face1.m_twin[j0];
                    ConvexHullFace& twinFace = *twinNode;
                    if (!twinFace.m_mark)
                    {
                        std::size_t j1 = (j0 == 2) ? 0 : j0 + 1;
                        std::list<ConvexHullFace>::iterator newNode = AddFace(currentIndex,
                            face1.m_index[j0],
                            face1.m_index[j1]);
                        boundaryFaces.push_front(newNode);
                        ConvexHullFace& newFace = *newNode;

                        newFace.m_twin[1] = twinNode;
                        for (std::size_t k = 0; k < twinFace.m_twin.size(); ++k)
                        {
                            if (twinFace.m_twin[k] == node1)
                            {
                                twinFace.m_twin[k] = newNode;
                            }
                        }
                        coneList.push_back(newNode);
                    }
                }
            }

            for (std::size_t i = 0; i < coneList.size() - 1; ++i)
            {
                std::list<ConvexHullFace>::iterator nodeA = coneList[i];
                ConvexHullFace& faceA = *nodeA;
                assert(faceA.m_mark == 0);
                for (std::size_t j = i + 1; j < coneList.size(); j++)
                {
                    std::list<ConvexHullFace>::iterator nodeB = coneList[j];
                    ConvexHullFace& faceB = *nodeB;
                    assert(faceB.m_mark == 0);
                    if (faceA.m_index[2] == faceB.m_index[1])
                    {
                        faceA.m_twin[2] = nodeB;
                        faceB.m_twin[0] = nodeA;
                        break;
                    }
                }

                for (std::size_t j = i + 1; j < coneList.size(); j++)
                {
                    std::list<ConvexHullFace>::iterator nodeB = coneList[j];
                    ConvexHullFace& faceB = *nodeB;
                    assert(faceB.m_mark == 0);
                    if (faceA.m_index[1] == faceB.m_index[2])
                    {
                        faceA.m_twin[0] = nodeB;
                        faceB.m_twin[2] = nodeA;
                        break;
                    }
                }
            }

            for (std::list<ConvexHullFace>::iterator node : deleteList)
            {
                auto it = std::find(boundaryFaces.begin(),
                    boundaryFaces.end(),
                    node);
                if (it != boundaryFaces.end())
                {
                    boundaryFaces.erase(it);
                }
                m_list.erase(node);
            }

            maxVertexCount--;
            currentIndex++;
            count--;
        }
        else
        {
            auto it = std::find(boundaryFaces.begin(),
                boundaryFaces.end(),
                faceNode);
            if (it != boundaryFaces.end())
            {
                boundaryFaces.erase(it);
            }
        }
    }
    m_points.resize(currentIndex);
}

//***********************************************************************************************
// End of ConvexHull generation code by Julio Jerez <jerezjulio0@gmail.com>
//***********************************************************************************************
