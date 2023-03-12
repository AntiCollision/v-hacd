#pragma once
#include <cstddef>
#include <list>
#include <assert.h>

namespace VHACD {
    /*
     * To minimize memory allocations while maintaining pointer stability.
     * Used in KdTreeNode and ConvexHull, as both use tree data structures that rely on pointer stability
     * Neither rely on random access or iteration
     * They just dump elements into a memory pool, then refer to pointers to the elements
     * All elements are default constructed in NodeStorage's m_nodes array
     */
    template <typename T, std::size_t MaxBundleSize = 1024>
    class NodeBundle
    {
        struct NodeStorage {
            bool IsFull() const;

            T& GetNextNode();

            std::size_t m_index;
            std::array<T, MaxBundleSize> m_nodes;
        };

        std::list<NodeStorage> m_list;
        typename std::list<NodeStorage>::iterator m_head { m_list.end() };

    public:
        T& GetNextNode();

        T& GetFirstNode();

        void Clear();
    };

    #include "NodeBundle.inl"
}
