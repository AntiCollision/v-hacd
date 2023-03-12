#pragma once

template <typename T, std::size_t MaxBundleSize>
bool NodeBundle<T, MaxBundleSize>::NodeStorage::IsFull() const
{
    return m_index == MaxBundleSize;
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::NodeStorage::GetNextNode()
{
    assert(m_index < MaxBundleSize);
    T& ret = m_nodes[m_index];
    m_index++;
    return ret;
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::GetNextNode()
{
    /*
     * || short circuits, so doesn't dereference if m_bundle == m_bundleHead.end()
     */
    if (m_head == m_list.end()
        || m_head->IsFull())
    {
        m_head = m_list.emplace(m_list.end());
    }

    return m_head->GetNextNode();
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::GetFirstNode()
{
    assert(m_head != m_list.end());
    return m_list.front().m_nodes[0];
}

template <typename T, std::size_t MaxBundleSize>
void NodeBundle<T, MaxBundleSize>::Clear()
{
    m_list.clear();
}
