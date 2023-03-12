#pragma once

KdTreeNode::KdTreeNode(uint32_t index)
    : m_index(index)
{
}

void KdTreeNode::Add(KdTreeNode& node,
    Axes dim,
    const KdTree& tree)
{
    Axes axis = X_AXIS;
    uint32_t idx = 0;
    switch (dim)
    {
    case X_AXIS:
        idx = 0;
        axis = Y_AXIS;
        break;
    case Y_AXIS:
        idx = 1;
        axis = Z_AXIS;
        break;
    case Z_AXIS:
        idx = 2;
        axis = X_AXIS;
        break;
    }

    const VHACD::Vertex& nodePosition = tree.GetPosition(node.m_index);
    const VHACD::Vertex& position = tree.GetPosition(m_index);
    if (nodePosition[idx] <= position[idx])
    {
        if (m_left)
            m_left->Add(node, axis, tree);
        else
            m_left = &node;
    }
    else
    {
        if (m_right)
            m_right->Add(node, axis, tree);
        else
            m_right = &node;
    }
}

uint32_t KdTreeNode::GetIndex() const
{
    return m_index;
}

void KdTreeNode::Search(Axes axis,
    const VHACD::Vect3& pos,
    double radius,
    uint32_t& count,
    uint32_t maxObjects,
    KdTreeFindNode* found,
    const KdTree& iface)
{
    const VHACD::Vect3 position = iface.GetPosition(m_index);

    const VHACD::Vect3 d = pos - position;

    KdTreeNode* search1 = 0;
    KdTreeNode* search2 = 0;

    uint32_t idx = 0;
    switch (axis)
    {
    case X_AXIS:
        idx = 0;
        axis = Y_AXIS;
        break;
    case Y_AXIS:
        idx = 1;
        axis = Z_AXIS;
        break;
    case Z_AXIS:
        idx = 2;
        axis = X_AXIS;
        break;
    }

    if (d[idx] <= 0) // JWR  if we are to the left
    {
        search1 = m_left; // JWR  then search to the left
        if (-d[idx] < radius) // JWR  if distance to the right is less than our search radius, continue on the right
            // as well.
            search2 = m_right;
    }
    else
    {
        search1 = m_right; // JWR  ok, we go down the left tree
        if (d[idx] < radius) // JWR  if the distance from the right is less than our search radius
            search2 = m_left;
    }

    double r2 = radius * radius;
    double m = d.GetNormSquared();

    if (m < r2)
    {
        switch (count)
        {
        case 0:
        {
            found[count].m_node = this;
            found[count].m_distance = m;
            break;
        }
        case 1:
        {
            if (m < found[0].m_distance)
            {
                if (maxObjects == 1)
                {
                    found[0].m_node = this;
                    found[0].m_distance = m;
                }
                else
                {
                    found[1] = found[0];
                    found[0].m_node = this;
                    found[0].m_distance = m;
                }
            }
            else if (maxObjects > 1)
            {
                found[1].m_node = this;
                found[1].m_distance = m;
            }
            break;
        }
        default:
        {
            bool inserted = false;

            for (uint32_t i = 0; i < count; i++)
            {
                if (m < found[i].m_distance) // if this one is closer than a pre-existing one...
                {
                    // insertion sort...
                    uint32_t scan = count;
                    if (scan >= maxObjects)
                        scan = maxObjects - 1;
                    for (uint32_t j = scan; j > i; j--)
                    {
                        found[j] = found[j - 1];
                    }
                    found[i].m_node = this;
                    found[i].m_distance = m;
                    inserted = true;
                    break;
                }
            }

            if (!inserted && count < maxObjects)
            {
                found[count].m_node = this;
                found[count].m_distance = m;
            }
        }
        break;
        }

        count++;

        if (count > maxObjects)
        {
            count = maxObjects;
        }
    }


    if (search1)
        search1->Search(axis, pos, radius, count, maxObjects, found, iface);

    if (search2)
        search2->Search(axis, pos, radius, count, maxObjects, found, iface);
}

