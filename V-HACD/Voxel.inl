#pragma once

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