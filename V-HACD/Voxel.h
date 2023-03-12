#pragma once

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
    static constexpr int VoxelBitsZStart = 0;
    static constexpr int VoxelBitsYStart = 10;
    static constexpr int VoxelBitsXStart = 20;
    static constexpr int VoxelBitMask = 0x03FF; // bits 0 through 9 inclusive
public:
    Voxel() = default;

    Voxel(uint32_t index);

    Voxel(uint32_t x,
        uint32_t y,
        uint32_t z);

    bool operator==(const Voxel& v) const;

    VHACD::Vector3<uint32_t> GetVoxel() const;

    uint32_t GetX() const;
    uint32_t GetY() const;
    uint32_t GetZ() const;

    uint32_t GetVoxelAddress() const;

private:
    uint32_t m_voxel{ 0 };
};
