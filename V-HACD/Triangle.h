#pragma once
#include <stdint.h>

namespace VHACD {
    struct Triangle
    {
        uint32_t mI0;
        uint32_t mI1;
        uint32_t mI2;

        Triangle() = default;
        Triangle(uint32_t i0, uint32_t i1, uint32_t i2) : mI0(i0), mI1(i1), mI2(i2) {}
    };
}