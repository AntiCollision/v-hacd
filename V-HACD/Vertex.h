#pragma once    

namespace VHACD {
    struct Vertex
    {
        double mX;
        double mY;
        double mZ;

        Vertex() = default;
        Vertex(double x, double y, double z) : mX(x), mY(y), mZ(z) {}

        const double& operator[](size_t idx) const
        {
            switch (idx)
            {
            case 0: return mX;
            case 1: return mY;
            case 2: return mZ;
            };
            return mX;
        }
    };
}
