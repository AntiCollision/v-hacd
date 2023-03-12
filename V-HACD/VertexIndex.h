#pragma once

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