#pragma once

namespace VHACD {
    template <typename T>
    class Vector3
    {
    public:
        /*
        * Getters
        */
        T& operator[](size_t i);
        const T& operator[](size_t i) const;
        T& GetX();
        T& GetY();
        T& GetZ();
        const T& GetX() const;
        const T& GetY() const;
        const T& GetZ() const;

        /*
        * Normalize and norming
        */
        T Normalize();
        Vector3 Normalized();
        T GetNorm() const;
        T GetNormSquared() const;
        int LongestAxis() const;

        /*
        * Vector-vector operations
        */
        Vector3& operator=(const Vector3& rhs);
        Vector3& operator+=(const Vector3& rhs);
        Vector3& operator-=(const Vector3& rhs);

        Vector3 CWiseMul(const Vector3& rhs) const;
        Vector3 Cross(const Vector3& rhs) const;
        T Dot(const Vector3& rhs) const;
        Vector3 operator+(const Vector3& rhs) const;
        Vector3 operator-(const Vector3& rhs) const;

        /*
        * Vector-scalar operations
        */
        Vector3& operator-=(T a);
        Vector3& operator+=(T a);
        Vector3& operator/=(T a);
        Vector3& operator*=(T a);

        Vector3 operator*(T rhs) const;
        Vector3 operator/(T rhs) const;

        /*
        * Unary operations
        */
        Vector3 operator-() const;

        /*
        * Comparison operators
        */
        bool operator<(const Vector3& rhs) const;
        bool operator>(const Vector3& rhs) const;

        /*
         * Returns true if all elements of *this are greater than or equal to all elements of rhs, coefficient wise
         * LE is less than or equal
         */
        bool CWiseAllGE(const Vector3<T>& rhs) const;
        bool CWiseAllLE(const Vector3<T>& rhs) const;

        Vector3 CWiseMin(const Vector3& rhs) const;
        Vector3 CWiseMax(const Vector3& rhs) const;
        T MinCoeff() const;
        T MaxCoeff() const;

        T MinCoeff(uint32_t& idx) const;
        T MaxCoeff(uint32_t& idx) const;

        /*
        * Constructors
        */
        Vector3() = default;
        Vector3(T a);
        Vector3(T x, T y, T z);
        Vector3(const Vector3& rhs);
        ~Vector3() = default;

        template <typename U>
        Vector3(const Vector3<U>& rhs);

        Vector3(const VHACD::Vertex&);
        Vector3(const VHACD::Triangle&);

        operator VHACD::Vertex() const;

    private:
        std::array<T, 3> m_data{ T(0.0) };
    };
    
    #include "Vector3.inl"
}

