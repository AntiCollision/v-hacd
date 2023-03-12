#pragma once

/*
 * Getters
 */
template <typename T>
inline T& Vector3<T>::operator[](size_t i)
{
    return m_data[i];
}

template <typename T>
inline const T& Vector3<T>::operator[](size_t i) const
{
    return m_data[i];
}

template <typename T>
inline T& Vector3<T>::GetX()
{
    return m_data[0];
}

template <typename T>
inline T& Vector3<T>::GetY()
{
    return m_data[1];
}

template <typename T>
inline T& Vector3<T>::GetZ()
{
    return m_data[2];
}

template <typename T>
inline const T& Vector3<T>::GetX() const
{
    return m_data[0];
}

template <typename T>
inline const T& Vector3<T>::GetY() const
{
    return m_data[1];
}

template <typename T>
inline const T& Vector3<T>::GetZ() const
{
    return m_data[2];
}

/*
 * Normalize and norming
 */
template <typename T>
inline T Vector3<T>::Normalize()
{
    T n = GetNorm();
    if (n != T(0.0)) (*this) /= n;
    return n;
}

template <typename T>
inline Vector3<T> Vector3<T>::Normalized()
{
    Vector3<T> ret = *this;
    T n = GetNorm();
    if (n != T(0.0)) ret /= n;
    return ret;
}

template <typename T>
inline T Vector3<T>::GetNorm() const
{
    return std::sqrt(GetNormSquared());
}

template <typename T>
inline T Vector3<T>::GetNormSquared() const
{
    return this->Dot(*this);
}

template <typename T>
inline int Vector3<T>::LongestAxis() const
{
    auto it = std::max_element(m_data.begin(), m_data.end());
    return int(std::distance(m_data.begin(), it));
}

/*
 * Vector-vector operations
 */
template <typename T>
inline Vector3<T>& Vector3<T>::operator=(const Vector3<T>& rhs)
{
    GetX() = rhs.GetX();
    GetY() = rhs.GetY();
    GetZ() = rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& rhs)
{
    GetX() += rhs.GetX();
    GetY() += rhs.GetY();
    GetZ() += rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& rhs)
{
    GetX() -= rhs.GetX();
    GetY() -= rhs.GetY();
    GetZ() -= rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMul(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() * rhs.GetX(),
        GetY() * rhs.GetY(),
        GetZ() * rhs.GetZ());
}

template <typename T>
inline Vector3<T> Vector3<T>::Cross(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetY() * rhs.GetZ() - GetZ() * rhs.GetY(),
        GetZ() * rhs.GetX() - GetX() * rhs.GetZ(),
        GetX() * rhs.GetY() - GetY() * rhs.GetX());
}

template <typename T>
inline T Vector3<T>::Dot(const Vector3<T>& rhs) const
{
    return   GetX() * rhs.GetX()
        + GetY() * rhs.GetY()
        + GetZ() * rhs.GetZ();
}

template <typename T>
inline Vector3<T> Vector3<T>::operator+(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() + rhs.GetX(),
        GetY() + rhs.GetY(),
        GetZ() + rhs.GetZ());
}

template <typename T>
inline Vector3<T> Vector3<T>::operator-(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() - rhs.GetX(),
        GetY() - rhs.GetY(),
        GetZ() - rhs.GetZ());
}

template <typename T>
inline Vector3<T> operator*(T lhs, const Vector3<T>& rhs)
{
    return Vector3<T>(lhs * rhs.GetX(),
        lhs * rhs.GetY(),
        lhs * rhs.GetZ());
}

/*
 * Vector-scalar operations
 */
template <typename T>
inline Vector3<T>& Vector3<T>::operator-=(T a)
{
    GetX() -= a;
    GetY() -= a;
    GetZ() -= a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator+=(T a)
{
    GetX() += a;
    GetY() += a;
    GetZ() += a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator/=(T a)
{
    GetX() /= a;
    GetY() /= a;
    GetZ() /= a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator*=(T a)
{
    GetX() *= a;
    GetY() *= a;
    GetZ() *= a;
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::operator*(T rhs) const
{
    return Vector3<T>(GetX() * rhs,
        GetY() * rhs,
        GetZ() * rhs);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator/(T rhs) const
{
    return Vector3<T>(GetX() / rhs,
        GetY() / rhs,
        GetZ() / rhs);
}

/*
 * Unary operations
 */
template <typename T>
inline Vector3<T> Vector3<T>::operator-() const
{
    return Vector3<T>(-GetX(),
        -GetY(),
        -GetZ());
}

/*
 * Comparison operators
 */
template <typename T>
inline bool Vector3<T>::operator<(const Vector3<T>& rhs) const
{
    if (GetX() == rhs.GetX())
    {
        if (GetY() == rhs.GetY())
        {
            return (GetZ() < rhs.GetZ());
        }
        return (GetY() < rhs.GetY());
    }
    return (GetX() < rhs.GetX());
}

template <typename T>
inline bool Vector3<T>::operator>(const Vector3<T>& rhs) const
{
    if (GetX() == rhs.GetX())
    {
        if (GetY() == rhs.GetY())
        {
            return (GetZ() > rhs.GetZ());
        }
        return (GetY() > rhs.GetY());
    }
    return (GetX() > rhs.GetZ());
}

template <typename T>
inline bool Vector3<T>::CWiseAllGE(const Vector3<T>& rhs) const
{
    return    GetX() >= rhs.GetX()
        && GetY() >= rhs.GetY()
        && GetZ() >= rhs.GetZ();
}

template <typename T>
inline bool Vector3<T>::CWiseAllLE(const Vector3<T>& rhs) const
{
    return    GetX() <= rhs.GetX()
        && GetY() <= rhs.GetY()
        && GetZ() <= rhs.GetZ();
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMin(const Vector3<T>& rhs) const
{
    return Vector3<T>(std::min(GetX(), rhs.GetX()),
        std::min(GetY(), rhs.GetY()),
        std::min(GetZ(), rhs.GetZ()));
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMax(const Vector3<T>& rhs) const
{
    return Vector3<T>(std::max(GetX(), rhs.GetX()),
        std::max(GetY(), rhs.GetY()),
        std::max(GetZ(), rhs.GetZ()));
}

template <typename T>
inline T Vector3<T>::MinCoeff() const
{
    return *std::min_element(m_data.begin(), m_data.end());
}

template <typename T>
inline T Vector3<T>::MaxCoeff() const
{
    return *std::max_element(m_data.begin(), m_data.end());
}

template <typename T>
inline T Vector3<T>::MinCoeff(uint32_t& idx) const
{
    auto it = std::min_element(m_data.begin(), m_data.end());
    idx = uint32_t(std::distance(m_data.begin(), it));
    return *it;
}

template <typename T>
inline T Vector3<T>::MaxCoeff(uint32_t& idx) const
{
    auto it = std::max_element(m_data.begin(), m_data.end());
    idx = uint32_t(std::distance(m_data.begin(), it));
    return *it;
}

/*
 * Constructors
 */
template <typename T>
inline Vector3<T>::Vector3(T a)
    : m_data{ a, a, a }
{
}

template <typename T>
inline Vector3<T>::Vector3(T x, T y, T z)
    : m_data{ x, y, z }
{
}

template <typename T>
inline Vector3<T>::Vector3(const Vector3& rhs)
    : m_data{ rhs.m_data }
{
}

template <typename T>
template <typename U>
inline Vector3<T>::Vector3(const Vector3<U>& rhs)
    : m_data{ T(rhs.GetX()), T(rhs.GetY()), T(rhs.GetZ()) }
{
}

template <typename T>
inline Vector3<T>::Vector3(const VHACD::Vertex& rhs)
    : Vector3<T>(rhs.mX, rhs.mY, rhs.mZ)
{
    static_assert(std::is_same<T, double>::value, "Vertex to Vector3 constructor only enabled for double");
}

template <typename T>
inline Vector3<T>::Vector3(const VHACD::Triangle& rhs)
    : Vector3<T>(rhs.mI0, rhs.mI1, rhs.mI2)
{
    static_assert(std::is_same<T, uint32_t>::value, "Triangle to Vector3 constructor only enabled for uint32_t");
}

template <typename T>
inline Vector3<T>::operator VHACD::Vertex() const
{
    static_assert(std::is_same<T, double>::value, "Vector3 to Vertex conversion only enable for double");
    return ::VHACD::Vertex(GetX(), GetY(), GetZ());
}
