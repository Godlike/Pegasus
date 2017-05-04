#include "Pegasus/include/math.hpp"

#include <cmath>

pegas::Vector3::Vector3()
    : x(0)
    , y(0)
    , z(0)
    , pad(0)
{
}

pegas::Vector3::Vector3(pegas::real const x, pegas::real const y,
    pegas::real const z)
    : x(x)
    , y(y)
    , z(z)
    , pad(0)
{
}

void pegas::Vector3::operator*=(pegas::real const r)
{
    x *= r;
    y *= r;
    z *= r;
}

pegas::Vector3 pegas::Vector3::operator*=(pegas::real const r) const
{
    auto v(*this);
    v *= r;
    return v;
}

void pegas::Vector3::operator+=(pegas::Vector3 const& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

pegas::Vector3 pegas::Vector3::operator+=(pegas::Vector3 v) const
{
    v += *this;
    return v;
}

pegas::Vector3 pegas::Vector3::operator+(const pegas::Vector3& v) const
{
    return (*this) += v;
}

pegas::Vector3 pegas::Vector3::operator-=(pegas::Vector3 const& v) const
{
    auto new_v(*this);
    new_v -= v;
    return new_v;
}

pegas::Vector3 pegas::Vector3::operator-(pegas::Vector3 const& v) const
{
    auto new_v(*this);
    new_v -= v;
    return new_v;
}

void pegas::Vector3::addScaledVector(pegas::Vector3 const& v,
    pegas::real const s)
{
    x += v.x * s;
    y += v.y * s;
    z += v.z * s;
}

void pegas::Vector3::componentProduct(pegas::Vector3 const& v)
{
    x *= v.x;
    y *= v.y;
    z *= v.z;
}

pegas::Vector3 pegas::Vector3::componentProduct(pegas::Vector3 const& v) const
{
    auto new_v(*this);
    new_v.componentProduct(v);
    return new_v;
}

pegas::real pegas::Vector3::scalarProduct(pegas::Vector3 const& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

pegas::real pegas::Vector3::operator*(pegas::Vector3 const& v) const
{
    return scalarProduct(v);
}

pegas::Vector3 pegas::Vector3::operator*(pegas::real const r) const
{
    auto new_v(*this);
    new_v *= r;
    return new_v;
}

pegas::Vector3 pegas::Vector3::vectorProduct(pegas::Vector3 const& v) const
{
    return Vector3{ y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x };
}

void pegas::Vector3::operator%=(pegas::Vector3 const& v)
{
    *this = vectorProduct(v);
}

pegas::Vector3 pegas::Vector3::operator%(pegas::Vector3 const& v) const
{
    return vectorProduct(v);
}

void pegas::Vector3::operator-=(pegas::Vector3 const& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
}

pegas::Vector3 pegas::Vector3::inverse() const
{
    auto v(*this);
    v.inverse();
    return v;
}

pegas::real pegas::Vector3::magnitude() const
{
    return std::sqrt(x * x + y * y + z * z);
}

pegas::real pegas::Vector3::squareMagnitude() const
{
    return (x * x + y * y + z * z);
}

void pegas::Vector3::normalize()
{
    auto const l = magnitude();
    if (l > 0) {
        (*this) *= (real(1) / l);
    }
}

pegas::Vector3 pegas::Vector3::normalize() const
{
    auto v(*this);
    v.normalize();
    return v;
}

pegas::Vector3 pegas::Vector3::unit() const
{
    auto result = *this;
    result.normalize();
    return result;
}

void pegas::Vector3::trim(pegas::real const size)
{
    if (squareMagnitude() > size * size) {
        normalize();
        x *= size;
        y *= size;
        z *= size;
    }
}
