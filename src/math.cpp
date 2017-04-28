#include "Pegasus/include/math.hpp"

#include <cmath>

pegasus::Vector3::Vector3()
    : x(0)
    , y(0)
    , z(0)
    , pad(0)
{
}

pegasus::Vector3::Vector3(pegasus::real const x, pegasus::real const y,
    pegasus::real const z)
    : x(x)
    , y(y)
    , z(z)
    , pad(0)
{
}

void pegasus::Vector3::operator*=(pegasus::real const r)
{
    x *= r;
    y *= r;
    z *= r;
}

pegasus::Vector3 pegasus::Vector3::operator*=(pegasus::real const r) const
{
    auto v(*this);
    v *= r;
    return v;
}

void pegasus::Vector3::operator+=(pegasus::Vector3 const& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

pegasus::Vector3 pegasus::Vector3::operator+=(pegasus::Vector3 v) const
{
    v += *this;
    return v;
}

pegasus::Vector3 pegasus::Vector3::operator+(const pegasus::Vector3& v) const
{
    return (*this) += v;
}

pegasus::Vector3 pegasus::Vector3::operator-=(pegasus::Vector3 const& v) const
{
    auto new_v(*this);
    new_v -= v;
    return new_v;
}

pegasus::Vector3 pegasus::Vector3::operator-(pegasus::Vector3 const& v) const
{
    auto new_v(*this);
    new_v -= v;
    return new_v;
}

void pegasus::Vector3::addScaledVector(pegasus::Vector3 const& v,
    pegasus::real const s)
{
    x += v.x * s;
    y += v.y * s;
    z += v.z * s;
}

void pegasus::Vector3::componentProduct(pegasus::Vector3 const& v)
{
    x *= v.x;
    y *= v.y;
    z *= v.z;
}

pegasus::Vector3 pegasus::Vector3::componentProduct(pegasus::Vector3 const& v) const
{
    auto new_v(*this);
    new_v.componentProduct(v);
    return new_v;
}

pegasus::real pegasus::Vector3::scalarProduct(pegasus::Vector3 const& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

pegasus::real pegasus::Vector3::operator*(pegasus::Vector3 const& v) const
{
    return scalarProduct(v);
}

pegasus::Vector3 pegasus::Vector3::operator*(pegasus::real const r) const
{
    auto new_v(*this);
    new_v *= r;
    return new_v;
}

pegasus::Vector3 pegasus::Vector3::vectorProduct(pegasus::Vector3 const& v) const
{
    return Vector3{ y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x };
}

void pegasus::Vector3::operator%=(pegasus::Vector3 const& v)
{
    *this = vectorProduct(v);
}

pegasus::Vector3 pegasus::Vector3::operator%(pegasus::Vector3 const& v) const
{
    return vectorProduct(v);
}

void pegasus::Vector3::operator-=(pegasus::Vector3 const& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
}

pegasus::Vector3 pegasus::Vector3::inverse() const
{
    auto v(*this);
    v.inverse();
    return v;
}

pegasus::real pegasus::Vector3::magnitude() const
{
    return std::sqrt(x * x + y * y + z * z);
}

pegasus::real pegasus::Vector3::squareMagnitude() const
{
    return (x * x + y * y + z * z);
}

void pegasus::Vector3::normalize()
{
    auto const l = magnitude();
    if (l > 0) {
        (*this) *= (real(1) / l);
    }
}

pegasus::Vector3 pegasus::Vector3::normalize() const
{
    auto v(*this);
    v.normalize();
    return v;
}

pegasus::Vector3 pegasus::Vector3::unit() const
{
    auto result = *this;
    result.normalize();
    return result;
}

void pegasus::Vector3::trim(pegasus::real const size)
{
    if (squareMagnitude() > size * size) {
        normalize();
        x *= size;
        y *= size;
        z *= size;
    }
}
