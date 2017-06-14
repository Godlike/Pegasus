/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/Math.hpp"

#include <cmath>

pegasus::Vector3::Vector3()
    : x(0)
    , y(0)
    , z(0)
{
}

pegasus::Vector3::Vector3(double x, double y, double z)
    : x(x)
    , y(y)
    , z(z)
{
}

bool pegasus::Vector3::operator==(const pegasus::Vector3& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

void pegasus::Vector3::operator*=(double r)
{
    x *= r;
    y *= r;
    z *= r;
}

pegasus::Vector3 pegasus::Vector3::operator*=(double r) const
{
    Vector3 v(*this);
    v *= r;
    return v;
}

void pegasus::Vector3::operator+=(Vector3 const& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

pegasus::Vector3 pegasus::Vector3::operator+=(Vector3 v) const
{
    v += *this;
    return v;
}

pegasus::Vector3 pegasus::Vector3::operator+(const Vector3& v) const
{
    return (*this) += v;
}

pegasus::Vector3 pegasus::Vector3::operator-=(Vector3 const& v) const
{
    Vector3 new_v(*this);
    new_v -= v;
    return new_v;
}

pegasus::Vector3 pegasus::Vector3::operator-(Vector3 const& v) const
{
    Vector3 new_v(*this);
    new_v -= v;
    return new_v;
}

void pegasus::Vector3::AddScaledVector(Vector3 const& v, double s)
{
    x += v.x * s;
    y += v.y * s;
    z += v.z * s;
}

void pegasus::Vector3::ComponentProduct(Vector3 const& v)
{
    x *= v.x;
    y *= v.y;
    z *= v.z;
}

pegasus::Vector3 pegasus::Vector3::ComponentProduct(Vector3 const& v) const
{
    Vector3 new_v(*this);
    new_v.ComponentProduct(v);
    return new_v;
}

double pegasus::Vector3::ScalarProduct(Vector3 const& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

double pegasus::Vector3::operator*(Vector3 const& v) const
{
    return ScalarProduct(v);
}

pegasus::Vector3 pegasus::Vector3::operator*(double r) const
{
    auto new_v(*this);
    new_v *= r;
    return new_v;
}

pegasus::Vector3 pegasus::Vector3::VectorProduct(Vector3 const& v) const
{
    return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
}

void pegasus::Vector3::operator%=(Vector3 const& v)
{
    *this = VectorProduct(v);
}

pegasus::Vector3 pegasus::Vector3::operator%(Vector3 const& v) const
{
    return VectorProduct(v);
}

void pegasus::Vector3::operator-=(Vector3 const& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
}

pegasus::Vector3 pegasus::Vector3::Inverse() const
{
    Vector3 v(*this);
    v.x = -v.x;
    v.y = -v.y;
    v.z = -v.z;
    return v;
}

double pegasus::Vector3::Magnitude() const
{
    return sqrt(x * x + y * y + z * z);
}

double pegasus::Vector3::SquareMagnitude() const
{
    return (x * x + y * y + z * z);
}

void pegasus::Vector3::Normalize()
{
    double const l = Magnitude();
    if (l > 0)
    {
        (*this) *= (double(1) / l);
    }
}

pegasus::Vector3 pegasus::Vector3::Unit() const
{
    Vector3 result(*this);
    result.Normalize();
    return result;
}

void pegasus::Vector3::Trim(double size)
{
    if (SquareMagnitude() > size * size)
    {
        Normalize();
        x *= size;
        y *= size;
        z *= size;
    }
}
