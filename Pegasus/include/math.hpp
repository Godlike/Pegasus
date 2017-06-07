/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

namespace pegasus
{
class Vector3
{
public:
    double x;
    double y;
    double z;

    Vector3();
    Vector3(double x, double y, double z);

    bool operator==(Vector3 const& other) const;

    void operator*=(double r);
    Vector3 operator*=(double r) const;

    void operator+=(Vector3 const& v);
    Vector3 operator+=(Vector3 v) const;
    Vector3 operator+(Vector3 const& v) const;

    void operator-=(Vector3 const& v);
    Vector3 operator-=(Vector3 const& v) const;
    Vector3 operator-(Vector3 const& v) const;

    void AddScaledVector(Vector3 const& v, double s);

    void ComponentProduct(Vector3 const& v);
    Vector3 ComponentProduct(Vector3 const& v) const;

    double ScalarProduct(Vector3 const& v) const;
    double operator*(Vector3 const& v) const;
    Vector3 operator*(double r) const;

    Vector3 VectorProduct(Vector3 const& v) const;
    void operator%=(Vector3 const& v);
    Vector3 operator%(Vector3 const& v) const;

    Vector3 Inverse() const;

    double Magnitude() const;
    double SquareMagnitude() const;

    void Normalize();
    Vector3 Unit() const;
    void Trim(double size);
};
} // namespace pegasus 

#endif // PEGASUS_MATH_HPP
