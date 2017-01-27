#ifndef PEGAS_MATH_HPP
#define PEGAS_MATH_HPP

#include "Pegas/include/core.hpp"

namespace pegas {

class Vector3 {
public:
    real x;

    real y;

    real z;

    Vector3();

    Vector3(const real x, const real y, const real z);

    void operator*=(const real r);

    Vector3 operator*=(const real r) const;

    void operator+=(const Vector3 & v);

    Vector3 operator+=(const Vector3 & v) const;

    void operator-=(const Vector3 & v);

    Vector3 operator-=(const Vector3 & v) const;

    Vector3 operator-(const Vector3 & v) const;

    void addScaledVector(const Vector3 & v, const real s);

    void componentProduct(const Vector3 & v);

    Vector3 componentProduct(const Vector3 & v) const;

    real scalarProduct(const Vector3 & v) const;

    real operator*(const Vector3 & v) const;

    Vector3 vectorProduct(const Vector3 & v) const;

    void operator%(const Vector3 & v);

    Vector3 operator%(const Vector3 & v) const;

    void inverse();

    Vector3 inverse() const;

    real magnitude() const;

    real squareMagnitude() const;

    void normalize();

    Vector3 normalize() const;

private:
    real pad;

};

}

#endif //PEGAS_MATH_HPP
