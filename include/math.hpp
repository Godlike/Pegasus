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

    Vector3(real const x, real const y, real const z);

    void operator*=(real const r);

    Vector3 operator*=(real const r) const;

    void operator+=(Vector3 const& v);

    Vector3 operator+=(Vector3 v) const;

    Vector3 operator+(Vector3 const& v) const;

    void operator-=(Vector3 const& v);

    Vector3 operator-=(Vector3 const& v) const;

    Vector3 operator-(Vector3 const& v) const;

    void addScaledVector(Vector3 const& v, real const s);

    void componentProduct(Vector3 const& v);

    Vector3 componentProduct(Vector3 const& v) const;

    real scalarProduct(Vector3 const& v) const;

    real operator*(Vector3 const& v) const;

    Vector3 operator*(real const r) const;

    Vector3 vectorProduct(Vector3 const& v) const;

    void operator%(Vector3 const& v);

    Vector3 operator%(Vector3 const& v) const;

    void inverse();

    Vector3 inverse() const;

    real magnitude() const;

    real squareMagnitude() const;

    void normalize();

    Vector3 normalize() const;

    Vector3 unit() const;

    void trim(real const size);

private:
    real pad;
};
}

#endif // PEGAS_MATH_HPP
