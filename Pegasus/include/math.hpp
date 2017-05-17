#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

namespace pegasus {

class Vector3 {
public:
    double x;
    double y;
    double z;

    Vector3();
    Vector3(double x, double y, double z);

    void operator*=(double r);
    Vector3 operator*=(double r) const;

    void operator+=(Vector3 const& v);
    Vector3 operator+=(Vector3 v) const;
    Vector3 operator+(Vector3 const& v) const;

    void operator-=(Vector3 const& v);
    Vector3 operator-=(Vector3 const& v) const;
    Vector3 operator-(Vector3 const& v) const;

    void addScaledVector(Vector3 const& v, double s);

    void componentProduct(Vector3 const& v);
    Vector3 componentProduct(Vector3 const& v) const;

    double scalarProduct(Vector3 const& v) const;
    double operator*(Vector3 const& v) const;
    Vector3 operator*(double r) const;

    Vector3 vectorProduct(Vector3 const& v) const;
    void operator%=(Vector3 const& v);
    Vector3 operator%(Vector3 const& v) const;

    Vector3 inverse() const;

    double magnitude() const;
    double squareMagnitude() const;

    void normalize();
    Vector3 normalize() const;
    Vector3 unit() const;
    void trim(double size);

};

} // namespace pegasus 

#endif // PEGASUS_MATH_HPP
