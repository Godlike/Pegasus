#include "Pegas/include/geometry.hpp"


pegas::Geometry::~Geometry() {}


pegas::Shape::Shape(pegas::Vector3 const & centerOfMass)
    : mCenterOfMass(centerOfMass)
{
}

void pegas::Shape::setCenterOfMass(pegas::Vector3 const & centerOfMass)
{
    mCenterOfMass = centerOfMass;
}

pegas::Vector3 pegas::Shape::getCenterOfMass() const
{
    return mCenterOfMass;
}

pegas::SimpleShape::SimpleShape(pegas::Vector3 const & centerOfMass)
    : Shape(centerOfMass)
{
}


pegas::Plane::Plane(pegas::Vector3 const & centerOfMass, pegas::Vector3 const & normal)
    : SimpleShape(centerOfMass), mNormal(normal)
{
}

void pegas::Plane::setNormal(pegas::Vector3 const & normal)
{
    mNormal = normal;
}

pegas::Vector3 pegas::Plane::getNormal() const
{
    return mNormal;
}


pegas::Triangle::Triangle(const pegas::Vector3 & a, const pegas::Vector3 & b, const pegas::Vector3 & c)
    : SimpleShape((a + b + c) * real(1.0f / 3.0f)), mA(a), mB(b), mC(c)
{
}

void pegas::Triangle::setAxes(const pegas::Vector3 & a, const pegas::Vector3 & b, const pegas::Vector3 & c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegas::Triangle::getAxes(pegas::Vector3 & a, pegas::Vector3 & b, pegas::Vector3 & c) const
{
    a = mA;
    b = mB;
    c = mC;
}


pegas::Sphere::Sphere(pegas::Vector3 const & centerOfMass, pegas::real const r) :
    SimpleShape(centerOfMass), mR(r)
{
}

void pegas::Sphere::setRadius(pegas::real const r)
{
    mR = r;
}

pegas::real pegas::Sphere::getRadius() const
{
    return mR;
}


pegas::Cone::Cone(const pegas::Vector3 & centerOfMass, const pegas::Vector3 & a, const pegas::real h, const pegas::real r)
    : SimpleShape(centerOfMass), mA(a), mH(h), mR(r)
{
}

void pegas::Cone::setAppex(const pegas::Vector3 & a)
{
    mA = a;
}

pegas::Vector3 pegas::Cone::getAppex() const
{
    return mA;
}

void pegas::Cone::setHeight(const pegas::real h)
{
    mH = h;
}

pegas::real pegas::Cone::getHeight() const
{
    return mH;
}

void pegas::Cone::setRadius(const pegas::real r)
{
    mR = r;
}

pegas::real pegas::Cone::getRadius() const
{
    return mR;
}


pegas::Capsule::Capsule(const pegas::Vector3 & centerOfMass, const pegas::Vector3 & halfHeight, const pegas::real r)
    : SimpleShape(centerOfMass), mHalfHeight(halfHeight), mR(r)
{
}

void pegas::Capsule::setHalfHeight(const pegas::Vector3 & halfHeight)
{
    mHalfHeight = halfHeight;
}

pegas::Vector3 pegas::Capsule::getHalfHeight() const
{
    return mHalfHeight;
}

void pegas::Capsule::setRadius(const pegas::real r)
{
    mR = r;
}

pegas::real pegas::Capsule::getRadius() const
{
    return mR;
}


pegas::Box::Box(const pegas::Vector3 & centerOfMass, const pegas::Vector3 & a, const pegas::Vector3 & b, const pegas::Vector3 & c)
    : SimpleShape(centerOfMass), mA(a), mB(b), mC(c)
{
}

void pegas::Box::setAxes(const pegas::Vector3 & a, const pegas::Vector3 & b, const pegas::Vector3 & c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegas::Box::getAxes(pegas::Vector3 & a, pegas::Vector3 & b, pegas::Vector3 & c) const
{
    a = mA;
    b = mB;
    c = mC;
}
