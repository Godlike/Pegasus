#include "Pegasus/include/geometry.hpp"

pegas::gmt::Geometry::~Geometry() {}

pegas::gmt::Shape::Shape(Vector3 const& centerOfMass)
    : mCenterOfMass(centerOfMass)
{
}

void pegas::gmt::Shape::setCenterOfMass(Vector3 const& centerOfMass)
{
    mCenterOfMass = centerOfMass;
}

pegas::Vector3 pegas::gmt::Shape::getCenterOfMass() const
{
    return mCenterOfMass;
}

pegas::gmt::SimpleShape::SimpleShape(Vector3 const& centerOfMass)
    : Shape(centerOfMass)
{
}

pegas::gmt::Plane::Plane(Vector3 const& centerOfMass, Vector3 const& normal)
    : SimpleShape(centerOfMass)
    , mNormal(normal)
{
}

void pegas::gmt::Plane::setNormal(Vector3 const& normal)
{
    mNormal = normal;
}

pegas::Vector3 pegas::gmt::Plane::getNormal() const
{
    return mNormal;
}

pegas::gmt::Triangle::Triangle(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mB(b)
    , mC(c)
{
    calculateNormal();
}

void pegas::gmt::Triangle::setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    mA = a;
    mB = b;
    mC = c;
    calculateNormal();
}

void pegas::gmt::Triangle::getAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}

pegas::Vector3 pegas::gmt::Triangle::getNormal() const
{
    return mNormal;
}

void pegas::gmt::Triangle::calculateNormal()
{
    mNormal = (mB - mA) % (mC - mA);
}

pegas::gmt::Sphere::Sphere(Vector3 const& centerOfMass, real const r)
    : SimpleShape(centerOfMass)
    , mR(r)
{
}

void pegas::gmt::Sphere::setRadius(real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Sphere::getRadius() const
{
    return mR;
}

pegas::gmt::Cone::Cone(Vector3 const& centerOfMass, Vector3 const& a, real const r)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mR(r)
{
}

void pegas::gmt::Cone::setAppex(Vector3 const& a)
{
    mA = a;
}

pegas::Vector3 pegas::gmt::Cone::getAppex() const
{
    return mA;
}

void pegas::gmt::Cone::setRadius(real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Cone::getRadius() const
{
    return mR;
}

pegas::gmt::Capsule::Capsule(Vector3 const& centerOfMass, Vector3 const& halfHeight, real const r)
    : SimpleShape(centerOfMass)
    , mHalfHeight(halfHeight)
    , mR(r)
{
}

void pegas::gmt::Capsule::setHalfHeight(Vector3 const& halfHeight)
{
    mHalfHeight = halfHeight;
}

pegas::Vector3 pegas::gmt::Capsule::getHalfHeight() const
{
    return mHalfHeight;
}

void pegas::gmt::Capsule::setRadius(real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Capsule::getRadius() const
{
    return mR;
}

pegas::gmt::Box::Box(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mB(b)
    , mC(c)
{
}

void pegas::gmt::Box::setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegas::gmt::Box::getAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}
