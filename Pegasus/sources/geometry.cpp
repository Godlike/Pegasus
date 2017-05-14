#include "Pegasus/include/geometry.hpp"

pegasus::geometry::Shape::Shape(Vector3 const& centerOfMass)
    : mCenterOfMass(centerOfMass)
{
}

void pegasus::geometry::Shape::setCenterOfMass(Vector3 const& centerOfMass)
{
    mCenterOfMass = centerOfMass;
}

pegasus::Vector3 pegasus::geometry::Shape::getCenterOfMass() const
{
    return mCenterOfMass;
}

pegasus::geometry::SimpleShape::SimpleShape(Vector3 const& centerOfMass, SimpleShapeType type)
    : Shape(centerOfMass)
    , type(type)
{
}

pegasus::geometry::Plane::Plane(Vector3 const& centerOfMass, Vector3 const& normal)
    : SimpleShape(centerOfMass, SimpleShapeType::PLANE)
    , mNormal(normal)
{
}

void pegasus::geometry::Plane::setNormal(Vector3 const& normal)
{
    mNormal = normal;
}

pegasus::Vector3 pegasus::geometry::Plane::getNormal() const
{
    return mNormal;
}

pegasus::geometry::Triangle::Triangle(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::TRIANGLE)
    , mA(a)
    , mB(b)
    , mC(c)
{
    calculateNormal();
}

void pegasus::geometry::Triangle::setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    mA = a;
    mB = b;
    mC = c;
    calculateNormal();
}

void pegasus::geometry::Triangle::getAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}

pegasus::Vector3 pegasus::geometry::Triangle::getNormal() const
{
    return mNormal;
}

void pegasus::geometry::Triangle::calculateNormal()
{
    mNormal = (mB - mA) % (mC - mA);
}

pegasus::geometry::Sphere::Sphere(Vector3 const& centerOfMass, real const r)
    : SimpleShape(centerOfMass, SimpleShapeType::SPHERE)
    , mR(r)
{
}

void pegasus::geometry::Sphere::setRadius(real const r)
{
    mR = r;
}

pegasus::real pegasus::geometry::Sphere::getRadius() const
{
    return mR;
}

pegasus::geometry::Cone::Cone(Vector3 const& centerOfMass, Vector3 const& a, real const r)
    : SimpleShape(centerOfMass, SimpleShapeType::CONE)
    , mA(a)
    , mR(r)
{
}

void pegasus::geometry::Cone::setAppex(Vector3 const& a)
{
    mA = a;
}

pegasus::Vector3 pegasus::geometry::Cone::getAppex() const
{
    return mA;
}

void pegasus::geometry::Cone::setRadius(real const r)
{
    mR = r;
}

pegasus::real pegasus::geometry::Cone::getRadius() const
{
    return mR;
}

pegasus::geometry::Capsule::Capsule(Vector3 const& centerOfMass, Vector3 const& halfHeight, real const r)
    : SimpleShape(centerOfMass, SimpleShapeType::CAPSULE)
    , mHalfHeight(halfHeight)
    , mR(r)
{
}

void pegasus::geometry::Capsule::setHalfHeight(Vector3 const& halfHeight)
{
    mHalfHeight = halfHeight;
}

pegasus::Vector3 pegasus::geometry::Capsule::getHalfHeight() const
{
    return mHalfHeight;
}

void pegasus::geometry::Capsule::setRadius(real const r)
{
    mR = r;
}

pegasus::real pegasus::geometry::Capsule::getRadius() const
{
    return mR;
}

pegasus::geometry::Cylinder::Cylinder(const Vector3& centerOfMass, const Vector3& halfHeight, const real r)
    : Capsule(centerOfMass, halfHeight, r)
{
}

pegasus::geometry::Box::Box(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::BOX)
    , mA(a)
    , mB(b)
    , mC(c)
{
}

void pegasus::geometry::Box::setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegasus::geometry::Box::getAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}

size_t pegasus::geometry::shapeTypePairHash(const ShapeTypePair &p)
{
    return std::hash<int>()(static_cast<int>(p.first)) ^ std::hash<int>()(static_cast<int>(p.second));
}