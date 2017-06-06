/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/Geometry.hpp"

pegasus::geometry::Shape::Shape(Vector3 const& centerOfMass)
    : m_centerOfMass(centerOfMass)
{
}

void pegasus::geometry::Shape::setCenterOfMass(Vector3 const& centerOfMass)
{
    m_centerOfMass = centerOfMass;
}

pegasus::Vector3 const& pegasus::geometry::Shape::getCenterOfMass() const
{
    return m_centerOfMass;
}

pegasus::geometry::SimpleShape::SimpleShape(Vector3 const& centerOfMass, SimpleShapeType type)
    : Shape(centerOfMass)
    , type(type)
{
}

pegasus::geometry::Plane::Plane(Vector3 const& centerOfMass, Vector3 const& normal)
    : SimpleShape(centerOfMass, SimpleShapeType::PLANE)
    , m_normal(normal)
{
}

void pegasus::geometry::Plane::SetNormal(Vector3 const& normal)
{
    m_normal = normal;
}

pegasus::Vector3 const& pegasus::geometry::Plane::GetNormal() const
{
    return m_normal;
}

pegasus::geometry::Triangle::Triangle(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::TRIANGLE)
    , m_aVertex(a)
    , m_bVertex(b)
    , m_cVertex(c)
{
    CalculateNormal();
}

void pegasus::geometry::Triangle::SetAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    m_aVertex = a;
    m_bVertex = b;
    m_cVertex = c;
    CalculateNormal();
}

void pegasus::geometry::Triangle::GetAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = m_aVertex;
    b = m_bVertex;
    c = m_cVertex;
}

pegasus::Vector3 const& pegasus::geometry::Triangle::GetNormal() const
{
    return m_normal;
}

void pegasus::geometry::Triangle::CalculateNormal()
{
    m_normal = (m_bVertex - m_aVertex) % (m_cVertex - m_aVertex);
}

pegasus::geometry::Sphere::Sphere(Vector3 const& centerOfMass, double r)
    : SimpleShape(centerOfMass, SimpleShapeType::SPHERE)
    , m_radius(r)
{
}

void pegasus::geometry::Sphere::SetRadius(double r)
{
    m_radius = r;
}

double pegasus::geometry::Sphere::GetRadius() const
{
    return m_radius;
}

pegasus::geometry::Cone::Cone(Vector3 const& centerOfMass, Vector3 const& a, double r)
    : SimpleShape(centerOfMass, SimpleShapeType::CONE)
    , m_appex(a)
    , m_radius(r)
{
}

void pegasus::geometry::Cone::SetAppex(Vector3 const& a)
{
    m_appex = a;
}

pegasus::Vector3 const& pegasus::geometry::Cone::GetAppex() const
{
    return m_appex;
}

void pegasus::geometry::Cone::SetRadius(double r)
{
    m_radius = r;
}

double pegasus::geometry::Cone::GetRadius() const
{
    return m_radius;
}

pegasus::geometry::Capsule::Capsule(Vector3 const& centerOfMass, Vector3 const& halfHeight, double r)
    : SimpleShape(centerOfMass, SimpleShapeType::CAPSULE)
    , m_halfHeight(halfHeight)
    , m_radius(r)
{
}

void pegasus::geometry::Capsule::SetHalfHeight(Vector3 const& halfHeight)
{
    m_halfHeight = halfHeight;
}

pegasus::Vector3 const& pegasus::geometry::Capsule::GetHalfHeight() const
{
    return m_halfHeight;
}

void pegasus::geometry::Capsule::SetRadius(double r)
{
    m_radius = r;
}

double pegasus::geometry::Capsule::GetRadius() const
{
    return m_radius;
}

pegasus::geometry::Cylinder::Cylinder(Vector3 const& centerOfMass, Vector3 const& halfHeight, double r)
    : Capsule(centerOfMass, halfHeight, r)
{
}

pegasus::geometry::Box::Box(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::BOX)
    , m_aAxis(a)
    , m_bAxis(b)
    , m_cAxis(c)
{
}

void pegasus::geometry::Box::SetAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c)
{
    m_aAxis = a;
    m_bAxis = b;
    m_cAxis = c;
}

void pegasus::geometry::Box::GetAxes(Vector3& a, Vector3& b, Vector3& c) const
{
    a = m_aAxis;
    b = m_bAxis;
    c = m_cAxis;
}

size_t pegasus::geometry::ShapeTypePairHash::operator()(ShapeTypePair const& p) const
{
    return std::hash<uint32_t>()(static_cast<uint32_t>(p.first)) ^ std::hash<uint32_t>()(static_cast<uint32_t>(p.second));
}

bool pegasus::geometry::intersection::IsSameSidePoint(
    Vector3 const& p1, Vector3 const& p2, Vector3 const& a, Vector3 const& b
)
{
    auto const ab = b - a;
    auto const cp1 = ab.vectorProduct(p1 - a);
    auto const cp2 = ab.vectorProduct(p2 - a);
    return cp1.scalarProduct(cp2) >= 0;
}
