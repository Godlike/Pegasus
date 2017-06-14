/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/Geometry.hpp"

pegasus::geometry::Shape::Shape(glm::dvec3 const& centerOfMass)
    : m_centerOfMass(centerOfMass)
{
}

void pegasus::geometry::Shape::setCenterOfMass(glm::dvec3 const& centerOfMass)
{
    m_centerOfMass = centerOfMass;
}

glm::dvec3 const& pegasus::geometry::Shape::getCenterOfMass() const
{
    return m_centerOfMass;
}

pegasus::geometry::SimpleShape::SimpleShape(glm::dvec3 const& centerOfMass, SimpleShapeType type)
    : Shape(centerOfMass)
    , type(type)
{
}

pegasus::geometry::Plane::Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShapeType::PLANE)
    , m_normal(normal)
{
}

void pegasus::geometry::Plane::SetNormal(glm::dvec3 const& normal)
{
    m_normal = normal;
}

glm::dvec3 const& pegasus::geometry::Plane::GetNormal() const
{
    return m_normal;
}

pegasus::geometry::Triangle::Triangle(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::TRIANGLE)
    , m_aVertex(a)
    , m_bVertex(b)
    , m_cVertex(c)
{
    CalculateNormal();
}

void pegasus::geometry::Triangle::SetAxes(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
{
    m_aVertex = a;
    m_bVertex = b;
    m_cVertex = c;
    CalculateNormal();
}

void pegasus::geometry::Triangle::GetAxes(glm::dvec3& a, glm::dvec3& b, glm::dvec3& c) const
{
    a = m_aVertex;
    b = m_bVertex;
    c = m_cVertex;
}

glm::dvec3 const& pegasus::geometry::Triangle::GetNormal() const
{
    return m_normal;
}

void pegasus::geometry::Triangle::CalculateNormal()
{
    m_normal = glm::cross(m_bVertex - m_aVertex, m_cVertex - m_aVertex);
}

pegasus::geometry::Sphere::Sphere(glm::dvec3 const& centerOfMass, double r)
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

pegasus::geometry::Cone::Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r)
    : SimpleShape(centerOfMass, SimpleShapeType::CONE)
    , m_appex(a)
    , m_radius(r)
{
}

void pegasus::geometry::Cone::SetAppex(glm::dvec3 const& a)
{
    m_appex = a;
}

glm::dvec3 const& pegasus::geometry::Cone::GetAppex() const
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

pegasus::geometry::Capsule::Capsule(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r)
    : SimpleShape(centerOfMass, SimpleShapeType::CAPSULE)
    , m_halfHeight(halfHeight)
    , m_radius(r)
{
}

void pegasus::geometry::Capsule::SetHalfHeight(glm::dvec3 const& halfHeight)
{
    m_halfHeight = halfHeight;
}

glm::dvec3 const& pegasus::geometry::Capsule::GetHalfHeight() const
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

pegasus::geometry::Cylinder::Cylinder(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r)
    : Capsule(centerOfMass, halfHeight, r)
{
}

pegasus::geometry::Box::Box(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
    : SimpleShape(centerOfMass, SimpleShapeType::BOX)
    , m_aAxis(a)
    , m_bAxis(b)
    , m_cAxis(c)
{
}

void pegasus::geometry::Box::SetAxes(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
{
    m_aAxis = a;
    m_bAxis = b;
    m_cAxis = c;
}

void pegasus::geometry::Box::GetAxes(glm::dvec3& a, glm::dvec3& b, glm::dvec3& c) const
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
    glm::dvec3 const& p1, glm::dvec3 const& p2, glm::dvec3 const& a, glm::dvec3 const& b
)
{
    glm::dvec3 const ab = b - a;
    glm::dvec3 const cp1 = glm::cross(ab, p1 - a);
    glm::dvec3 const cp2 = glm::cross(ab, p2 - a);
    return glm::dot(cp1, cp2) >= 0;
}
