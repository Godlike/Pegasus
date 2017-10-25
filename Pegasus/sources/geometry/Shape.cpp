/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <geometry/Shape.hpp>

using namespace pegasus;
using namespace geometry;

Shape::Shape(glm::dvec3 const& centerOfMass)
    : centerOfMass(centerOfMass)
{
}

SimpleShape::SimpleShape(glm::dvec3 const& centerOfMass, SimpleShape::Type type)
    : Shape(centerOfMass)
    , type(type)
{
}

Ray::Ray()
    : SimpleShape(SimpleShape::Type::RAY)
{
}

Ray::Ray(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShape::Type::RAY)
    , direction(normal)
{
}

Plane::Plane()
    : SimpleShape(SimpleShape::Type::PLANE)
{
}

Plane::Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShape::Type::PLANE)
    , normal(normal)
{
}

Triangle::Triangle()
    : SimpleShape(SimpleShape::Type::TRIANGLE)
{
}

Triangle::Triangle(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c
)
    : SimpleShape(centerOfMass, SimpleShape::Type::TRIANGLE)
    , aVertex(a)
    , bVertex(b)
    , cVertex(c)
{
    CalculateNormal();
}

void Triangle::CalculateNormal()
{
    normal = glm::cross(bVertex - aVertex, cVertex - aVertex);
}

Sphere::Sphere()
    : SimpleShape(SimpleShape::Type::SPHERE)
    , radius()
{
}

Sphere::Sphere(glm::dvec3 const& centerOfMass, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::SPHERE)
    , radius(r)
{
}

Cone::Cone()
    : SimpleShape(SimpleShape::Type::CONE)
    , radius()
{
}

Cone::Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::CONE)
    , apex(a)
    , radius(r)
{
}

Capsule::Capsule()
    : SimpleShape(SimpleShape::Type::CAPSULE)
    , radius()
{
}

Capsule::Capsule(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : SimpleShape(centerOfMass, SimpleShape::Type::CAPSULE)
    , halfHeight(halfHeight)
    , radius(r)
{
}

Cylinder::Cylinder()
    : SimpleShape(SimpleShape::Type::CYLINDER)
    , radius()
{
}

Cylinder::Cylinder(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : SimpleShape(centerOfMass, SimpleShape::Type::CYLINDER)
    , halfHeight(halfHeight)
    , radius(r)
{
}

Box::Box()
    : SimpleShape(SimpleShape::Type::BOX)
{
}

Box::Box(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k
)
    : SimpleShape(centerOfMass, SimpleShape::Type::BOX)
    , iAxis(i)
    , jAxis(j)
    , kAxis(k)
{
}
