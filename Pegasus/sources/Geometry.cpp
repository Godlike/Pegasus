/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/Geometry.hpp"

pegasus::geometry::Shape::Shape(glm::dvec3 const& centerOfMass)
    : centerOfMass(centerOfMass)
{
}

pegasus::geometry::SimpleShape::SimpleShape(glm::dvec3 const& centerOfMass, SimpleShape::Type type)
    : Shape(centerOfMass)
    , type(type)
{
}

pegasus::geometry::Ray::Ray()
    : SimpleShape(SimpleShape::Type::RAY)
{
}

pegasus::geometry::Ray::Ray(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShape::Type::RAY)
    , direction(normal)
{
}

pegasus::geometry::Plane::Plane()
    : SimpleShape(SimpleShape::Type::PLANE)
{
}

pegasus::geometry::Plane::Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShape::Type::PLANE)
    , normal(normal)
{
}

pegasus::geometry::Triangle::Triangle()
    : SimpleShape(SimpleShape::Type::TRIANGLE)
{
}

pegasus::geometry::Triangle::Triangle(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c
)
    : SimpleShape(centerOfMass, SimpleShape::Type::TRIANGLE)
    , aVertex(a)
    , bVertex(b)
    , cVertex(c)
{
    CalculateNormal();
}

void pegasus::geometry::Triangle::CalculateNormal()
{
    normal = glm::cross(bVertex - aVertex, cVertex - aVertex);
}

pegasus::geometry::Sphere::Sphere()
    : SimpleShape(SimpleShape::Type::SPHERE)
    , radius()
{
}

pegasus::geometry::Sphere::Sphere(glm::dvec3 const& centerOfMass, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::SPHERE)
    , radius(r)
{
}

pegasus::geometry::Cone::Cone()
    : SimpleShape(SimpleShape::Type::CONE)
    , radius()
{
}

pegasus::geometry::Cone::Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::CONE)
    , appex(a)
    , radius(r)
{
}

pegasus::geometry::Capsule::Capsule()
    : SimpleShape(SimpleShape::Type::CAPSULE)
    , radius()
{
}

pegasus::geometry::Capsule::Capsule(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : SimpleShape(centerOfMass, SimpleShape::Type::CAPSULE)
    , halfHeight(halfHeight)
    , radius(r)
{
}

pegasus::geometry::Cylinder::Cylinder()
    : SimpleShape(SimpleShape::Type::CYLINDER)
    , radius()
{
}

pegasus::geometry::Cylinder::Cylinder(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : SimpleShape(centerOfMass, SimpleShape::Type::CYLINDER)
    , halfHeight(halfHeight)
    , radius(r)
{
}

pegasus::geometry::Box::Box()
    : SimpleShape(SimpleShape::Type::BOX)
{
}

pegasus::geometry::Box::Box(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c
)
    : SimpleShape(centerOfMass, SimpleShape::Type::BOX)
    , iAxis(a)
    , jAxis(b)
    , kAxis(c)
{
}

pegasus::geometry::SimpleShapeIntersectionDetector::SimpleShapeIntersectionDetector()
    : m_intersectionCaches(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_initializeFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculateIntersectionFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculateContactNormalFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculatePenetrationFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
{
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::IntersectionCache<Plane, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::IntersectionCache<Plane, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::IntersectionCache<Plane, Box>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::IntersectionCache<Sphere, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::IntersectionCache<Sphere, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::IntersectionCache<Sphere, Box>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::IntersectionCache<Box, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::IntersectionCache<Box, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::IntersectionCache<Box, Box>>();

    m_initializeFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = intersection::Initialize<Plane, Plane>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = intersection::Initialize<Plane, Sphere>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = intersection::Initialize<Plane, Box>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = intersection::Initialize<Sphere, Plane>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = intersection::Initialize<Sphere, Sphere>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = intersection::Initialize<Sphere, Box>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = intersection::Initialize<Box, Plane>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = intersection::Initialize<Box, Sphere>;
    m_initializeFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = intersection::Initialize<Box, Box>;

    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = intersection::CalculateIntersection<Plane, Plane>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = intersection::CalculateIntersection<Plane, Sphere>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = intersection::CalculateIntersection<Plane, Box>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = intersection::CalculateIntersection<Sphere, Plane>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = intersection::CalculateIntersection<Sphere, Sphere>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = intersection::CalculateIntersection<Sphere, Box>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = intersection::CalculateIntersection<Box, Plane>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = intersection::CalculateIntersection<Box, Sphere>;
    m_calculateIntersectionFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = intersection::CalculateIntersection<Box, Box>;

    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = intersection::CalculateContactNormal<Plane, Plane>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = intersection::CalculateContactNormal<Plane, Sphere>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = intersection::CalculateContactNormal<Plane, Box>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = intersection::CalculateContactNormal<Sphere, Plane>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = intersection::CalculateContactNormal<Sphere, Sphere>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = intersection::CalculateContactNormal<Sphere, Box>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = intersection::CalculateContactNormal<Box, Plane>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = intersection::CalculateContactNormal<Box, Sphere>;
    m_calculateContactNormalFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = intersection::CalculateContactNormal<Box, Box>;

    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = intersection::CalculatePenetration<Plane, Plane>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = intersection::CalculatePenetration<Plane, Sphere>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = intersection::CalculatePenetration<Plane, Box>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = intersection::CalculatePenetration<Sphere, Plane>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = intersection::CalculatePenetration<Sphere, Sphere>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = intersection::CalculatePenetration<Sphere, Box>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = intersection::CalculatePenetration<Box, Plane>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = intersection::CalculatePenetration<Box, Sphere>;
    m_calculatePenetrationFunctors[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = intersection::CalculatePenetration<Box, Box>;
}

void pegasus::geometry::SimpleShapeIntersectionDetector::Initialize(SimpleShape const* a, SimpleShape const* b)
{
    m_initializeFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

bool pegasus::geometry::SimpleShapeIntersectionDetector::CalculateIntersection(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculateIntersectionFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

glm::dvec3 pegasus::geometry::SimpleShapeIntersectionDetector::CalculateContactNormal(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculateContactNormalFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

double pegasus::geometry::SimpleShapeIntersectionDetector::CalculatePenetration(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculatePenetrationFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

size_t pegasus::geometry::SimpleShapeIntersectionDetector::ShapeTypePairHasher::operator()(ShapeTypePair const& p) const
{
    return std::hash<uint32_t>()(static_cast<uint32_t>(p.first))
        ^ std::hash<uint32_t>()(static_cast<uint32_t>(p.second));
}
