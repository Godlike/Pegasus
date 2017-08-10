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

void pegasus::geometry::Shape::SetCenterOfMass(glm::dvec3 const& centerOfMass)
{
    m_centerOfMass = centerOfMass;
}

glm::dvec3 const& pegasus::geometry::Shape::GetCenterOfMass() const
{
    return m_centerOfMass;
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
    , m_normal(normal)
{
}

void pegasus::geometry::Ray::SetNormal(glm::dvec3 const& normal)
{
    m_normal = normal;
}

glm::dvec3 const& pegasus::geometry::Ray::GetNormal() const
{
    return m_normal;
}

pegasus::geometry::Plane::Plane()
    : SimpleShape(SimpleShape::Type::PLANE)
{
}

pegasus::geometry::Plane::Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal)
    : SimpleShape(centerOfMass, SimpleShape::Type::PLANE)
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

pegasus::geometry::Triangle::Triangle()
    : SimpleShape(SimpleShape::Type::TRIANGLE)
{
}

pegasus::geometry::Triangle::Triangle(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c
)
    : SimpleShape(centerOfMass, SimpleShape::Type::TRIANGLE)
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

pegasus::geometry::Sphere::Sphere()
    : SimpleShape(SimpleShape::Type::SPHERE)
    , m_radius()
{
}

pegasus::geometry::Sphere::Sphere(glm::dvec3 const& centerOfMass, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::SPHERE)
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

pegasus::geometry::Cone::Cone()
    : SimpleShape(SimpleShape::Type::CONE)
    , m_radius()
{
}

pegasus::geometry::Cone::Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r)
    : SimpleShape(centerOfMass, SimpleShape::Type::CONE)
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

pegasus::geometry::Capsule::Capsule()
    : SimpleShape(SimpleShape::Type::CAPSULE)
    , m_radius()
{
}

pegasus::geometry::Capsule::Capsule(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : SimpleShape(centerOfMass, SimpleShape::Type::CAPSULE)
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

pegasus::geometry::Cylinder::Cylinder()
{
    type = SimpleShape::Type::CYLINDER;
}

pegasus::geometry::Cylinder::Cylinder(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r
)
    : Capsule(centerOfMass, halfHeight, r)
{
    type = SimpleShape::Type::CAPSULE;
}

pegasus::geometry::Box::Box()
    : SimpleShape(SimpleShape::Type::BOX)
{
}

pegasus::geometry::Box::Box(
    glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c
)
    : SimpleShape(centerOfMass, SimpleShape::Type::BOX)
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
