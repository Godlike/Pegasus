/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <geometry/SimpleShapeIntersectionDetector.hpp>

using namespace pegasus;
using namespace geometry;

SimpleShapeIntersectionDetector::SimpleShapeIntersectionDetector()
    : m_intersectionCaches(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculateIntersectionFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculateContactNormalFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
    , m_calculatePenetrationFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHasher())
{
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::Cache<Plane, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::Cache<Plane, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::PLANE, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::Cache<Plane, Box>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::Cache<Sphere, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::Cache<Sphere, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::SPHERE, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::Cache<Sphere, Box>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::PLANE)]
        = std::make_unique<intersection::Cache<Box, Plane>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::SPHERE)]
        = std::make_unique<intersection::Cache<Box, Sphere>>();
    m_intersectionCaches[std::make_pair(SimpleShape::Type::BOX, SimpleShape::Type::BOX)]
        = std::make_unique<intersection::Cache<Box, Box>>();

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

bool SimpleShapeIntersectionDetector::CalculateIntersection(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculateIntersectionFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

glm::dvec3 SimpleShapeIntersectionDetector::CalculateContactNormal(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculateContactNormalFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

double SimpleShapeIntersectionDetector::CalculatePenetration(SimpleShape const* a, SimpleShape const* b)
{
    return m_calculatePenetrationFunctors[std::make_pair(a->type, b->type)](
        a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
}

size_t SimpleShapeIntersectionDetector::ShapeTypePairHasher::operator()(ShapeTypePair const& p) const
{
    return std::hash<uint32_t>{}(static_cast<uint32_t>(p.first))
        ^ std::hash<uint32_t>{}(static_cast<uint32_t>(p.second));
}
