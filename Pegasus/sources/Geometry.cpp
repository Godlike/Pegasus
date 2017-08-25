/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Pegasus/include/Geometry.hpp>

#include <algorithm>
#include <numeric>

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

bool intersection::CheckRaySphereIntersection(
    glm::dvec3 const& raySphere, double sphereRadius, glm::dvec3 const& rayDirection
)
{
    double const tCenter = glm::dot(raySphere, rayDirection);
    double const distanceSquare = glm::dot(raySphere, raySphere) - tCenter * tCenter;

    return sphereRadius * sphereRadius - distanceSquare >= 0.0;
}

intersection::RayIntersectionFactors
intersection::CalculateRaySphereIntersectionFactors(
    glm::dvec3 const& raySphere, double sphereRadius, glm::dvec3 const& rayDirection
)
{
    double const tCenter = glm::dot(raySphere, rayDirection);
    double const distanceSquare = glm::dot(raySphere, raySphere) - tCenter * tCenter;
    double const tDelta = glm::sqrt(sphereRadius * sphereRadius - distanceSquare);

    return {tCenter - tDelta, tCenter + tDelta};
}

intersection::RayIntersectionFactors
intersection::CalculateRayAabbIntersectionFactors(
    glm::dvec3 const& boxMinPoint, glm::dvec3 const& boxMaxPoint,
    glm::dvec3 const& rayDirection, glm::dvec3 const& rayOrigin
)
{
    double const t1 = (boxMinPoint.x - rayOrigin.x) / rayDirection.x;
    double const t2 = (boxMaxPoint.x - rayOrigin.x) / rayDirection.x;
    double const t3 = (boxMinPoint.y - rayOrigin.y) / rayDirection.y;
    double const t4 = (boxMaxPoint.y - rayOrigin.y) / rayDirection.y;
    double const t5 = (boxMinPoint.z - rayOrigin.z) / rayDirection.z;
    double const t6 = (boxMaxPoint.z - rayOrigin.z) / rayDirection.z;

    double const tmin = glm::max(glm::max(glm::min(t1, t2), glm::min(t3, t4)), glm::min(t5, t6));
    double const tmax = glm::min(glm::min(glm::max(t1, t2), glm::max(t3, t4)), glm::max(t5, t6));

    return {tmin, tmax};
}

intersection::AabbExtremalVertices
intersection::MakeExtremalVerticesAabb(
    glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k
)
{
    glm::dmat3 const boxModelMatrixInverse = glm::inverse(
        glm::dmat3{glm::normalize(i), glm::normalize(j), glm::normalize(k)}
    );

    glm::dmat3 const boxAxesModelSpace{
        boxModelMatrixInverse * i, boxModelMatrixInverse * j, boxModelMatrixInverse * k
    };

    auto const findMaxAbs = [](double a, double b)
    {
        return std::abs(a) < std::abs(b);
    };

    glm::dvec3 const maxVertex = glm::dvec3{
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[0]), glm::value_ptr(boxAxesModelSpace[0]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[1]), glm::value_ptr(boxAxesModelSpace[1]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[2]), glm::value_ptr(boxAxesModelSpace[2]) + 3, findMaxAbs)
        )
    };

    return {-maxVertex, maxVertex};
}

bool intersection::CheckRayIntersectionFactors(RayIntersectionFactors factors)
{
    // tMax < 0, intersection is behind ray; tMin > tMax, no intesection
    return factors.tMax > 0 && factors.tMin < factors.tMax;
}

bool intersection::IsPointInsideTriangle(
    glm::dvec3 const& triangleVertex1, glm::dvec3 const& triangleVertex2, glm::dvec3 const& triangleVertex3, glm::dvec3 const& point
)
{
    double const distance = math::HyperPlane{triangleVertex1, triangleVertex2, triangleVertex3}.Distance(point);
    if (!math::fp::IsZero(distance))
    {
        return false;
    }

    return IsSameSide(triangleVertex1, triangleVertex2, triangleVertex3, point)
        && IsSameSide(triangleVertex1, triangleVertex3, triangleVertex2, point)
        && IsSameSide(triangleVertex2, triangleVertex3, triangleVertex1, point);
}

glm::dvec3 intersection::cso::Support(Sphere const& sphere, glm::dvec3 direction)
{
    glm::dvec3 const vertex = sphere.centerOfMass + direction * sphere.radius;
    return vertex;
}

glm::dvec3 intersection::cso::Support(Box const& box, glm::dvec3 direction)
{
    using namespace intersection;

    std::array<glm::dvec3, 8> boxVertices;
    math::CalculateBoxVertices(box.iAxis, box.jAxis, box.kAxis, box.centerOfMass, boxVertices.begin());
    glm::dvec3 const maxPoint = *std::max_element(boxVertices.begin(), boxVertices.end(),
        [&direction](glm::dvec3 const& a, glm::dvec3 const& b) -> bool
    {
        return glm::dot(a, direction) < glm::dot(b, direction);
    });

    return maxPoint;
}

namespace
{
/**
 *  @brief  Checks if line passes through the origin
 *
 *  @param  lineStart   line segment start point
 *  @param  lineEnd     line segment end point
 *
 *  @return @c true if line passes though the origin, @c false otherwise
 */
bool LineSegmentContainsOrigin(glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd)
{
    double const distance = math::LineSegmentPointDistance(lineStart, lineEnd, glm::dvec3{});
    return math::fp::IsZero(distance);
}

/**
 *  @brief  Checks if triangle contains the origin
 *
 *  @attention  All input points must not lie on the same line at the same time
 *
 *  @param  a   triangle vertex
 *  @param  b   triangle vertex
 *  @param  c   triangle vertex
 *
 *  @return @c true if triangle contains the origin, @c false othewise
 */
bool TriangleContainsOrigin(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
{
    return intersection::IsPointInsideTriangle(a, b, c, glm::dvec3{});
}

/**
 *  @brief  Checks if tetrahedron contains the origin
 *
 *  @param  vertices    tetrahedron vertices
 *
 *  @return @c true if tetrahedron contains the origin, @c false otherwise
 */
bool TetrahedronContainsOrigin(std::array<glm::dvec3, 4> const& vertices)
{
    std::array<math::HyperPlane, 4> const faces{
        math::HyperPlane{vertices[0], vertices[1], vertices[2], &vertices[3]},
        math::HyperPlane{vertices[1], vertices[2], vertices[3], &vertices[0]},
        math::HyperPlane{vertices[0], vertices[2], vertices[3], &vertices[1]},
        math::HyperPlane{vertices[0], vertices[1], vertices[3], &vertices[2]}
    };

    return math::fp::IsGreaterOrEqual(faces[0].GetDistance(), 0.0)
        && math::fp::IsGreaterOrEqual(faces[1].GetDistance(), 0.0)
        && math::fp::IsGreaterOrEqual(faces[2].GetDistance(), 0.0)
        && math::fp::IsGreaterOrEqual(faces[3].GetDistance(), 0.0);
}

/**
 *  @brief  Finds nearest simplex from the line segment simplex to the origin
 *
 *  Given simplex may be reduced down to size 1 as a result of this method.
 *
 *  @param[in,out]  simplex line segment simplex
 *
 *  @return new search direction
 */
glm::dvec3 NearestSimplexLineSegment(intersection::gjk::Simplex& simplex)
{
    glm::dvec3 const AB = simplex.vertices[0] - simplex.vertices[1];
    glm::dvec3 const A0 = glm::dvec3{0, 0, 0} - simplex.vertices[1];

    if (intersection::IsAngleAcute(AB, A0))
    {
        glm::dvec3 const direction = glm::cross(glm::cross(AB, A0), AB);
        simplex.size = 2;
        return direction;
    }

    simplex.vertices[0] = simplex.vertices[1];
    simplex.size = 1;
    return A0;
}

/**
 *  @brief  Finds nearest simplex from given triangle simplex to the origin
 *
 *  Given simplex may be reduced down to size 1 as a result of this method.
 *
 *  @param[in,out]  simplex triange simplex
 *
 *  @return new search direction
 */
glm::dvec3 NearestSimplexTriangle(intersection::gjk::Simplex& simplex)
{
    glm::dvec3 const& A = simplex.vertices[2];
    glm::dvec3 const& B = simplex.vertices[1];
    glm::dvec3 const& C = simplex.vertices[0];

    glm::dvec3 const AB = B - A;
    glm::dvec3 const AC = C - A;

    glm::dvec3 const A0 = glm::dvec3{0, 0, 0} - A;
    glm::dvec3 const ABC = glm::cross(AB, AC);

    glm::dvec3 result;

    if (intersection::IsAngleAcute(glm::cross(ABC, AC), -A))
    {
        if (intersection::IsAngleAcute(AC, A0))
        {
            simplex.vertices = {C, A};
            simplex.size = 2;

            result = glm::cross(glm::cross(AC, -B), AC);
        }
        else if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.vertices = {B, A};
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.vertices = {A};
            simplex.size = 1;

            result = -C;
        }
    }
    else if (intersection::IsAngleAcute(glm::cross(AB, ABC), -A))
    {
        if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.vertices = {B, A};
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.vertices = {A};
            simplex.size = 1;

            result = -C;
        }
    }
    else
    {
        if (intersection::IsAngleAcute(ABC, A0))
        {
            result = ABC;
        }
        else
        {
            result = -ABC;
        }
    }

    return result;
}

/**
 *  @brief  Finds nearest simplex from the tetrahedron simplex to the origin
 *
 *  Given simplex may be reduced down to size 1 as a result of this method.
 *
 *  @param[in,out]  simplex tetrahedron simplex
 *
 *  @return new search direction
 *
 *  @sa NearestSimplexTriangle
 */
glm::dvec3 NearestSimplexTetrahedron(intersection::gjk::Simplex& simplex)
{
    std::array<std::array<uint8_t, 3>, 3> const simplices{
        std::array<uint8_t, 3>{0, 1, 3},
        std::array<uint8_t, 3>{1, 2, 3},
        std::array<uint8_t, 3>{0, 2, 3}
    };

    std::array<glm::dvec3, 4> const& vertices = simplex.vertices;

    std::array<double, 3> const planeOriginDistances{
        math::HyperPlane{vertices[0], vertices[1], vertices[3], &vertices[2]}.GetDistance(),
        math::HyperPlane{vertices[1], vertices[2], vertices[3], &vertices[0]}.GetDistance(),
        math::HyperPlane{vertices[0], vertices[2], vertices[3], &vertices[1]}.GetDistance()
    };

    size_t const closestPlaneIndex = std::distance(planeOriginDistances.begin(),
        std::min_element(planeOriginDistances.begin(), planeOriginDistances.end()));

    simplex.vertices = {
        vertices[simplices[closestPlaneIndex][0]],
        vertices[simplices[closestPlaneIndex][1]],
        vertices[simplices[closestPlaneIndex][2]]
    };
    simplex.size = 3;

    return NearestSimplexTriangle(simplex);
}
} // anonymous namespace

bool intersection::gjk::SimplexContainsOrigin(Simplex const& simplex)
{
    if (simplex.size == 2)
    {
        return ::LineSegmentContainsOrigin(simplex.vertices[0], simplex.vertices[1]);
    }
    
    if (simplex.size == 3)
    {
        return ::TriangleContainsOrigin(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]);
    }

    return ::TetrahedronContainsOrigin(simplex.vertices);
}

glm::dvec3 intersection::gjk::NearestSimplex(Simplex& simplex)
{
    if (2 == simplex.size)
    {
        return ::NearestSimplexLineSegment(simplex);
    }
    
    if (3 == simplex.size)
    {
        return ::NearestSimplexTriangle(simplex);
    }

    return ::NearestSimplexTetrahedron(simplex);
}

bool intersection::gjk::DoSimplex(gjk::Simplex& simplex, glm::dvec3& direction)
{
    //Check if a current simplex contains the origin
    if (SimplexContainsOrigin(simplex))
    {
        return true;
    }

    //Calculate sub simplex nearest to the origin
    direction = glm::normalize(NearestSimplex(simplex));

    return false;
}

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
