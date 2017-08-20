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

bool intersection::CalculateRaySphereIntersection(
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

bool intersection::CalculateRayAabbIntersection(double tMin, double tMax)
{
    // tMax < 0, AABB is behind ray; tMin > tMax, no intesection
    return tMax > 0 && tMin < tMax;
}

glm::dvec3 intersection::cso::Support(Sphere const& sphere, glm::dvec3 direction)
{
    using namespace intersection;

    Ray const ray{sphere.centerOfMass - direction * (sphere.radius + 1), direction};
    RayIntersectionFactors intersectionFactors = CalculateRaySphereIntersectionFactors(
        sphere.centerOfMass - ray.centerOfMass, sphere.radius, direction
    );

    glm::dvec3 const vertex = ray.centerOfMass + direction * intersectionFactors.tMax;
    return vertex;
}

glm::dvec3 intersection::cso::Support(Box const& box, glm::dvec3 direction)
{
    using namespace intersection;

    std::array<glm::dvec3, 8> boxVertices;
    math::CalculateBoxVertices(box.iAxis, box.jAxis, box.kAxis, box.centerOfMass, boxVertices.begin());
    std::sort(boxVertices.begin(), boxVertices.end(),
        [&direction](glm::dvec3 const& a, glm::dvec3 const& b) -> bool
    {
        return glm::dot(a, direction) < glm::dot(b, direction);
    });

    return boxVertices.back();
}

glm::dvec3 intersection::cso::Support(Box const& box1, Box const& box2, glm::dvec3 direction)
{
    glm::dvec3 const vertex = Support(box1, direction) - Support(box2, -direction);
    return vertex;
}

/**
 * @brief Calculates whether a line passes through the origin and returns true if it does
 * @param[in] lineStart line segment start point
 * @param[in] lineEnd line segment end point
 * @return @c true if a line passes though the origin @c false otherwise
 */
bool LineSegmentContainsOrigin(glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd)
{
    double const distance = math::LineSegmentPointDistance(lineStart, lineEnd, glm::dvec3{});
    return math::fp::IsNull(distance);
}

/**
 * @brief Calculates whether a triangle contains the origin and returns true if it does
 *
 * All input point must not lie on the same line at the same time, otherwise result is undefined
 * @param[in] a triangle vertex
 * @param[in] b triangle vertex
 * @param[in] c triangle vertex
 * @return @c true if a triangle contains the origin @c false othewise
 */
bool TriangleContainsOrigin(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c)
{
    return intersection::IsTriangleContainsPoint(a, b, c, glm::dvec3{});
}

/**
 * @brief Calculates whether a point is inside a tetrahedron
 * @param[in] vertices tetrahedron vertices
 * @return @c true if there is an intersection, @c false otherwise
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

bool intersection::gjk::SimplexContainsOrigin(Simplex const& simplex)
{
    if (simplex.size == 2)
    {
        return LineSegmentContainsOrigin(simplex.vertices[0], simplex.vertices[1]);
    }
    if (simplex.size == 3)
    {
        return TriangleContainsOrigin(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]);
    }

    return TetrahedronContainsOrigin(simplex.vertices);
}

/**
 * @brief Finds nearest simplex from the line sigment simplex to the origin and returns its size and new search direction
 * @param[in] simplex line segment simplex vertices
 * @return new simplex size and search direction
 */
intersection::gjk::NearestSimplexData NearestSimplexLineSegment(std::array<glm::dvec3, 4>& simplex)
{
    glm::dvec3 const AB = simplex[0] - simplex[1];
    glm::dvec3 const A0 = glm::dvec3{0, 0, 0} - simplex[1];

    if (intersection::IsSameDirection(AB, A0))
    {
        glm::dvec3 const direction = glm::cross(glm::cross(AB, A0), AB);
        return {2, direction};
    }

    simplex[0] = simplex[1];
    return {1, A0};
}

/**
 * @brief Finds nearest simplex from the triangle simplex to the origin and returns its size and new search direction
 * @param[in] simplex triangle simplex vertices
 * @return new simplex size and search direction
 */
intersection::gjk::NearestSimplexData NearestSimplexTriangle(std::array<glm::dvec3, 4>& simplex)
{
    glm::dvec3 const AB = simplex[1] - simplex[2];
    glm::dvec3 const AC = simplex[0] - simplex[2];
    glm::dvec3 const A0 = glm::dvec3{0, 0, 0} - simplex[2];
    glm::dvec3 const ABC = glm::cross(AB, AC);
    intersection::gjk::NearestSimplexData result;

    if (intersection::IsSameDirection(glm::cross(ABC, AC), -simplex[2]))
    {
        if (intersection::IsSameDirection(AC, A0))
        {
            simplex = {simplex[0], simplex[2]};
            glm::dvec3 const direction = glm::cross(glm::cross(AC, -simplex[1]), AC);
            result = {2, direction};
        }
        else if (intersection::IsSameDirection(AB, A0))
        {
            simplex = {simplex[1], simplex[2]};
            glm::dvec3 const direction = glm::cross(glm::cross(AB, -simplex[1]), AB);
            result = {2, direction};
        }
        else
        {
            simplex = {simplex[2]};
            glm::dvec3 const direction = -simplex[0];
            result = {1, direction};
        }
    }
    else if (intersection::IsSameDirection(glm::cross(AB, ABC), -simplex[2]))
    {
        if (intersection::IsSameDirection(AB, A0))
        {
            simplex = {simplex[1], simplex[2]};
            glm::dvec3 const direction = glm::cross(glm::cross(AB, -simplex[1]), AB);
            result = {2, direction};
        }
        else
        {
            simplex = {simplex[2]};
            glm::dvec3 const direction = -simplex[0];
            result = {1, direction};
        }
    }
    else
    {
        if (intersection::IsSameDirection(ABC, A0))
        {
            glm::dvec3 const direction = ABC;
            result = {3, direction};
        }
        else
        {
            glm::dvec3 const direction = -ABC;
            result = {3, direction};
        }
    }

    return result;
}

/**
 * @brief Finds nearest simplex from the tetrahedron simplex to the origin and returns its size and new search direction
 * @param[in] simplex tetrahedron simplex vertices
 * @return new simplex size and search direction
 */
intersection::gjk::NearestSimplexData NearestSimplexTetrahedron(std::array<glm::dvec3, 4>& simplex)
{
    std::array<std::array<uint8_t, 3>, 3> const simplexes{
        std::array<uint8_t, 3>{0, 1, 3},
        std::array<uint8_t, 3>{1, 2, 3},
        std::array<uint8_t, 3>{0, 2, 3}
    };

    std::array<double, 3> const planeOriginDistances{
        math::HyperPlane{simplex[0], simplex[1], simplex[3], &simplex[2]}.GetDistance(),
        math::HyperPlane{simplex[1], simplex[2], simplex[3], &simplex[0]}.GetDistance(),
        math::HyperPlane{simplex[0], simplex[2], simplex[3], &simplex[1]}.GetDistance()
    };

    size_t const closestPlaneIndex = std::distance(planeOriginDistances.begin(),
        std::min_element(planeOriginDistances.begin(), planeOriginDistances.end()));

    simplex = {
        simplex[simplexes[closestPlaneIndex][0]],
        simplex[simplexes[closestPlaneIndex][1]],
        simplex[simplexes[closestPlaneIndex][2]]
    };

    return NearestSimplexTriangle(simplex);
}

intersection::gjk::NearestSimplexData intersection::gjk::NearestSimplex(std::array<glm::dvec3, 4>& simplex, uint8_t simplexSize)
{
    if (2 == simplexSize)
    {
        return NearestSimplexLineSegment(simplex);
    }
    if (3 == simplexSize)
    {
        return NearestSimplexTriangle(simplex);
    }

    return NearestSimplexTetrahedron(simplex);
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
