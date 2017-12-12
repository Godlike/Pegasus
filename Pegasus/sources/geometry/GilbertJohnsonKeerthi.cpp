/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <geometry/GilbertJohnsonKeerthi.hpp>
#include <geometry/Intersection.hpp>
#include <Epona/Analysis.hpp>

using namespace pegasus;
using namespace geometry;

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
    double const distance = epona::LineSegmentPointDistance(lineStart, lineEnd, glm::dvec3{});
    return epona::fp::IsZero(distance);
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
*  @return @c true if triangle contains the origin, @c false otherwise
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
    std::array<epona::HyperPlane, 4> const faces{ {
            epona::HyperPlane{ vertices[0], vertices[1], vertices[2], &vertices[3] },
            epona::HyperPlane{ vertices[1], vertices[2], vertices[3], &vertices[0] },
            epona::HyperPlane{ vertices[0], vertices[2], vertices[3], &vertices[1] },
            epona::HyperPlane{ vertices[0], vertices[1], vertices[3], &vertices[2] }
        } };

    return epona::fp::IsGreaterOrEqual(faces[0].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[1].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[2].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[3].GetDistance(), 0.0);
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
    glm::dvec3 const A0 = glm::dvec3{ 0, 0, 0 } -simplex.vertices[1];

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
*  @param[in,out]  simplex triangle simplex
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

    glm::dvec3 const A0 = glm::dvec3{ 0, 0, 0 } -A;
    glm::dvec3 const ABC = glm::cross(AB, AC);

    glm::dvec3 result;

    if (intersection::IsAngleAcute(glm::cross(ABC, AC), -A))
    {
        if (intersection::IsAngleAcute(AC, A0))
        {
            simplex.vertices = { { C, A } };
            simplex.size = 2;

            result = glm::cross(glm::cross(AC, -B), AC);
        }
        else if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.vertices = { { B, A } };
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.vertices = { { A } };
            simplex.size = 1;

            result = -C;
        }
    }
    else if (intersection::IsAngleAcute(glm::cross(AB, ABC), -A))
    {
        if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.vertices = { { B, A } };
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.vertices = { { A } };
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
    std::array<std::array<uint8_t, 3>, 3> const simplices{ {
            std::array<uint8_t, 3>{ {0, 1, 3}},
            std::array<uint8_t, 3>{ {1, 2, 3}},
            std::array<uint8_t, 3>{ {0, 2, 3}}
        } };

    std::array<glm::dvec3, 4> const& vertices = simplex.vertices;

    std::array<double, 3> const planeOriginDistances{ {
            epona::HyperPlane{ vertices[0], vertices[1], vertices[3], &vertices[2] }.GetDistance(),
            epona::HyperPlane{ vertices[1], vertices[2], vertices[3], &vertices[0] }.GetDistance(),
            epona::HyperPlane{ vertices[0], vertices[2], vertices[3], &vertices[1] }.GetDistance()
        } };

    size_t const closestPlaneIndex = std::distance(planeOriginDistances.begin(),
        std::min_element(planeOriginDistances.begin(), planeOriginDistances.end()));

    simplex.vertices = { {
            vertices[simplices[closestPlaneIndex][0]],
            vertices[simplices[closestPlaneIndex][1]],
            vertices[simplices[closestPlaneIndex][2]]
        } };
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
