/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <geometry/Intersection.hpp>
#include <math/HyperPlane.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>

using namespace pegasus;
using namespace geometry;

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
