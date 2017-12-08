/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_INTERSECTION_HPP
#define PEGASUS_INTERSECTION_HPP

#include <Epona/FloatingPoint.hpp>
#include <glm/glm.hpp>

namespace pegasus
{
namespace geometry
{
namespace intersection
{
/** Stores ray factors for Ray collisions */
struct RayIntersectionFactors
{
    //! Factor to closest to ray origin intersection point
    double tMin;

    //! Factor to furthest from ray origin intersection point
    double tMax;
};

/**
 *  @brief  Checks if Ray and Sphere are intersecting
 *
 *  @param  raySphere      vector from the ray origin to the sphere center
 *  @param  sphereRadius   radius of the sphere
 *  @param  rayDirection   normalized direction vector of the ray
 *
 *  @return @c true if there is intersection, @c false otherwise
 */
bool CheckRaySphereIntersection(
    glm::dvec3 const& raySphere, double sphereRadius, glm::dvec3 const& rayDirection
);

/**
 *  @brief  Calculates ray factors for the Ray-Sphere intersection points
 *
 *  @attention  Must be called only if given ray and sphere are intersecting
 *
 *  @param  raySphere       vector from the ray to the sphere center
 *  @param  sphereRadius    radius of the sphere
 *  @param  rayDirection    normalized direction vector of the ray
 *
 *  @return ray factors for intersection
 */
RayIntersectionFactors CalculateRaySphereIntersectionFactors(
    glm::dvec3 const& raySphere, double sphereRadius, glm::dvec3 const& rayDirection
);

/** Stores AABB using minimum and maximum points */
struct AabbExtremalVertices
{
    glm::dvec3 minVertex;
    glm::dvec3 maxVertex;
};

/**
 *  @brief Calculates ray intersection factors for AABB-Ray collision
 *
 *  @param  boxMinPoint     min point of AABB
 *  @param  boxMaxPoint     max point of AABB
 *  @param  rayDirection    normalized direction vector
 *  @param  rayOrigin       ray origin
 *
 *  @return ray factors for intersection
 */
RayIntersectionFactors CalculateRayAabbIntersectionFactors(
    glm::dvec3 const& boxMinPoint, glm::dvec3 const& boxMaxPoint,
    glm::dvec3 const& rayDirection, glm::dvec3 const& rayOrigin
);

/**
  * @brief  Calculates AABB min and max points from the given OBB basis
  *
  * @attention  given vectors must be different
  *
  * @param  i   vector from an orthogonal basis
  * @param  j   vector from an orthogonal basis
  * @param  k   vector from an orthogonal basis
  *
  * @return AABB min and max points
  */
AabbExtremalVertices MakeExtremalVerticesAabb(
    glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k
);

/**
 *  @brief  Checks if Ray intersection factors indicate a valid intersection
 *
 *  @param  factors ray factors for intersection
 *
 *  @return @c true if intersection factors are valid, @c false otherwise
 */
bool CheckRayIntersectionFactors(RayIntersectionFactors factors);

/**
 *  @brief  Checks if two vectors are at acute angle
 *
 *  @param  aVector input vector
 *  @param  bVector input vector
 *
 *  @return @c true if vectors are at acute angle, @c false otherwise
 */
inline bool IsAngleAcute(glm::dvec3 const& aVector, glm::dvec3 const& bVector)
{
    return epona::fp::IsGreater(glm::dot(aVector, bVector), 0);
}

/**
 *  @brief  Checks if two points are on the same side of the halfspace
 *
 *  Halfspace is defined by a line and a point
 *
 *  @param  lineStart   line start point
 *  @param  lineEnd     line end point
 *  @param  aPoint      point of interest
 *  @param  bPoint      point of interest
 *
 *  @return @c true if points are on the same side of the halfspace, @c false otherwise
 */
inline bool IsSameSide(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd,
    glm::dvec3 const& aPoint, glm::dvec3 const& bPoint
)
{
    glm::dvec3 const cp1 = glm::cross(lineEnd - lineStart, aPoint - lineStart);
    glm::dvec3 const cp2 = glm::cross(lineEnd - lineStart, bPoint - lineStart);
    return epona::fp::IsGreaterOrEqual(glm::dot(cp1, cp2), 0);
}

/**
 *  @brief  Checks if point is inside triangle
 *
 *  @param  triangleVertex1 triangle vertex
 *  @param  triangleVertex2 triangle vertex
 *  @param  triangleVertex3 triangle vertex
 *  @param  point           point of interest
 *
 *  @return @c true if point is inside triangle, @c false otherwise
 */
bool IsPointInsideTriangle(
    glm::dvec3 const& triangleVertex1, glm::dvec3 const& triangleVertex2,
    glm::dvec3 const& triangleVertex3, glm::dvec3 const& point
);
} // namespace intersection
} // namespace geometry
} // namespace pegasus

#endif // PEGASUS_INTERSECTION_HPP
