/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <math/Math.hpp>
#include <glm/glm.hpp>

double pegasus::math::LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 point
)
{
    point = point - lineStart;
    glm::dvec3 const lineDirection = glm::normalize(lineEnd - lineStart);
    glm::dvec3 const pointLineProjection = glm::dot(lineDirection, point) * lineDirection;
    return glm::distance(point, pointLineProjection);
}
