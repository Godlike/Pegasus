/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <math/HyperPlane.hpp>

using namespace pegasus;
using namespace math;

HyperPlane::HyperPlane(glm::dvec3 const& normal, glm::dvec3 const& point, glm::dvec3 const* below)
    : m_normal(normal)
    , m_point(point)
    , m_distance(glm::dot(m_normal, m_point))
{
    if (below != nullptr)
    {
        glm::dvec3 const outward = point - *below;
        if (glm::dot(outward, m_normal) < 0.0)
        {
            SetNormal(m_normal * -1.0);
        }
    }
}

HyperPlane::HyperPlane(
    glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c, glm::dvec3 const* below
)
    : HyperPlane(glm::normalize(glm::cross(a - c, b - c)), c, below)
{
}

HyperPlane::HyperPlane(glm::dmat3 const& vertices, glm::dvec3 const* below)
    : HyperPlane(vertices[0], vertices[1], vertices[2], below)
{
}

glm::dvec3 const& HyperPlane::GetPoint() const
{
    return m_point;
}

glm::dvec3 const& HyperPlane::GetNormal() const
{
    return m_normal;
}

double HyperPlane::GetDistance() const
{
    return m_distance;
}

void HyperPlane::SetNormal(glm::dvec3 const& normal)
{
    m_normal = normal;
    m_distance = glm::dot(m_normal, m_point);
}

void HyperPlane::SetPoint(glm::dvec3 const& point)
{
    m_point = point;
    m_distance = glm::dot(m_normal, m_point);
}

double HyperPlane::Distance(glm::dvec3 const& point) const
{
    return glm::abs(SignedDistance(point));
}

double HyperPlane::SignedDistance(glm::dvec3 const& point) const
{
    return glm::dot(m_normal, point) - m_distance;
}

bool HyperPlane::RayIntersection(
    glm::dvec3 const& rayNormal, glm::dvec3 const& rayPoint, glm::dvec3& resultPoint
) const
{
    double const rayPlaneProjection = glm::dot(m_normal, rayNormal);

    if (rayPlaneProjection != 0.0)
    {
        double const t = (glm::dot(m_normal, m_point - rayPoint)) / rayPlaneProjection;
        resultPoint = rayPoint + rayNormal * t;
        return true;
    }

    return false;
}

bool HyperPlane::LineSegmentIntersection(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3& resultPoint
) const
{
    if ((glm::dot(lineStart, m_normal) - m_distance)
        * (glm::dot(lineEnd, m_normal) - m_distance) >= 0)
    {
        return false;
    }

    glm::dvec3 const lineNormal = glm::normalize(lineEnd - lineStart);

    return RayIntersection(lineNormal, lineStart, resultPoint);
}

glm::dvec3 HyperPlane::ClosestPoint(const glm::dvec3& point) const
{
    glm::dvec3 const closestPoint = point - (glm::dot(point, m_normal) - m_distance) * m_normal;

    return closestPoint;
}
