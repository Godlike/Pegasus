/*
 * Copyright (c) Icosagon 2003.
 * Copyright (C) 2017-2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#include <pegasus/Force.hpp>
#include <pegasus/Integration.hpp>
#include <Epona/FloatingPoint.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/optimum_pow.hpp>
#include <glm/gtx/norm.hpp>

namespace pegasus
{
namespace force
{

StaticField::StaticField(glm::vec3 force)
    : m_force(force)
{
}

glm::vec3 StaticField::CalculateForce(mechanics::Body const& body) const
{
    return m_force * body.material.GetMass();
}

SquareDistanceSource::SquareDistanceSource(float magnitude, glm::vec3 centerOfMass)
    : centerOfMass(centerOfMass)
    , m_magnitude(magnitude)
{
}

glm::vec3 SquareDistanceSource::CalculateForce(mechanics::Body const& body) const
{
    float const distance = glm::distance(body.linearMotion.position, centerOfMass);
    glm::vec3 force{ 0 };

    if (!epona::fp::IsZero(distance))
    {
        glm::vec3 const direction = glm::normalize(centerOfMass - body.linearMotion.position);

        if (!epona::fp::IsZero(glm::length(direction)))
        {
            force = direction * (m_magnitude / glm::pow2(distance));
        }
    }

    return force;
}

Drag::Drag(float k1, float k2)
    : m_k1(k1)
    , m_k2(k2)
{
}

glm::vec3 Drag::CalculateForce(mechanics::Body const& body) const
{
    float const speedSq = glm::length2(body.linearMotion.velocity);
    if (epona::fp::IsZero(speedSq) || std::isinf(speedSq) || std::isnan(speedSq))
    {
        return glm::vec3{ 0 };
    }

    float const dragFactor = m_k1 * glm::sqrt(speedSq) + m_k2 * speedSq;
    glm::vec3 const force = -glm::normalize(body.linearMotion.velocity) * dragFactor;

    float const magnitudeSq = glm::length2(force);
    if (epona::fp::IsZero(magnitudeSq) || std::isinf(magnitudeSq) || std::isnan(magnitudeSq))
    {
        return glm::vec3{ 0 };
    }

    return force;
}

Spring::Spring(
    glm::vec3 anchor, float springConstant, float restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::vec3 Spring::CalculateForce(mechanics::Body const& body) const
{
    glm::vec3 const direction = body.linearMotion.position - m_anchor;
    float const magnitude = m_springConstant * std::fabs(glm::length(direction) - m_restLength);

    return glm::normalize(direction) * -magnitude;
}

Bungee::Bungee(glm::vec3 anchor, float springConstant, float restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::vec3 Bungee::CalculateForce(mechanics::Body const& body) const
{
    glm::vec3 force = body.linearMotion.position - m_anchor;

    float magnitude = glm::length(force);
    if (magnitude <= m_restLength)
    {
        return glm::vec3();
    }

    magnitude = m_springConstant * (magnitude - m_restLength);

    force = glm::normalize(force) * -magnitude;
    return force;
}

Buoyancy::Buoyancy(
    float maxDepth, float volume, float waterWight, float liquidDensity)
    : m_maxDepth(maxDepth)
    , m_volume(volume)
    , m_waterHeight(waterWight)
    , m_liquidDensity(liquidDensity)
{
}

glm::vec3 Buoyancy::CalculateForce(mechanics::Body const& body) const
{
    float const depth = body.linearMotion.position.y;

    if (depth >= m_waterHeight + m_maxDepth)
    {
        return glm::vec3();
    }

    glm::vec3 force;
    if (depth <= m_waterHeight - m_maxDepth)
    {
        force.y = m_liquidDensity * m_volume;
    }
    else
    {
        force.y = m_liquidDensity * m_volume * (depth - m_maxDepth - m_waterHeight) / 2.0f * m_maxDepth;
    }

    return force;
}
} // namespace force
} // namespace pegasus
