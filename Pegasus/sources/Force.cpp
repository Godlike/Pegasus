/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*
* Copyright (c) Icosagon 2003. All Rights Reserved.
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include <pegasus/Force.hpp>
#include <pegasus/Integration.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/optimum_pow.hpp>

using namespace pegasus;
using namespace force;

StaticField::StaticField(glm::dvec3 force)
    : m_force(force)
{
}

glm::dvec3 StaticField::CalculateForce(mechanics::Body const& body) const
{
    return m_force * body.material.GetMass();
}

Drag::Drag(double k1, double k2)
    : m_k1(k1)
    , m_k2(k2)
{
}

glm::dvec3 Drag::CalculateForce(mechanics::Body const& body) const
{
    double const speed = glm::length(body.linearMotion.velocity);
    double const dragFactor = m_k1 * speed + m_k2 * glm::pow2(speed);

    return -glm::normalize(body.linearMotion.velocity) * dragFactor;
}

Spring::Spring(
    glm::dvec3 anchor, double springConstant, double restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::dvec3 Spring::CalculateForce(mechanics::Body const& body) const
{
    glm::dvec3 const direction = body.linearMotion.position - m_anchor;
    double const magnitude = m_springConstant * std::fabs(glm::length(direction) - m_restLength);

    return glm::normalize(direction) * -magnitude;
}

Bungee::Bungee(glm::dvec3 anchor, double springConstant, double restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::dvec3 Bungee::CalculateForce(mechanics::Body const& body) const
{
    glm::dvec3 force = body.linearMotion.position - m_anchor;

    double magnitude = glm::length(force);
    if (magnitude <= m_restLength)
    {
        return glm::dvec3();
    }

    magnitude = m_springConstant * (magnitude - m_restLength);

    force = glm::normalize(force) * -magnitude;
    return force;
}

Buoyancy::Buoyancy(
    double maxDepth, double volume, double waterWight, double liquidDensity)
    : m_maxDepth(maxDepth)
    , m_volume(volume)
    , m_waterHeight(waterWight)
    , m_liquidDensity(liquidDensity)
{
}

glm::dvec3 Buoyancy::CalculateForce(mechanics::Body const& body) const
{
    double const depth = body.linearMotion.position.y;

    if (depth >= m_waterHeight + m_maxDepth)
    {
        return glm::dvec3();
    }

    glm::dvec3 force;
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
