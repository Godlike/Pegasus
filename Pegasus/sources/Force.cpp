/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include <pegasus/Force.hpp>
#include <pegasus/Integration.hpp>

using namespace pegasus;
using namespace force;

StaticField::StaticField(glm::dvec3 const& g)
    : m_force(g)
{
}

glm::dvec3 StaticField::CalculateForce(mechanics::Body& body) const
{
    return m_force * body.material.GetMass();
}

Drag::Drag(double k1, double k2)
    : m_k1(k1)
    , m_k2(k2)
{
}

glm::dvec3 Drag::CalculateForce(mechanics::Body& body) const
{
    glm::dvec3 force = body.linearMotion.velocity;

    double dragCoeff = glm::length(force);
    dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

    force = glm::normalize(force) * -dragCoeff;
    return force;
}

Spring::Spring(
    mechanics::Body& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::dvec3 Spring::CalculateForce(mechanics::Body& body) const
{
    glm::dvec3 force = body.linearMotion.position - m_other.linearMotion.position;

    auto const magnitude = m_springConstant * std::fabs(glm::length(force) - m_restLength);

    force = glm::normalize(force) * -magnitude;
    return force;
}

AnchoredSpring::AnchoredSpring(
    glm::dvec3 const& anchor, double springConstant, double restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::dvec3 AnchoredSpring::CalculateForce(mechanics::Body& body) const
{
    glm::dvec3 force = body.linearMotion.position - m_anchor;
    auto const magnitude = m_springConstant * std::fabs(glm::length(force) - m_restLength);

    force = glm::normalize(force) * -magnitude;
    return force;
}

Bungee::Bungee(mechanics::Body& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

glm::dvec3 Bungee::CalculateForce(mechanics::Body& body) const
{
    glm::dvec3 force = body.linearMotion.position - m_other.linearMotion.position;

    double magnitude = glm::length(force);
    if (magnitude <= m_restLength)
    {
        return;
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

glm::dvec3 Buoyancy::CalculateForce(mechanics::Body& body) const
{
    double const depth = body.linearMotion.position.y;

    if (depth >= m_waterHeight + m_maxDepth)
    {
        return;
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
