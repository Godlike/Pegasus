/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include <pegasus/ParticleForceGenerator.hpp>

#include <cmath>

void pegasus::ParticleForceRegistry::Add(
    integration::Body& p, ParticleForceGenerator& pfg)
{
    auto particle = mRegistrations.find(&p);
    if (particle != mRegistrations.end())
    {
        particle->second.insert(&pfg);
    }
    else
    {
        mRegistrations.insert(
            make_pair(&p, std::set<ParticleForceGenerator*>({&pfg})));
    }
}

void pegasus::ParticleForceRegistry::Remove(integration::Body& p)
{
    auto entry = mRegistrations.find(&p);
    if (entry != mRegistrations.end())
    {
        mRegistrations.erase(entry);
    }
}

void pegasus::ParticleForceRegistry::Remove(
    integration::Body& p, ParticleForceGenerator& pfg)
{
    auto entry = mRegistrations.find(&p);
    if (entry != mRegistrations.end())
    {
        auto force = entry->second.find(&pfg);
        if (force != entry->second.end())
        {
            entry->second.erase(force);
        }
    }
}

void pegasus::ParticleForceRegistry::Clear() { mRegistrations.clear(); }

void pegasus::ParticleForceRegistry::UpdateForces()
{
    for (auto& entry : mRegistrations)
    {
        for (auto& force : entry.second)
        {
            force->UpdateForce(*entry.first);
        }
    }
}

pegasus::ParticleGravity::ParticleGravity(glm::dvec3 const& g)
    : m_gravity(g)
{
}

void pegasus::ParticleGravity::UpdateForce(integration::Body& p)
{
    if (integration::FiniteMass(p.material.inverseMass))
    {
        p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, m_gravity * p.material.mass);
    }
}

pegasus::ParticleDrag::ParticleDrag(double k1, double k2)
    : m_k1(k1)
    , m_k2(k2)
{
}

void pegasus::ParticleDrag::UpdateForce(integration::Body& p)
{
    glm::dvec3 force = p.linearMotion.velocity;

    double dragCoeff = glm::length(force);
    dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

    force = glm::normalize(force) * -dragCoeff;
    p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, force);
}

pegasus::ParticleSpring::ParticleSpring(
    integration::Body& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleSpring::UpdateForce(integration::Body& p)
{
    glm::dvec3 force = p.linearMotion.position;
    force -= m_other.linearMotion.position;

    auto const magnitude = m_springConstant * std::fabs(glm::length(force) - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, force);
}

pegasus::ParticleAnchoredSpring::ParticleAnchoredSpring(
    glm::dvec3 const& anchor, double springConstant, double restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleAnchoredSpring::UpdateForce(integration::Body& p)
{
    glm::dvec3 force = p.linearMotion.position;
    force -= m_anchor;

    auto const magnitude = m_springConstant * std::fabs(glm::length(force) - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, force);
}

pegasus::ParticleBungee::ParticleBungee(integration::Body& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleBungee::UpdateForce(integration::Body& p)
{
    glm::dvec3 force = p.linearMotion.position;
    force -= m_other.linearMotion.position;

    double magnitude = glm::length(force);
    if (magnitude <= m_restLength)
    {
        return;
    }

    magnitude = m_springConstant * (magnitude - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, force);
}

pegasus::ParticleBuoyancy::ParticleBuoyancy(
    double maxDepth, double volume, double waterWight, double liquidDensity)
    : m_maxDepth(maxDepth)
    , m_volume(volume)
    , m_waterHeight(waterWight)
    , m_liquidDensity(liquidDensity)
{
}

void pegasus::ParticleBuoyancy::UpdateForce(integration::Body& p)
{
    double const depth = p.linearMotion.position.y;

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

    p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, force);
}

pegasus::ParticleFakeSpring::ParticleFakeSpring(
    glm::dvec3 const& anchor, double springConstant, double damping)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_damping(damping)
    , m_duration(0)
{
}

void pegasus::ParticleFakeSpring::UpdateForce(integration::Body& p, double duration) const
{
    if (integration::FiniteMass(p.material.inverseMass))
    {
        glm::dvec3 const position = p.linearMotion.position - m_anchor;
        double const gamma = 0.5f * std::sqrt(4 * m_springConstant - m_damping * m_damping);

        if (gamma == 0.0f)
            return;

        auto const c = position * (m_damping / (2.0f * gamma)) + p.linearMotion.velocity * (1.0f / gamma);
        auto target = position * std::cos(gamma * duration) + c * sin(gamma * duration);
        target *= std::exp(-0.5f * duration * m_damping);

        auto const accel = (target - position) * (1.0f / duration * duration) - p.linearMotion.velocity * duration;
        p.linearMotion.force = integration::IntegrateForce(p.linearMotion.force, accel * p.material.mass);
    }
}

void pegasus::ParticleFakeSpring::UpdateForce(integration::Body& p)
{
    UpdateForce(p, m_duration);
}
