/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleForceGenerator.hpp"

#include <cmath>

void pegasus::ParticleForceRegistry::Add(
    Particle& p, ParticleForceGenerator& pfg)
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

void pegasus::ParticleForceRegistry::Remove(Particle& p)
{
    auto entry = mRegistrations.find(&p);
    if (entry != mRegistrations.end())
    {
        mRegistrations.erase(entry);
    }
}

void pegasus::ParticleForceRegistry::Remove(
    Particle& p, ParticleForceGenerator& pfg)
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

void pegasus::ParticleGravity::UpdateForce(Particle& p)
{
    if (!p.HasFiniteMass())
    {
        return;
    }

    p.AddForce(m_gravity * p.GetMass());
}

pegasus::ParticleDrag::ParticleDrag(double k1, double k2)
    : m_k1(k1)
    , m_k2(k2)
{
}

void pegasus::ParticleDrag::UpdateForce(Particle& p)
{
    glm::dvec3 force = p.GetVelocity();

    double dragCoeff = force.length();
    dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

    force = glm::normalize(force) * -dragCoeff;
    p.AddForce(force);
}

pegasus::ParticleSpring::ParticleSpring(
    Particle& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleSpring::UpdateForce(Particle& p)
{
    glm::dvec3 force = p.GetPosition();
    force -= m_other.GetPosition();

    auto const magnitude = m_springConstant * std::fabs(force.length() - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.AddForce(force);
}

pegasus::ParticleAnchoredSpring::ParticleAnchoredSpring(
    glm::dvec3 const& anchor, double springConstant, double restLength)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleAnchoredSpring::UpdateForce(Particle& p)
{
    glm::dvec3 force = p.GetPosition();
    force -= m_anchor;

    auto const magnitude = m_springConstant * std::fabs(force.length() - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.AddForce(force);
}

pegasus::ParticleBungee::ParticleBungee(Particle& other, double springConstant, double restLength)
    : m_other(other)
    , m_springConstant(springConstant)
    , m_restLength(restLength)
{
}

void pegasus::ParticleBungee::UpdateForce(Particle& p)
{
    glm::dvec3 force = p.GetPosition();
    force -= m_other.GetPosition();

    double magnitude = force.length();
    if (magnitude <= m_restLength)
    {
        return;
    }

    magnitude = m_springConstant * (magnitude - m_restLength);

    force = glm::normalize(force) * -magnitude;
    p.AddForce(force);
}

pegasus::ParticleBuoyancy::ParticleBuoyancy(
    double maxDepth, double volume, double waterWight, double liquidDensity)
    : m_maxDepth(maxDepth)
    , m_volume(volume)
    , m_waterHeight(waterWight)
    , m_liquidDensity(liquidDensity)
{
}

void pegasus::ParticleBuoyancy::UpdateForce(Particle& p)
{
    double const depth = p.GetPosition().y;

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

    p.AddForce(force);
}

pegasus::ParticleFakeSpring::ParticleFakeSpring(
    glm::dvec3 const& anchor, double springConstant, double damping)
    : m_anchor(anchor)
    , m_springConstant(springConstant)
    , m_damping(damping)
    , m_duration(0)
{
}

void pegasus::ParticleFakeSpring::UpdateForce(Particle& p, double duration) const
{
    if (!p.HasFiniteMass())
    {
        return;
    }

    glm::dvec3 const position = p.GetPosition() - m_anchor;
    double const gamma = 0.5f * std::sqrt(4 * m_springConstant - m_damping * m_damping);

    if (gamma == 0.0f)
        return;

    auto const c = position * (m_damping / (2.0f * gamma)) + p.GetVelocity() * (1.0f / gamma);
    auto target = position * std::cos(gamma * duration) + c * sin(gamma * duration);
    target *= std::exp(-0.5f * duration * m_damping);

    auto const accel = (target - position) * (1.0f / duration * duration) - p.GetVelocity() * duration;
    p.AddForce(accel * p.GetMass());
}

void pegasus::ParticleFakeSpring::UpdateForce(Particle& p)
{
    UpdateForce(p, m_duration);
}
