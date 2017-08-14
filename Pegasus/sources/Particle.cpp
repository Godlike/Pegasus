/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/Particle.hpp"

#include <limits>
#include <stdexcept>

pegasus::Particle::Particle()
    : m_damping(1.0)
    , m_mass(1.0)
    , m_inverseMass(1.0)
{
}

void pegasus::Particle::Integrate(double duration)
{
    if (!HasFiniteMass())
    {
        return;
    }

    if (duration <= 0)
    {
        return;
    }

    m_position += m_velocity * duration;

    glm::dvec3 const resultingAcc = m_acceleration + m_forceAccumulator * m_inverseMass;
    m_velocity += resultingAcc * duration;
    m_velocity *= std::pow(m_damping, duration);

    ClearForceAccumulator();
}

glm::dvec3 pegasus::Particle::GetPosition() const
{
    return m_position;
}

void pegasus::Particle::SetPosition(glm::dvec3 const& position)
{
    m_position = position;
}

void pegasus::Particle::SetPosition(double x, double y, double z)
{
    SetPosition({x, y, z});
}

glm::dvec3 pegasus::Particle::GetVelocity() const
{
    return m_velocity;
}

void pegasus::Particle::SetVelocity(glm::dvec3 const& velocity)
{
    m_velocity = velocity;
}

void pegasus::Particle::SetVelocity(double x, double y, double z)
{
    SetVelocity({x, y, z});
}

glm::dvec3 pegasus::Particle::GetAcceleration() const
{
    return m_acceleration;
}

void pegasus::Particle::SetAcceleration(glm::dvec3 const& acceleration)
{
    m_acceleration = acceleration;
}

void pegasus::Particle::SetAcceleration(double x, double y, double z)
{
    SetAcceleration({x, y, z});
}

double pegasus::Particle::GetDamping() const
{
    return m_damping;
}

void pegasus::Particle::SetDamping(double damping)
{
    m_damping = damping;
}

double pegasus::Particle::GetMass() const
{
    return (m_inverseMass == 0) ? std::numeric_limits<double>::max() : m_mass;
}

void pegasus::Particle::SetMass(double mass)
{
    if (mass <= 0)
    {
        return;
    }

    m_mass = mass;
    m_inverseMass = 1.0 / mass;
}

bool pegasus::Particle::HasFiniteMass() const
{
    return m_inverseMass != 0;
}

double pegasus::Particle::GetInverseMass() const
{
    return m_inverseMass;
}

void pegasus::Particle::SetInverseMass(double inverseMass)
{
    m_inverseMass = inverseMass;
}

void pegasus::Particle::AddForce(glm::dvec3 const& force)
{
    m_forceAccumulator += force;
}

void pegasus::Particle::ClearForceAccumulator()
{
    m_forceAccumulator = glm::dvec3();
}
