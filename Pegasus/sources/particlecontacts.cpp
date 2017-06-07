/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleContacts.hpp"

pegasus::ParticleContact::ParticleContact(
    Particle& a,
    Particle* b,
    double restitution,
    Vector3 const& contactNormal,
    double penetration)
    : m_pParticleA(&a)
    , m_pParticleB(b)
    , m_restitution(restitution)
    , m_contactNormal(contactNormal)
    , m_penetration(penetration)
{
}

void pegasus::ParticleContact::Resolve(double duration) const
{
    if (duration < 0)
    {
        return;
    }

    ResolveVelocity(duration);
    ResolveInterpenetration();
}

double pegasus::ParticleContact::CalculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = m_pParticleA->GetVelocity();
    if (m_pParticleB)
    {
        relativeVelocity -= m_pParticleB->GetVelocity();
    }

    return relativeVelocity * m_contactNormal;
}

void pegasus::ParticleContact::ResolveVelocity(double duration) const
{
    auto const separatingVelocity = CalculateSeparatingVelocity();
    if (separatingVelocity > 0)
    {
        return;
    }

    auto newSepVelocity = -separatingVelocity * m_restitution;
    auto accCausedVelocity = m_pParticleA->GetAcceleration();
    if (m_pParticleB)
    {
        accCausedVelocity -= m_pParticleB->GetAcceleration();
    }
    auto const accCausedSepVelocity = accCausedVelocity * m_contactNormal * duration;
    if (accCausedSepVelocity < 0)
    {
        newSepVelocity += m_restitution * accCausedSepVelocity;

        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }
    auto const deltaVelocity = newSepVelocity - separatingVelocity;

    auto totalInverseMass = m_pParticleA->GetInverseMass();
    if (m_pParticleB)
    {
        totalInverseMass += m_pParticleB->GetInverseMass();
    }

    if (totalInverseMass <= 0)
    {
        return;
    }

    auto const impulse = deltaVelocity / totalInverseMass;
    auto const impulsePerIMass = m_contactNormal * impulse;

    m_pParticleA->SetVelocity(m_pParticleA->GetVelocity() + impulsePerIMass * m_pParticleA->GetInverseMass());
    if (m_pParticleB)
    {
        m_pParticleB->SetVelocity(m_pParticleB->GetVelocity() + impulsePerIMass * -m_pParticleB->GetInverseMass());
    }
}

void pegasus::ParticleContact::ResolveInterpenetration() const
{
    if (m_penetration <= 0)
    {
        return;
    }

    double totalInverseMass = m_pParticleA->GetInverseMass();
    if (m_pParticleB)
    {
        totalInverseMass += m_pParticleB->GetInverseMass();
    }

    if (totalInverseMass <= 0)
    {
        return;
    }

    Vector3 const movePerIMass = m_contactNormal * (m_penetration / totalInverseMass);
    m_pParticleA->SetPosition(m_pParticleA->GetPosition() + movePerIMass * m_pParticleA->GetInverseMass());
    m_pParticleA->AddForce((movePerIMass * m_pParticleA->GetInverseMass()).Inverse());

    if (m_pParticleB)
    {
        m_pParticleB->SetPosition(m_pParticleB->GetPosition() - movePerIMass * m_pParticleB->GetInverseMass());
        m_pParticleB->AddForce((movePerIMass * m_pParticleB->GetInverseMass()).Inverse());
    }
}

pegasus::ParticleContactResolver::ParticleContactResolver(uint32_t iterations)
    : m_iterations(iterations)
    , m_iterationsUsed(0)
{
}

void pegasus::ParticleContactResolver::SetIterations(uint32_t iterations)
{
    m_iterations = iterations;
}

void pegasus::ParticleContactResolver::ResolveContacts(ParticleContacts& contacts, double duration)
{
    m_iterationsUsed = 0;

    std::sort(contacts.begin(), contacts.end(),
              [](ParticleContact const& a, ParticleContact const& b)
              {
                  return a.CalculateSeparatingVelocity() < b.CalculateSeparatingVelocity();
              });

    while (m_iterationsUsed++ < m_iterations && !contacts.empty())
    {
        auto maxSepVelocityContact = contacts.back();
        contacts.pop_back();
        maxSepVelocityContact.Resolve(duration);
    }
}

pegasus::ParticleContactGenerator::~ParticleContactGenerator()
{
}
