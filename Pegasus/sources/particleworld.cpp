/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleWorld.hpp"

pegasus::ParticleWorld::ParticleWorld(
    Particles& particles,
    ParticleForceRegistry& forceRegistry,
    ParticleContactGenerators& contactGenerators,
    uint32_t maxContacts,
    uint32_t iterations)
    : m_particles(particles)
    , m_forceRegistry(forceRegistry)
    , m_contactGenerators(contactGenerators)
    , m_contactResolver(iterations)
    , m_calculateIterations(false)
    , m_maxContacts(maxContacts)
{
    m_contacts.reserve(m_maxContacts);
}

void pegasus::ParticleWorld::StartFrame() const
{
    for (auto& p : m_particles)
    {
        p.ClearForceAccumulator();
    }
}

void pegasus::ParticleWorld::RunPhysics(double duration)
{
    auto usedContacts = GenerateContacts();

    if (usedContacts)
    {
        if (m_calculateIterations)
        {
            m_contactResolver.SetIterations(usedContacts * 2);
        }
        m_contactResolver.ResolveContacts(m_contacts, duration);
    }

    m_forceRegistry.UpdateForces();
    Integrate(duration);
}

uint32_t pegasus::ParticleWorld::GenerateContacts()
{
    uint32_t limit = m_maxContacts;
    m_contacts.clear();

    for (auto const& g : m_contactGenerators)
    {
        limit -= g->AddContact(m_contacts, limit);

        if (limit == 0)
        {
            break;
        }
    }

    return m_maxContacts - limit;
}

void pegasus::ParticleWorld::Integrate(double duration) const
{
    for (auto& p : m_particles)
    {
        p.Integrate(duration);
    }
}
