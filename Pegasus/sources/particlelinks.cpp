/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleLinks.hpp"

pegasus::ParticleLink::ParticleLink(Particle& a, Particle& b)
    : m_aParticle(a)
    , m_bParticle(b)
{
}

double pegasus::ParticleLink::CurrentLength() const
{
    auto const relativePos = m_aParticle.GetPosition() - m_bParticle.GetPosition();
    return relativePos.Magnitude();
}

pegasus::ParticleCabel::ParticleCabel(
    Particle& a,
    Particle& b,
    double maxLength,
    double restutuition)
    : ParticleLink(a, b)
    , m_maxLength(maxLength)
    , m_restitution(restutuition)
{
}

uint32_t
pegasus::ParticleCabel::AddContact(ParticleContacts& contacts, uint32_t limit) const
{
    auto const length = CurrentLength();

    if (length < m_maxLength)
    {
        return 0;
    }

    Vector3 normal = (m_bParticle.GetPosition() - m_aParticle.GetPosition());
    normal.Normalize();

    contacts.emplace_back(m_aParticle, &m_bParticle, m_restitution, normal, length - m_maxLength);
    return 1;
}

pegasus::ParticleRod::ParticleRod(Particle& a, Particle& b, double length)
    : ParticleLink(a, b)
    , m_length(length)
{
}

uint32_t
pegasus::ParticleRod::AddContact(ParticleContacts& contacts, uint32_t limit) const
{
    double const currentLen = CurrentLength();

    if (currentLen == m_length)
    {
        return 0;
    }

    Vector3 normal = (m_bParticle.GetPosition() - m_aParticle.GetPosition());
    normal.Normalize();

    contacts.emplace_back(m_aParticle, &m_bParticle, 0.0,
                          (currentLen > m_length ? normal : normal * -1),
                          (currentLen > m_length ? currentLen - m_length : m_length - currentLen)
    );
    return 1;
}
