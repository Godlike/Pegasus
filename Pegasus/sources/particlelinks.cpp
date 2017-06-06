/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleLinks.hpp"

pegasus::ParticleLink::ParticleLink(Particle& a, Particle& b)
    : mA(a), mB(b)
{
}

double pegasus::ParticleLink::currentLength() const
{
    auto const relativePos = mA.getPosition() - mB.getPosition();
    return relativePos.magnitude();
}

pegasus::ParticleCabel::ParticleCabel(
    Particle & a,
    Particle & b,
    double maxLength,
    double restutuition)
    : ParticleLink(a, b)
    , maxLength(maxLength)
    , restitution(restutuition)
{
}

uint32_t
pegasus::ParticleCabel::addContact(ParticleContacts & contacts, uint32_t limit) const
{
    auto const length = currentLength();

    if (length < maxLength) {
        return 0;
    }

    Vector3 normal = (mB.getPosition() - mA.getPosition());
    normal.normalize();

    contacts.emplace_back(mA, &mB, restitution, normal, length - maxLength);
    return 1;
}

pegasus::ParticleRod::ParticleRod(Particle & a, Particle & b, double length)
    : ParticleLink(a, b)
    , length(length)
{
}

uint32_t
pegasus::ParticleRod::addContact(ParticleContacts& contacts, uint32_t limit) const
{
    double const currentLen = currentLength();

    if (currentLen == length) {
        return 0;
    }

    Vector3 normal = (mB.getPosition() - mA.getPosition());
    normal.normalize();

    contacts.emplace_back(mA, &mB, 0.0,
        (currentLen > length ? normal : normal * -1), 
        (currentLen > length ? currentLen - length : length - currentLen)
    );
    return 1;
}
