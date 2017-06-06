/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/ParticleWorld.hpp"

pegasus::ParticleWorld::ParticleWorld(
    Particles & particles,
    ParticleForceRegistry & forceRegistry,
    ParticleContactGenerators & contactGenerators,
    uint32_t maxContacts,
    uint32_t iterations)
    : mParticles(particles)
    , mForceRegistry(forceRegistry)
    , mContactGenerators(contactGenerators)
    , mContactResolver(iterations)
    , mCalculateIterations(false)
    , mMaxContacts(maxContacts)
{
    mContacts.reserve(mMaxContacts);
}

void pegasus::ParticleWorld::startFrame() const
{
    for (auto & p : mParticles) {
        p.ClearForceAccumulator();
    }
}

void pegasus::ParticleWorld::runPhysics(double duration)
{
    auto usedContacts = generateContacts();

    if (usedContacts) {
        if (mCalculateIterations) {
            mContactResolver.setIterations(usedContacts * 2);
        }
        mContactResolver.resolveContacts(mContacts, duration);
    }

    mForceRegistry.updateForces();
    integrate(duration);
}

uint32_t pegasus::ParticleWorld::generateContacts()
{
    uint32_t limit = mMaxContacts;
    mContacts.clear();

    for (auto const& g : mContactGenerators)
    {
        limit -= g->addContact(mContacts, limit);

        if (limit == 0) {
            break;
        }
    }

    return mMaxContacts - limit;
}

void pegasus::ParticleWorld::integrate(double duration) const
{
    for (auto & p : mParticles) {
        p.Integrate(duration);
    }
}
