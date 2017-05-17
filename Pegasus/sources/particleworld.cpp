#include "Pegasus/include/particleworld.hpp"

pegasus::ParticleWorld::ParticleWorld(
    Particles & particles,
    ParticleForceRegistry & forceRegistry,
    ParticleContactGenerators & contactGenerators,
    unsigned int maxContacts,
    unsigned int iterations)
    : mParticles(particles)
    , mForceRegistry(forceRegistry)
    , mContactGenerators(contactGenerators)
    , mContactResolver(iterations)
    , mCalculateIterations(false)
    , mMaxContacts(maxContacts)
{
    mContacts.reserve(mMaxContacts);
}

void pegasus::ParticleWorld::startFrame()
{
    for (auto & p : mParticles) {
        p.clearForceAccum();
    }
}

void pegasus::ParticleWorld::runPhysics(double duration)
{
    mForceRegistry.updateForces();

    integrate(duration);

    auto usedContacts = generateContacts();

    if (usedContacts) {
        if (mCalculateIterations) {
            mContactResolver.setIterations(usedContacts * 2);
        }
        mContactResolver.resolveContacts(mContacts, duration);
    }
}

unsigned int pegasus::ParticleWorld::generateContacts()
{
    unsigned int limit = mMaxContacts;
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

void pegasus::ParticleWorld::integrate(double duration)
{
    for (auto & p : mParticles) {
        p.integrate(duration);
    }
}
