#include "Pegasus/include/particleworld.hpp"

pegasus::ParticleWorld::ParticleWorld(unsigned int maxContacts, unsigned int iterations)
    : mResolver(iterations)
    , mCalculateIterations(false)
    , mMaxContacts(maxContacts)
{
}

void pegasus::ParticleWorld::startFrame()
{
    for (auto const& p : mParticles) {
        p->clearForceAccum();
    }
}

void pegasus::ParticleWorld::setParticles(Particles particles)
{
    mParticles = particles;
}

void pegasus::ParticleWorld::setParticleForcesRegistry(
    ParticleForceRegistry::Ptr registry)
{
    mRegistry = registry;
}

void pegasus::ParticleWorld::setParticleContactGenerators(
    ParticleContactGenerators generators)
{
    mGeneratos = generators;
}

void pegasus::ParticleWorld::runPhysics(double const duration)
{
    mRegistry->updateForces();

    integrate(duration);

    auto usedContacts = generateContacts();

    if (usedContacts) {
        if (mCalculateIterations) {
            mResolver.setIterations(usedContacts * 2);
        }
        mResolver.resolveContacts(mContacts, duration);
    }
}

unsigned int pegasus::ParticleWorld::generateContacts()
{
    auto limit = mMaxContacts;
    mContacts.clear();

    for (auto const& g : mGeneratos) {
        auto const used = g->addContact(mContacts, limit);
        limit -= used;

        if (limit == 0) {
            break;
        }
    }
    return mMaxContacts - limit;
}

void pegasus::ParticleWorld::integrate(double const duration)
{
    for (auto const& p : mParticles) {
        p->integrate(duration);
    }
}
