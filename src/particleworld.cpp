#include "Pegas/include/particleworld.hpp"

pegas::ParticleWorld::ParticleWorld(unsigned int maxContacts,
    unsigned int iterations)
    : mResolver(iterations)
    , mMaxContacts(maxContacts)
{
}

void pegas::ParticleWorld::startFrame()
{
    for (auto const& p : mParticles) {
        p->clearForceAccum();
    }
}

void pegas::ParticleWorld::setParticles(
    pegas::ParticleWorld::Particles particles)
{
    mParticles = particles;
}

void pegas::ParticleWorld::setParticleForcesRegistry(
    pegas::ParticleForceRegistry::Ptr registry)
{
    mRegistry = registry;
}

void pegas::ParticleWorld::setParticleContactGenerators(
    pegas::ParticleWorld::ParticleContactGenerators generators)
{
    mGeneratos = generators;
}

void pegas::ParticleWorld::runPhysics(pegas::real const duration)
{
    pegas::real const duration_ = 0.005;

    mRegistry->updateForces();

    integrate(duration_);

    unsigned int usedContacts = generateContacts();

    if (usedContacts) {
        if (mCalculateIterations) {
            mResolver.setIterations(usedContacts * 2);
        }
        mResolver.resolveContacts(mContacts, duration_);
    }
}

unsigned int pegas::ParticleWorld::generateContacts()
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
    // Return the number of contacts used.
    return mMaxContacts - limit;
}

void pegas::ParticleWorld::integrate(pegas::real const duration)
{
    for (auto const& p : mParticles) {
        p->integrate(duration);
    }
}
