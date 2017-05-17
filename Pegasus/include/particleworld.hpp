#ifndef PEGASUS_PARTICLE_WORLD_HPP
#define PEGASUS_PARTICLE_WORLD_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include <vector>
#include <list>
#include <memory>

namespace pegasus {

using ParticleContactGenerators = std::list<std::unique_ptr<ParticleContactGenerator>>;
using Particles = std::list<Particle>;

class ParticleWorld {
public:
    ParticleWorld(Particles & particles,
                  ParticleForceRegistry & forceRegistry,
                  ParticleContactGenerators & contactGenerators,
                  unsigned int maxContacts,
                  unsigned int iterations = 0);

    void startFrame();
    void runPhysics(double duration);

private:
    Particles & mParticles;
    ParticleForceRegistry & mForceRegistry;

    ParticleContacts mContacts;
    ParticleContactGenerators & mContactGenerators;
    ParticleContactResolver mContactResolver;

    bool mCalculateIterations;
    unsigned int mMaxContacts;

private:
    unsigned int generateContacts();
    void integrate(double duration);
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_WORLD_HPP
