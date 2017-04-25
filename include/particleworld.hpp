#ifndef PEGAS_PARTICLE_WORLD_HPP
#define PEGAS_PARTICLE_WORLD_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include <vector>

namespace pegas {

class ParticleWorld {
public:
    using Particles = std::vector<Particle::Ptr>;
    using ParticleContacts = std::vector<ParticleContact::Ptr>;
    using ParticleContactGenerators = std::vector<ParticleContactGenerator::Ptr>;

    ParticleWorld(unsigned int maxContacts, unsigned int iterations = 0);

    void runPhysics(real const duration);

    void startFrame();

    void setParticles(Particles particles);

    void setParticleForcesRegistry(ParticleForceRegistry::Ptr registry);

    void setParticleContactGenerators(ParticleContactGenerators generators);

private:
    Particles mParticles;
    ParticleContacts mContacts;

    ParticleForceRegistry::Ptr mRegistry;
    ParticleContactGenerators mGeneratos;
    ParticleContactResolver mResolver;

    bool mCalculateIterations;
    unsigned int mMaxContacts;

    unsigned int generateContacts();

    void integrate(real const duration);
};

} // namespace pegas

#endif // PEGAS_PARTICLE_WORLD_HPP
