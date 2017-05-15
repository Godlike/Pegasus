#ifndef PEGASUS_PARTICLE_WORLD_HPP
#define PEGASUS_PARTICLE_WORLD_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include <vector>

namespace pegasus {

class ParticleWorld {
public:
    using ParticleContactGenerators = std::vector<ParticleContactGenerator::Ptr>;

    explicit ParticleWorld(unsigned int maxContacts, unsigned int iterations = 0);

    void runPhysics(double const duration);

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

    void integrate(double const duration);
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_WORLD_HPP
