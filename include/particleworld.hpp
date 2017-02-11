#ifndef PEGAS_PARTICLE_WORLD_HPP
#define PEGAS_PARTICLE_WORLD_HPP

#include "Pegas/include/particle.hpp"
#include "Pegas/include/particlecontacts.hpp"
#include "Pegas/include/particleforcegenerator.hpp"
#include <vector>

namespace pegas
{

    class ParticleWorld
    {
    public:
        using Particles = std::vector<Particle::Ptr>;
        using ParticleContacts = std::vector<ParticleContact::Ptr>;
        using ParticleContactGenerators = std::vector<ParticleContactGenerator::Ptr>;

        ParticleWorld(unsigned int maxContacts, unsigned int iterations = 0);

        ~ParticleWorld();

        unsigned int generateContacts();

        void integrate(real const duration);

        void runPhysics(real const duration);

        void startFrame();

    private:
        Particles particles;
        ParticleForceRegistry registry;
        ParticleContacts contacts;
        ParticleContactResolver resolver;
        ParticleContactGenerators generatos;

        bool calculateIterations;
        unsigned int maxContacts;
    };

} //namespace pegas

#endif //PEGAS_PARTICLE_WORLD_HPP
