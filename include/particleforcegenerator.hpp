#ifndef PEGAS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGAS_PARTICLE_FORCE_GENERATOR_HPP

#include "Pegas/include/particle.hpp"
#include <memory>
#include <map>
#include <set>


namespace pegas
{
    class ParticleForceGenerator
    {
    public:
        using Ptr = std::shared_ptr<ParticleForceGenerator>;
        using ConstPtr = std::shared_ptr<ParticleForceGenerator const>;

        virtual ~ParticleForceGenerator() {}

        virtual void updateForce(Particle::Ptr const & p) = 0;
    };

    class ParticleForceRegistry
    {
    public:
        void add(Particle::Ptr & p, ParticleForceGenerator::Ptr & pfg);

        void remove(Particle::Ptr const & p);

        void remove(Particle::Ptr const & p, ParticleForceGenerator::Ptr const & pfg);

        void clear();

        void updateForces();

    private:
        using Registry = std::map< Particle::Ptr, std::set<ParticleForceGenerator::Ptr> >;
        Registry registrations;
    };
}

#endif
