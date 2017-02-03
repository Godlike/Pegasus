#ifndef PEGAS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGAS_PARTICLE_FORCE_GENERATOR_HPP

#include "Pegas/include/particle.hpp"
#include <memory>
#include <map>
#include <set>
#include <cmath>


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


    class ParticleGravity : public ParticleForceGenerator
    {
    public:
        ParticleGravity(Vector3 const & g);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        Vector3 const gravity;
    };


    class ParticleDrag : public ParticleForceGenerator
    {
    public:
        ParticleDrag(real const k1, real const k2);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        real const k1;
        real const k2;
    };


    class ParticleSpring : public ParticleForceGenerator
    {
    public:
        ParticleSpring(Particle::Ptr const & other, real const springConstant, real const restLenght);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        Particle::Ptr const other;
        real const springConstant;
        real const restLenght;
    };


    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
    public:
        ParticleAnchoredSpring(Vector3 const & anchor, real const springConstant, real const restLenght);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        Vector3 const anchor;
        real const springConstant;
        real const restLenght;
    };


    class ParticleBungee : public ParticleForceGenerator
    {
    public:
        ParticleBungee(Particle::Ptr const & other, real const springConstant, real const restLenght);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        Particle::Ptr const other;
        real const springConstant;
        real const restLenght;
    };


    class ParticleBuoyancy : public ParticleForceGenerator
    {
    public:
        ParticleBuoyancy(real const maxDepth, real const volume, real const waterWight, real const liquidDensity);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        real const maxDepth;
        real const volume;
        real const waterHeight;
        real const liquidDensity;
    };


    class ParticleFakeSpring : public ParticleForceGenerator
    {
    public:
        ParticleFakeSpring(Vector3 const & anchor, real const springConstant, real const damping);

        void updateForce(Particle::Ptr const & p, real const duration);

        virtual void updateForce(Particle::Ptr const & p) override;

    private:
        Vector3 const anchor;
        real const springConstant;
        real const damping;
        real duration;
    };

}// namespace pegas

#endif// PEGAS_PARTICLE_FORCE_GENERATOR_HPP
