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
        ParticleGravity(Vector3 const & g)
            : gravity(g)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            if (!p->hasFiniteMass()) return;

            p->addForce(gravity * p->getMass());
        }

    private:
        Vector3 const gravity;
    };


    class ParticleDrag : public ParticleForceGenerator
    {
    public:
        ParticleDrag(real const k1, real const k2)
            : k1(k1), k2(k2)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            Vector3 force = p->getVelocity();

            real dragCoeff = force.magnitude();
            dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

            force.normalize();
            force *= -dragCoeff;
            p->addForce(force);
        }

    private:
        real const k1;
        real const k2;
    };


    class ParticleSpring : public ParticleForceGenerator
    {
    public:
        ParticleSpring(Particle::Ptr const & other, real const springConstant, real const restLenght)
            : other(other), springConstant(springConstant), restLenght(restLenght)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            Vector3 force = p->getPosition();
            force -= other->getPosition();

            real const magnitude
                = springConstant * std::fabs(force.magnitude() - restLenght);

            force.normalize();
            force *= -magnitude;
            p->addForce(force);
        }

    private:
        Particle::Ptr const other;
        real const springConstant;
        real const restLenght;
    };


    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
    public:
        ParticleAnchoredSpring(Vector3 const & anchor, real const springConstant, real const restLenght)
            : anchor(anchor), springConstant(springConstant), restLenght(restLenght)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            Vector3 force = p->getPosition();
            force -= anchor;

            real const magnitude
                = springConstant * std::fabs(force.magnitude() - restLenght);

            force.normalize();
            force *= -magnitude;
            p->addForce(force);
        }

    private:
        Vector3 const anchor;
        real const springConstant;
        real const restLenght;
    };


    class ParticleBungee : public ParticleForceGenerator
    {
    public:
        ParticleBungee(Particle::Ptr const & other, real const springConstant, real const restLenght)
            : other(other), springConstant(springConstant), restLenght(restLenght)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            Vector3 force = p->getPosition();
            force -= other->getPosition();

            real magnitude = force.magnitude();
            if (magnitude <= restLenght) return;

            magnitude = springConstant * (magnitude - restLenght);

            force.normalize();
            force *= -magnitude;
            p->addForce(force);
        }

    private:
        Particle::Ptr const other;
        real const springConstant;
        real const restLenght;
    };


    class ParticleBuoyancy : public ParticleForceGenerator
    {
    public:
        ParticleBuoyancy(real const maxDepth, real const volume, real const waterWight, real const liquidDensity)
            : maxDepth(maxDepth), volume(volume), waterHeight(waterWight), liquidDensity(liquidDensity)
        {
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            real const depth = p->getPosition().y;

            if(depth >= waterHeight + maxDepth) return;

            Vector3 force;
            if(depth <= waterHeight - maxDepth)
            {
                force.y = liquidDensity * volume;
            }
            else
            {
                force.y = liquidDensity * volume *(depth - maxDepth - waterHeight) / 2.0f * maxDepth;
            }

            p->addForce(force);
        }

    private:
        real const maxDepth;
        real const volume;
        real const waterHeight;
        real const liquidDensity;
    };


    class ParticleFakeSpring : public ParticleForceGenerator
    {
    public:
        ParticleFakeSpring(Vector3 const & anchor, real const springConstant, real const damping)
            : anchor(anchor), springConstant(springConstant), damping(damping), duration(0)
        {
        }

        void updateForce(Particle::Ptr const & p, real const duration)
        {
            if (!p->hasFiniteMass()) return;

            Vector3 const position = p->getPosition() - anchor;
            real const gamma = 0.5f * std::sqrt(4 * springConstant - damping * damping);

            if (gamma == 0.0f) return;

            Vector3 const c = position * (damping / (2.0f * gamma)) + p->getVelocity() * (1.0f / gamma);
            Vector3 target = position * std::cos(gamma * duration) + c * std::sin(gamma * duration);
            target *= std::exp(-0.5f * duration * damping);

            Vector3 const accel = (target - position) * (1.0f / duration * duration) - p->getVelocity() * duration;
            p->addForce(accel * p->getMass());
        }

        virtual void updateForce(Particle::Ptr const & p) override
        {
            updateForce(p, duration);
        }

    private:
        Vector3 const anchor;
        real const springConstant;
        real const damping;
        real duration;
    };

}// namespace pegas

#endif// PEGAS_PARTICLE_FORCE_GENERATOR_HPP
