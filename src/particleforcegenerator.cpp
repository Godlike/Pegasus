#include "Pegas/include/particleforcegenerator.hpp"
#include <stdexcept>

void pegas::ParticleForceRegistry::add(pegas::Particle::Ptr & p, pegas::ParticleForceGenerator::Ptr & pfg)
{
    if (!p || !pfg)
    {
        throw std::invalid_argument("ParticleForceRegistry::add !p || !pfg");
    }

    auto particle = registrations.find(p);
    if (particle != registrations.end())
    {
        particle->second.insert(pfg);
    }
}

void pegas::ParticleForceRegistry::remove(pegas::Particle::Ptr const & p)
{
    if (!p)
    {
        throw std::invalid_argument("ParticleForceRegistry::remove !p");
    }

    auto entry = registrations.find(p);
    if (entry != registrations.end())
    {
        registrations.erase(entry);
    }
}

void pegas::ParticleForceRegistry::remove(pegas::Particle::Ptr const & p, pegas::ParticleForceGenerator::Ptr const & pfg)
{
    if (!p || !pfg)
    {
        throw std::invalid_argument("ParticleForceRegistry::remove !p || !pfg");
    }

    auto entry = registrations.find(p);
    if (entry != registrations.end())
    {
        auto force = entry->second.find(pfg);
        if (force != entry->second.end())
        {
            entry->second.erase(force);
        }
    }
}

void pegas::ParticleForceRegistry::clear()
{
    registrations.clear();
}

void pegas::ParticleForceRegistry::updateForces()
{
    for (auto & entry : registrations)
    {
        for(auto & force : entry.second)
        {
            force->updateForce(entry.first);
        }
    }

}

pegas::ParticleGravity::ParticleGravity(pegas::Vector3 const & g)
    : gravity(g)
{
}

void pegas::ParticleGravity::updateForce(pegas::Particle::Ptr const & p)
{
    if (!p->hasFiniteMass()) return;

    p->addForce(gravity * p->getMass());
}

pegas::ParticleDrag::ParticleDrag(pegas::real const k1, pegas::real const k2)
    : k1(k1), k2(k2)
{
}

void pegas::ParticleDrag::updateForce(pegas::Particle::Ptr const & p)
{
    Vector3 force = p->getVelocity();

    real dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    p->addForce(force);
}

pegas::ParticleSpring::ParticleSpring(pegas::Particle::Ptr const & other, pegas::real const springConstant, pegas::real const restLenght)
    : other(other), springConstant(springConstant), restLenght(restLenght)
{
}

void pegas::ParticleSpring::updateForce(pegas::Particle::Ptr const & p)
{
    Vector3 force = p->getPosition();
    force -= other->getPosition();

    real const magnitude
            = springConstant * std::fabs(force.magnitude() - restLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegas::ParticleAnchoredSpring::ParticleAnchoredSpring(pegas::Vector3 const & anchor, pegas::real const springConstant, pegas::real const restLenght)
    : anchor(anchor), springConstant(springConstant), restLenght(restLenght)
{
}

void pegas::ParticleAnchoredSpring::updateForce(pegas::Particle::Ptr const & p)
{
    Vector3 force = p->getPosition();
    force -= anchor;

    real const magnitude
            = springConstant * std::fabs(force.magnitude() - restLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegas::ParticleBungee::ParticleBungee(pegas::Particle::Ptr const & other, pegas::real const springConstant, pegas::real const restLenght)
    : other(other), springConstant(springConstant), restLenght(restLenght)
{
}

void pegas::ParticleBungee::updateForce(pegas::Particle::Ptr const & p)
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

pegas::ParticleBuoyancy::ParticleBuoyancy(pegas::real const maxDepth, pegas::real const volume, pegas::real const waterWight, pegas::real const liquidDensity)
    : maxDepth(maxDepth), volume(volume), waterHeight(waterWight), liquidDensity(liquidDensity)
{
}

void pegas::ParticleBuoyancy::updateForce(pegas::Particle::Ptr const & p)
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

pegas::ParticleFakeSpring::ParticleFakeSpring(pegas::Vector3 const & anchor, pegas::real const springConstant, pegas::real const damping)
    : anchor(anchor), springConstant(springConstant), damping(damping), duration(0)
{
}

void pegas::ParticleFakeSpring::updateForce(pegas::Particle::Ptr const & p, pegas::real const duration)
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

void pegas::ParticleFakeSpring::updateForce(pegas::Particle::Ptr const & p)
{
    updateForce(p, duration);
}
