#include "Pegasus/include/particleforcegenerator.hpp"

#include <cmath>
#include <stdexcept>

void pegasus::ParticleForceRegistry::add(
    Particle::Ptr p, ParticleForceGenerator::Ptr pfg)
{
    if (!p || !pfg) {
        throw std::invalid_argument("ParticleForceRegistry::add !p || !pfg");
    }

    auto particle = mRegistrations.find(p);
    if (particle != mRegistrations.end()) {
        particle->second.insert(pfg);
    } else {
        mRegistrations.insert(
            make_pair(p, std::set<ParticleForceGenerator::Ptr>({ pfg })));
    }
}

void pegasus::ParticleForceRegistry::remove(Particle::Ptr const p)
{
    if (!p) {
        throw std::invalid_argument("ParticleForceRegistry::remove !p");
    }

    auto entry = mRegistrations.find(p);
    if (entry != mRegistrations.end()) {
        mRegistrations.erase(entry);
    }
}

void pegasus::ParticleForceRegistry::remove(
    Particle::Ptr const p,
    ParticleForceGenerator::Ptr const pfg)
{
    if (!p || !pfg) {
        throw std::invalid_argument("ParticleForceRegistry::remove !p || !pfg");
    }

    auto entry = mRegistrations.find(p);
    if (entry != mRegistrations.end()) {
        auto force = entry->second.find(pfg);
        if (force != entry->second.end()) {
            entry->second.erase(force);
        }
    }
}

void pegasus::ParticleForceRegistry::clear() { mRegistrations.clear(); }

void pegasus::ParticleForceRegistry::updateForces()
{
    for (auto& entry : mRegistrations) {
        for (auto& force : entry.second) {
            force->updateForce(entry.first);
        }
    }
}

pegasus::ParticleGravity::ParticleGravity(Vector3 const& g)
    : mGravity(g)
{
}

void pegasus::ParticleGravity::updateForce(Particle::Ptr const& p)
{
    if (!p->hasFiniteMass())
        return;

    p->addForce(mGravity * p->getMass());
}

pegasus::ParticleDrag::ParticleDrag(real const k1, real const k2)
    : mK1(k1)
    , mK2(k2)
{
}

void pegasus::ParticleDrag::updateForce(Particle::Ptr const& p)
{
    auto force = p->getVelocity();

    auto dragCoeff = force.magnitude();
    dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    p->addForce(force);
}

pegasus::ParticleSpring::ParticleSpring(Particle::Ptr const& other,
    real const springConstant,
    real const restLenght)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegasus::ParticleSpring::updateForce(Particle::Ptr const& p)
{
    auto force = p->getPosition();
    force -= mOther->getPosition();

    auto const magnitude = mSpringConstant * fabs(force.magnitude() - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegasus::ParticleAnchoredSpring::ParticleAnchoredSpring(
    Vector3 const& anchor, real const springConstant,
    real const restLenght)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegasus::ParticleAnchoredSpring::updateForce(Particle::Ptr const& p)
{
    auto force = p->getPosition();
    force -= mAnchor;

    auto const magnitude = mSpringConstant * fabs(force.magnitude() - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegasus::ParticleBungee::ParticleBungee(Particle::Ptr const& other,
    real const springConstant,
    real const restLenght)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegasus::ParticleBungee::updateForce(Particle::Ptr const& p)
{
    auto force = p->getPosition();
    force -= mOther->getPosition();

    auto magnitude = force.magnitude();
    if (magnitude <= mRestLenght)
        return;

    magnitude = mSpringConstant * (magnitude - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegasus::ParticleBuoyancy::ParticleBuoyancy(real const maxDepth,
    real const volume,
    real const waterWight,
    real const liquidDensity)
    : mMaxDepth(maxDepth)
    , mVolume(volume)
    , mWaterHeight(waterWight)
    , mLiquidDensity(liquidDensity)
{
}

void pegasus::ParticleBuoyancy::updateForce(Particle::Ptr const& p)
{
    auto const depth = p->getPosition().y;

    if (depth >= mWaterHeight + mMaxDepth)
        return;

    Vector3 force;
    if (depth <= mWaterHeight - mMaxDepth) {
        force.y = mLiquidDensity * mVolume;
    } else {
        force.y = mLiquidDensity * mVolume * (depth - mMaxDepth - mWaterHeight) / 2.0f * mMaxDepth;
    }

    p->addForce(force);
}

pegasus::ParticleFakeSpring::ParticleFakeSpring(Vector3 const& anchor,
    real const springConstant,
    real const damping)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mDamping(damping)
    , mDuration(0)
{
}

void pegasus::ParticleFakeSpring::updateForce(Particle::Ptr const& p,
    real const duration) const
{
    if (!p->hasFiniteMass())
        return;

    auto const position = p->getPosition() - mAnchor;
    auto const gamma = 0.5f * sqrt(4 * mSpringConstant - mDamping * mDamping);

    if (gamma == 0.0f)
        return;

    auto const c = position * (mDamping / (2.0f * gamma)) + p->getVelocity() * (1.0f / gamma);
    auto target = position * cos(gamma * duration) + c * sin(gamma * duration);
    target *= exp(-0.5f * duration * mDamping);

    auto const accel = (target - position) * (1.0f / duration * duration) - p->getVelocity() * duration;
    p->addForce(accel * p->getMass());
}

void pegasus::ParticleFakeSpring::updateForce(Particle::Ptr const& p)
{
    updateForce(p, mDuration);
}

pegasus::BlobForceGenerator::BlobForceGenerator(std::vector<Particle::Ptr>& particles)
    : particles(particles)
    , maxReplusion(0)
    , maxAttraction(0)
    , minNaturalDistance(0)
    , maxNaturalDistance(0)
    , floatHead(0)
    , maxFloat(0)
    , maxDistance(0)
{
}

void pegasus::BlobForceGenerator::updateForce(const Particle::Ptr& particle)
{
    unsigned joinCount = 0;
    for (unsigned i = 0; i < particles.size(); ++i) {
        if (particles[i] == particle)
            continue;

        // Work out the separation distance
        auto separation = particles[i]->getPosition() - particle->getPosition();
        separation.z = 0.0f;
        auto distance = separation.magnitude();

        if (distance < minNaturalDistance) {
            // Use a repulsion force.
            distance = 1.0f - distance / minNaturalDistance;
            particle->addForce(separation.unit() * (1.0f - distance) * maxReplusion * -1.0f);
            joinCount++;
        } else if (distance > maxNaturalDistance && distance < maxDistance) {
            // Use an attraction force.
            distance = (distance - maxNaturalDistance) / (maxDistance - maxNaturalDistance);
            particle->addForce(separation.unit() * distance * maxAttraction);
            joinCount++;
        }
    }

    // If the particle is the head, and we've got a join count, then float it.
    if (particle == particles.front() && joinCount > 0 && maxFloat > 0) {
        auto force = real(joinCount / maxFloat) * floatHead;
        if (force > floatHead)
            force = floatHead;
        particle->addForce(Vector3(0, force, 0));
    }
}
