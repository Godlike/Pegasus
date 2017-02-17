#include "Pegas/include/particleforcegenerator.hpp"

#include <cmath>
#include <stdexcept>

void pegas::ParticleForceRegistry::add(
    pegas::Particle::Ptr& p, pegas::ParticleForceGenerator::Ptr& pfg)
{
    if (!p || !pfg) {
        throw std::invalid_argument("ParticleForceRegistry::add !p || !pfg");
    }

    auto particle = mRegistrations.find(p);
    if (particle != mRegistrations.end()) {
        particle->second.insert(pfg);
    } else {
        mRegistrations.insert(
            std::make_pair(p, std::set<ParticleForceGenerator::Ptr>({ pfg })));
    }
}

void pegas::ParticleForceRegistry::remove(pegas::Particle::Ptr const& p)
{
    if (!p) {
        throw std::invalid_argument("ParticleForceRegistry::remove !p");
    }

    auto entry = mRegistrations.find(p);
    if (entry != mRegistrations.end()) {
        mRegistrations.erase(entry);
    }
}

void pegas::ParticleForceRegistry::remove(
    pegas::Particle::Ptr const& p,
    pegas::ParticleForceGenerator::Ptr const& pfg)
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

void pegas::ParticleForceRegistry::clear() { mRegistrations.clear(); }

void pegas::ParticleForceRegistry::updateForces()
{
    for (auto& entry : mRegistrations) {
        for (auto& force : entry.second) {
            force->updateForce(entry.first);
        }
    }
}

pegas::ParticleGravity::ParticleGravity(pegas::Vector3 const& g)
    : mGravity(g)
{
}

void pegas::ParticleGravity::updateForce(pegas::Particle::Ptr const& p)
{
    if (!p->hasFiniteMass())
        return;

    p->addForce(mGravity * p->getMass());
}

pegas::ParticleDrag::ParticleDrag(pegas::real const k1, pegas::real const k2)
    : mK1(k1)
    , mK2(k2)
{
}

void pegas::ParticleDrag::updateForce(pegas::Particle::Ptr const& p)
{
    Vector3 force = p->getVelocity();

    real dragCoeff = force.magnitude();
    dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    p->addForce(force);
}

pegas::ParticleSpring::ParticleSpring(pegas::Particle::Ptr const& other,
    pegas::real const springConstant,
    pegas::real const restLenght)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegas::ParticleSpring::updateForce(pegas::Particle::Ptr const& p)
{
    Vector3 force = p->getPosition();
    force -= mOther->getPosition();

    real const magnitude = mSpringConstant * std::fabs(force.magnitude() - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegas::ParticleAnchoredSpring::ParticleAnchoredSpring(
    pegas::Vector3 const& anchor, pegas::real const springConstant,
    pegas::real const restLenght)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegas::ParticleAnchoredSpring::updateForce(pegas::Particle::Ptr const& p)
{
    Vector3 force = p->getPosition();
    force -= mAnchor;

    real const magnitude = mSpringConstant * std::fabs(force.magnitude() - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegas::ParticleBungee::ParticleBungee(pegas::Particle::Ptr const& other,
    pegas::real const springConstant,
    pegas::real const restLenght)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLenght(restLenght)
{
}

void pegas::ParticleBungee::updateForce(pegas::Particle::Ptr const& p)
{
    Vector3 force = p->getPosition();
    force -= mOther->getPosition();

    real magnitude = force.magnitude();
    if (magnitude <= mRestLenght)
        return;

    magnitude = mSpringConstant * (magnitude - mRestLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

pegas::ParticleBuoyancy::ParticleBuoyancy(pegas::real const maxDepth,
    pegas::real const volume,
    pegas::real const waterWight,
    pegas::real const liquidDensity)
    : mMaxDepth(maxDepth)
    , mVolume(volume)
    , mWaterHeight(waterWight)
    , mLiquidDensity(liquidDensity)
{
}

void pegas::ParticleBuoyancy::updateForce(pegas::Particle::Ptr const& p)
{
    real const depth = p->getPosition().y;

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

pegas::ParticleFakeSpring::ParticleFakeSpring(pegas::Vector3 const& anchor,
    pegas::real const springConstant,
    pegas::real const damping)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mDamping(damping)
    , mDuration(0)
{
}

void pegas::ParticleFakeSpring::updateForce(pegas::Particle::Ptr const& p,
    pegas::real const duration)
{
    if (!p->hasFiniteMass())
        return;

    Vector3 const position = p->getPosition() - mAnchor;
    real const gamma = 0.5f * std::sqrt(4 * mSpringConstant - mDamping * mDamping);

    if (gamma == 0.0f)
        return;

    Vector3 const c = position * (mDamping / (2.0f * gamma)) + p->getVelocity() * (1.0f / gamma);
    Vector3 target = position * std::cos(gamma * duration) + c * std::sin(gamma * duration);
    target *= std::exp(-0.5f * duration * mDamping);

    Vector3 const accel = (target - position) * (1.0f / duration * duration) - p->getVelocity() * duration;
    p->addForce(accel * p->getMass());
}

void pegas::ParticleFakeSpring::updateForce(pegas::Particle::Ptr const& p)
{
    updateForce(p, mDuration);
}

pegas::BlobForceGenerator::BlobForceGenerator(std::vector<pegas::Particle::Ptr>& particles)
    : particles(particles)
{
}

void pegas::BlobForceGenerator::updateForce(const pegas::Particle::Ptr& particle)
{
    unsigned joinCount = 0;
    for (unsigned i = 0; i < particles.size(); ++i) {
        if (particles[i] == particle)
            continue;

        // Work out the separation distance
        Vector3 separation = particles[i]->getPosition() - particle->getPosition();
        separation.z = 0.0f;
        real distance = separation.magnitude();

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
        real force = real(joinCount / maxFloat) * floatHead;
        if (force > floatHead)
            force = floatHead;
        particle->addForce(Vector3(0, force, 0));
    }
}
