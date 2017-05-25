/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/particleforcegenerator.hpp"

void pegasus::ParticleForceRegistry::add(
    Particle & p, ParticleForceGenerator & pfg)
{
    auto particle = mRegistrations.find(&p);
    if (particle != mRegistrations.end()) {
        particle->second.insert(&pfg);
    } else {
        mRegistrations.insert(
            make_pair(&p, std::set<ParticleForceGenerator*>({ &pfg })));
    }
}

void pegasus::ParticleForceRegistry::remove(Particle & p)
{
    auto entry = mRegistrations.find(&p);
    if (entry != mRegistrations.end()) {
        mRegistrations.erase(entry);
    }
}

void pegasus::ParticleForceRegistry::remove(
    Particle & p, ParticleForceGenerator & pfg)
{
    auto entry = mRegistrations.find(&p);
    if (entry != mRegistrations.end()) {
        auto force = entry->second.find(&pfg);
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
            force->updateForce(*entry.first);
        }
    }
}

pegasus::ParticleGravity::ParticleGravity(Vector3 const& g)
    : mGravity(g)
{
}

void pegasus::ParticleGravity::updateForce(Particle & p)
{
    if (!p.hasFiniteMass()) {
        return;
    }

    p.addForce(mGravity * p.getMass());
}

pegasus::ParticleDrag::ParticleDrag(double k1, double k2)
    : mK1(k1)
    , mK2(k2)
{
}

void pegasus::ParticleDrag::updateForce(Particle & p)
{
    Vector3 force = p.getVelocity();

    double dragCoeff = force.magnitude();
    dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    p.addForce(force);
}

pegasus::ParticleSpring::ParticleSpring(
    Particle & other, double springConstant, double restLength)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLength(restLength)
{
}

void pegasus::ParticleSpring::updateForce(Particle & p)
{
    Vector3 force = p.getPosition();
    force -= mOther.getPosition();

    auto const magnitude = mSpringConstant * std::fabs(force.magnitude() - mRestLength);

    force.normalize();
    force *= -magnitude;
    p.addForce(force);
}

pegasus::ParticleAnchoredSpring::ParticleAnchoredSpring(
    Vector3 const& anchor, double springConstant, double restLength)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mRestLength(restLength)
{
}

void pegasus::ParticleAnchoredSpring::updateForce(Particle & p)
{
    Vector3 force = p.getPosition();
    force -= mAnchor;

    auto const magnitude = mSpringConstant * std::fabs(force.magnitude() - mRestLength);

    force.normalize();
    force *= -magnitude;
    p.addForce(force);
}

pegasus::ParticleBungee::ParticleBungee(Particle & other, double springConstant, double restLength)
    : mOther(other)
    , mSpringConstant(springConstant)
    , mRestLength(restLength)
{
}

void pegasus::ParticleBungee::updateForce(Particle & p)
{
    Vector3 force = p.getPosition();
    force -= mOther.getPosition();

    double magnitude = force.magnitude();
    if (magnitude <= mRestLength) {
        return;
    }

    magnitude = mSpringConstant * (magnitude - mRestLength);

    force.normalize();
    force *= -magnitude;
    p.addForce(force);
}

pegasus::ParticleBuoyancy::ParticleBuoyancy(
    double maxDepth, double volume, double waterWight, double liquidDensity)
    : mMaxDepth(maxDepth)
    , mVolume(volume)
    , mWaterHeight(waterWight)
    , mLiquidDensity(liquidDensity)
{
}

void pegasus::ParticleBuoyancy::updateForce(Particle & p)
{
    double const depth = p.getPosition().y;

    if (depth >= mWaterHeight + mMaxDepth) {
        return;
    }

    Vector3 force;
    if (depth <= mWaterHeight - mMaxDepth) {
        force.y = mLiquidDensity * mVolume;
    } else {
        force.y = mLiquidDensity * mVolume * (depth - mMaxDepth - mWaterHeight) / 2.0f * mMaxDepth;
    }

    p.addForce(force);
}

pegasus::ParticleFakeSpring::ParticleFakeSpring(
    Vector3 const& anchor, double springConstant, double damping)
    : mAnchor(anchor)
    , mSpringConstant(springConstant)
    , mDamping(damping)
    , mDuration(0)
{
}

void pegasus::ParticleFakeSpring::updateForce(Particle & p, double duration) const
{
    if (!p.hasFiniteMass()) {
        return;
    }

    Vector3 const position = p.getPosition() - mAnchor;
    double const gamma = 0.5f * std::sqrt(4 * mSpringConstant - mDamping * mDamping);

    if (gamma == 0.0f)
        return;

    auto const c = position * (mDamping / (2.0f * gamma)) + p.getVelocity() * (1.0f / gamma);
    auto target = position * std::cos(gamma * duration) + c * sin(gamma * duration);
    target *= std::exp(-0.5f * duration * mDamping);

    auto const accel = (target - position) * (1.0f / duration * duration) - p.getVelocity() * duration;
    p.addForce(accel * p.getMass());
}

void pegasus::ParticleFakeSpring::updateForce(Particle & p)
{
    updateForce(p, mDuration);
}
