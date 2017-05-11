#include "Pegasus/include/particle.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

pegasus::Particle::Particle()
    : mDamping(1)
    , mMass(1)
    , mInverseMass(1)
{
}

void pegasus::Particle::integrate(pegasus::real const duration)
{
    if (!hasFiniteMass()) {
        return;
    }

    if (duration <= 0.0f) {
        throw std::invalid_argument("Particle::integrate duration <= 0");
    }

    mPosition.addScaledVector(mVelocity, duration);

    Vector3 resultingAcc(mAcceleration);
    resultingAcc.addScaledVector(mForceAccum, mInverseMass);

    mVelocity.addScaledVector(resultingAcc, duration);

    mVelocity *= std::pow(mDamping, duration);

    clearForceAccum();
}

pegasus::Vector3 pegasus::Particle::getPosition() const { return mPosition; }

void pegasus::Particle::setPosition(pegasus::Vector3 const& position)
{
    mPosition = position;
}

void pegasus::Particle::setPosition(pegasus::real const x, pegasus::real const y,
    pegasus::real const z)
{
    mPosition = Vector3(x, y, z);
}

pegasus::Vector3 pegasus::Particle::getVelocity() const { return mVelocity; }

void pegasus::Particle::setVelocity(pegasus::Vector3 const& velocity)
{
    mVelocity = velocity;
}

void pegasus::Particle::setVelocity(pegasus::real const x, pegasus::real const y,
    pegasus::real const z)
{
    mVelocity = Vector3(x, y, z);
}

pegasus::Vector3 pegasus::Particle::getAcceleration() const
{
    return mAcceleration;
}

void pegasus::Particle::setAcceleration(pegasus::Vector3 const& acceleration)
{
    mAcceleration = acceleration;
}

void pegasus::Particle::setAcceleration(pegasus::real const x, pegasus::real const y,
    pegasus::real const z)
{
    mAcceleration = Vector3(x, y, z);
}

pegasus::real pegasus::Particle::getDamping() const { return mDamping; }

void pegasus::Particle::setDamping(pegasus::real const damping)
{
    mDamping = damping;
}

pegasus::real pegasus::Particle::getMass() const
{
    return (mInverseMass == 0) ? std::numeric_limits<real>::max() : mMass;
}

void pegasus::Particle::setMass(pegasus::real const mass)
{
    if (mass <= 0) {
        throw std::invalid_argument("Particle::setMass mass <= 0");
    }

    mMass = mass;
    mInverseMass = real(1) / mass;
}

bool pegasus::Particle::hasFiniteMass() const { return mInverseMass != 0; }

pegasus::real pegasus::Particle::getInverseMass() const { return mInverseMass; }

void pegasus::Particle::setInverseMass(pegasus::real const inverseMass)
{
    mInverseMass = inverseMass;
}

void pegasus::Particle::addForce(pegasus::Vector3 const& force)
{
    mForceAccum += force;
}

void pegasus::Particle::clearForceAccum() { mForceAccum = Vector3(); }
