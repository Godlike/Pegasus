#include "Pegas/include/particle.hpp"

#include <stdexcept>
#include <limits>
#include <cmath>

void pegas::Particle::integrate(const pegas::real duration)
{
    if (!hasFiniteMass())
    {
        return;
    }

    if (duration <= 0.0f)
    {
        throw std::invalid_argument("Particle::integrate duration <= 0");
    }

    mPosition.addScaledVector(mVelocity, duration);

    Vector3 resultingAcc(mAcceleration);
    resultingAcc.addScaledVector(mForceAccum, mInverseMass);

    mVelocity.addScaledVector(resultingAcc, duration);

    mVelocity *= std::pow(mDamping, duration);

    clearForceAccum();
}

pegas::Vector3 pegas::Particle::getPosition() const
{
    return mPosition;
}

void pegas::Particle::setPosition(const pegas::Vector3 & position)
{
    mPosition = position;
}

void pegas::Particle::setPosition(
        const pegas::real x, const pegas::real y, const pegas::real z)
{
    mPosition = Vector3(x, y, z);
}

pegas::Vector3 pegas::Particle::getVelocity() const
{
    return mVelocity;
}

void pegas::Particle::setVelocity(const pegas::Vector3 & velocity)
{
    mVelocity = velocity;
}

void pegas::Particle::setVelocity(const pegas::real x, const pegas::real y, const pegas::real z)
{
    mVelocity = Vector3(x, y, z);
}

pegas::Vector3 pegas::Particle::getAcceleration() const
{
    return mAcceleration;
}

void pegas::Particle::setAcceleration(const pegas::Vector3 & acceleration)
{
    mAcceleration = acceleration;
}

void pegas::Particle::setAcceleration(const pegas::real x, const pegas::real y, const pegas::real z)
{
    mAcceleration = Vector3(x, y, z);
}

pegas::real pegas::Particle::getDamping() const
{
    return mDamping;
}

void pegas::Particle::setDamping(const pegas::real damping)
{
    mDamping = damping;
}

pegas::real pegas::Particle::getMass() const
{
    return (mInverseMass == 0) ? std::numeric_limits<real>::max() : mMass;
}

void pegas::Particle::setMass(const pegas::real mass)
{
    if (mass <= 0)
    {
        throw std::invalid_argument("Particle::setMass mass <= 0");
    }

    mMass = mass;
    mInverseMass = real(1) / mass;
}

bool pegas::Particle::hasFiniteMass() const
{
    return mInverseMass != 0;
}

pegas::real pegas::Particle::getInverseMass() const
{
    return mInverseMass;
}

void pegas::Particle::setInverseMass(const pegas::real inverseMass)
{
    mInverseMass = inverseMass;
}

void pegas::Particle::addForce(const pegas::Vector3 & force)
{
    mForceAccum += force;
}

void pegas::Particle::clearForceAccum()
{
    mForceAccum = Vector3();
}
