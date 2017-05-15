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

void pegasus::Particle::integrate(double const duration)
{
    if (!hasFiniteMass()) {
        return;
    }

    if (duration < 0) {
        return;
    }

    mPosition.addScaledVector(mVelocity, duration);

    Vector3 resultingAcc(mAcceleration);
    resultingAcc.addScaledVector(mForceAccum, mInverseMass);

    mVelocity.addScaledVector(resultingAcc, duration);

    mVelocity *= pow(mDamping, duration);

    clearForceAccum();
}

pegasus::Vector3 pegasus::Particle::getPosition() const { return mPosition; }

void pegasus::Particle::setPosition(Vector3 const& position)
{
    mPosition = position;
}

void pegasus::Particle::setPosition(double const x, double const y,
    double const z)
{
    mPosition = Vector3(x, y, z);
}

pegasus::Vector3 pegasus::Particle::getVelocity() const { return mVelocity; }

void pegasus::Particle::setVelocity(Vector3 const& velocity)
{
    mVelocity = velocity;
}

void pegasus::Particle::setVelocity(double const x, double const y,
    double const z)
{
    mVelocity = Vector3(x, y, z);
}

pegasus::Vector3 pegasus::Particle::getAcceleration() const
{
    return mAcceleration;
}

void pegasus::Particle::setAcceleration(Vector3 const& acceleration)
{
    mAcceleration = acceleration;
}

void pegasus::Particle::setAcceleration(double const x, double const y,
    double const z)
{
    mAcceleration = Vector3(x, y, z);
}

double pegasus::Particle::getDamping() const { return mDamping; }

void pegasus::Particle::setDamping(double const damping)
{
    mDamping = damping;
}

double pegasus::Particle::getMass() const
{
    return (mInverseMass == 0) ? std::numeric_limits<double>::max() : mMass;
}

void pegasus::Particle::setMass(double const mass)
{
    if (mass <= 0) {
        return;
    }

    mMass = mass;
    mInverseMass = double(1) / mass;
}

bool pegasus::Particle::hasFiniteMass() const { return mInverseMass != 0; }

double pegasus::Particle::getInverseMass() const { return mInverseMass; }

void pegasus::Particle::setInverseMass(double const inverseMass)
{
    mInverseMass = inverseMass;
}

void pegasus::Particle::addForce(Vector3 const& force)
{
    mForceAccum += force;
}

void pegasus::Particle::clearForceAccum() { mForceAccum = Vector3(); }
