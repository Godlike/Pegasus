/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/particlecontacts.hpp"

pegasus::ParticleContact::ParticleContact(
    Particle & a,
    Particle * b,
    double restitution,
    Vector3 const & contactNormal,
    double penetration)
    : mParticleA(&a)
    , mParticleB(b)
    , mRestitution(restitution)
    , mContactNormal(contactNormal)
    , mPenetration(penetration)
{
}

void pegasus::ParticleContact::resolve(double duration) const
{
    if (duration < 0) {
        return;
    }

    resolveVelocity(duration);
    resolveInterpenetration();
}

double pegasus::ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = mParticleA->getVelocity();
    if (mParticleB) {
        relativeVelocity -= mParticleB->getVelocity();
    }

    return relativeVelocity * mContactNormal;
}

void pegasus::ParticleContact::resolveVelocity(double duration) const
{
    auto const separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0) {
        return;
    }

    auto newSepVelocity = -separatingVelocity * mRestitution;
    auto accCausedVelocity = mParticleA->getAcceleration();
    if (mParticleB) {
        accCausedVelocity -= mParticleB->getAcceleration();
    }
    auto const accCausedSepVelocity = accCausedVelocity * mContactNormal * duration;
    if (accCausedSepVelocity < 0) {
        newSepVelocity += mRestitution * accCausedSepVelocity;

        if (newSepVelocity < 0) {
            newSepVelocity = 0;
        }
    }
    auto const deltaVelocity = newSepVelocity - separatingVelocity;

    auto totalInverseMass = mParticleA->getInverseMass();
    if (mParticleB) {
        totalInverseMass += mParticleB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    auto const impulse = deltaVelocity / totalInverseMass;
    auto const impulsePerIMass = mContactNormal * impulse;

    mParticleA->setVelocity(mParticleA->getVelocity() + impulsePerIMass * mParticleA->getInverseMass());
    if (mParticleB) {
        mParticleB->setVelocity(mParticleB->getVelocity() + impulsePerIMass * -mParticleB->getInverseMass());
    }
}

void pegasus::ParticleContact::resolveInterpenetration() const
{
    if (mPenetration <= 0) {
        return;
    }

    double totalInverseMass = mParticleA->getInverseMass();
    if (mParticleB) {
        totalInverseMass += mParticleB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    auto const movePerIMass = mContactNormal * (mPenetration / totalInverseMass);
    mParticleA->setPosition(mParticleA->getPosition() + movePerIMass * mParticleA->getInverseMass());
    if (mParticleB) {
        mParticleB->setPosition(mParticleB->getPosition() - movePerIMass * mParticleB->getInverseMass());
    }
}

pegasus::ParticleContactResolver::ParticleContactResolver(uint32_t iterations)
    : mIterations(iterations)
    , mIterationsUsed(0)
{
}

void pegasus::ParticleContactResolver::setIterations(uint32_t iterations)
{
    mIterations = iterations;
}

void pegasus::ParticleContactResolver::resolveContacts(ParticleContacts & contacts, double duration)
{
    mIterationsUsed = 0;

    std::sort(contacts.begin(), contacts.end(),
        [](ParticleContact const& a, ParticleContact const& b) {
            return a.calculateSeparatingVelocity() < b.calculateSeparatingVelocity();
        });

    while (mIterationsUsed++ < mIterations && !contacts.empty()) {
        auto maxSepVelocityContact = contacts.back();
        contacts.pop_back();
        maxSepVelocityContact.resolve(duration);
    }
}

pegasus::ParticleContactGenerator::~ParticleContactGenerator() {}
