#include "Pegasus/include/particlecontacts.hpp"

#include <algorithm>

pegasus::ParticleContact::ParticleContact(
    Particle & a,
    Particle * b,
    double restitution,
    Vector3 const & contactNormal,
    double penetration)
    : mA(&a)
    , mB(b)
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
    resolveInterpenetration(duration);
}

double pegasus::ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = mA->getVelocity();
    if (mB) {
        relativeVelocity -= mB->getVelocity();
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
    auto accCausedVelocity = mA->getAcceleration();
    if (mB) {
        accCausedVelocity -= mB->getAcceleration();
    }
    auto const accCausedSepVelocity = accCausedVelocity * mContactNormal * duration;
    if (accCausedSepVelocity < 0) {
        newSepVelocity += mRestitution * accCausedSepVelocity;

        if (newSepVelocity < 0) {
            newSepVelocity = 0;
        }
    }
    auto const deltaVelocity = newSepVelocity - separatingVelocity;

    auto totalInverseMass = mA->getInverseMass();
    if (mB) {
        totalInverseMass += mB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    auto const impulse = deltaVelocity / totalInverseMass;
    auto const impulsePerIMass = mContactNormal * impulse;

    mA->setVelocity(mA->getVelocity() + impulsePerIMass * mA->getInverseMass());
    if (mB) {
        mB->setVelocity(mB->getVelocity() + impulsePerIMass * -mB->getInverseMass());
    }
}

void pegasus::ParticleContact::resolveInterpenetration(double duration) const
{
    if (mPenetration <= 0) {
        return;
    }

    double totalInverseMass = mA->getInverseMass();
    if (mB) {
        totalInverseMass += mB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    auto const movePerIMass = mContactNormal * (mPenetration / totalInverseMass);
    mA->setPosition(mA->getPosition() + movePerIMass * mA->getInverseMass());
    if (mB) {
        mB->setPosition(mB->getPosition() + movePerIMass * mB->getInverseMass());
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
