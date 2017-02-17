#include "Pegas/include/particlecontacts.hpp"

#include <algorithm>

pegas::ParticleContact::ParticleContact(pegas::Particle::Ptr const& a,
    pegas::Particle::Ptr const& b,
    pegas::real const restitution,
    pegas::Vector3 const& contactNormal,
    pegas::real const penetration)
    : mA(a)
    , mB(b)
    , mRestitution(restitution)
    , mContactNormal(contactNormal)
    , mPenetration(penetration)
{
    if (!mA && !mB) {
        throw std::invalid_argument("ParticleContact::ParticleContact !mA && !mB");
    }
}

void pegas::ParticleContact::resolve(pegas::real const duration)
{
    if (duration < 0) {
        throw std::invalid_argument("ParticleContact::resolve duration < 0");
    }

    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

pegas::real pegas::ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = mA->getVelocity();
    if (mB) {
        relativeVelocity -= mB->getVelocity();
    }

    return relativeVelocity * mContactNormal;
}

void pegas::ParticleContact::resolveVelocity(pegas::real const duration)
{
    real const separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0) {
        return;
    }

    real newSepVelocity = -separatingVelocity * mRestitution;
    Vector3 accCausedVelocity = mA->getAcceleration();
    if (mB) {
        accCausedVelocity -= mB->getAcceleration();
    }
    real const accCausedSepVelocity = accCausedVelocity * mContactNormal * duration;
    if (accCausedSepVelocity < 0) {
        newSepVelocity += mRestitution * accCausedSepVelocity;

        if (newSepVelocity < 0) {
            newSepVelocity = 0;
        }
    }
    real const deltaVelocity = newSepVelocity - separatingVelocity;

    real totalInverseMass = mA->getInverseMass();
    if (mB) {
        totalInverseMass += mB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    real const impulse = deltaVelocity / totalInverseMass;
    Vector3 const impulsePerIMass = mContactNormal * impulse;

    mA->setVelocity(mA->getVelocity() + impulsePerIMass * mA->getInverseMass());
    if (mB) {
        mB->setVelocity(mB->getVelocity() + impulsePerIMass * -mB->getInverseMass());
    }
}

void pegas::ParticleContact::resolveInterpenetration(
    pegas::real const duration)
{
    if (mPenetration <= 0) {
        return;
    }

    real totalInverseMass = mA->getInverseMass();
    if (mB) {
        totalInverseMass += mB->getInverseMass();
    }

    if (totalInverseMass <= 0) {
        return;
    }

    Vector3 const movePerIMass = mContactNormal * (mPenetration / totalInverseMass);
    mA->setPosition(mA->getPosition() + movePerIMass * mA->getInverseMass());
    if (mB) {
        mB->setPosition(mB->getPosition() + movePerIMass * mB->getInverseMass());
    }
}

pegas::ParticleContactResolver::ParticleContactResolver(
    unsigned int const iterations)
    : mIterations(iterations)
{
}

void pegas::ParticleContactResolver::setIterations(
    unsigned int const iterations)
{
    mIterations = iterations;
}

void pegas::ParticleContactResolver::resolveContacts(
    pegas::ParticleContactsArray& contacts, pegas::real const duration)
{
    mIterationsUsed = 0;

    std::sort(contacts.begin(), contacts.end(),
        [](ParticleContact::Ptr const& a, ParticleContact::Ptr const& b) {
            return a->calculateSeparatingVelocity() < b->calculateSeparatingVelocity();
        });

    while (mIterationsUsed++ < mIterations && !contacts.empty()) {
        auto maxSepVelocityContact = contacts.back();
        contacts.pop_back();
        maxSepVelocityContact->resolve(duration);
    }
}

pegas::ParticleContactGenerator::~ParticleContactGenerator() {}
