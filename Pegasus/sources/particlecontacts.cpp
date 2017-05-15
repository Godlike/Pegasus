#include "Pegasus/include/particlecontacts.hpp"

#include <algorithm>

pegasus::ParticleContact::ParticleContact(
    Particle::Ptr const a,
    Particle::Ptr const b,
    double const restitution,
    Vector3 const& contactNormal,
    double const penetration)
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

void pegasus::ParticleContact::resolve(double const duration) const
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

void pegasus::ParticleContact::resolveVelocity(double const duration) const
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

void pegasus::ParticleContact::resolveInterpenetration(double const duration) const
{
    if (mPenetration <= 0) {
        return;
    }

    auto totalInverseMass = mA->getInverseMass();
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

pegasus::ParticleContactResolver::ParticleContactResolver(unsigned int const iterations)
    : mIterations(iterations)
    , mIterationsUsed(0)
{
}

void pegasus::ParticleContactResolver::setIterations(
    unsigned int const iterations)
{
    mIterations = iterations;
}

void pegasus::ParticleContactResolver::resolveContacts(
    ParticleContacts& contacts, double const duration)
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

pegasus::Platform::Platform(
    Vector3 start, Vector3 end, Particles& particles, const double blobRadius)
    : start(start)
    , end(end)
    , particles(particles)
    , blobRadius(blobRadius)
{
}

unsigned int pegasus::Platform::addContact(ParticleContacts& contacts, unsigned int limit) const
{
    static auto const restitution = 0.0f;

    unsigned int used = 0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
        if (used >= limit) {
            break;
        }

        auto toParticle = particles[i]->getPosition() - start;
        auto const lineDirection = end - start;
        auto const projected = toParticle * lineDirection;
        auto const platformSqLength = lineDirection.squareMagnitude();

        if (projected <= 0) {
            if (toParticle.squareMagnitude() < blobRadius * blobRadius) {
                auto contactNormal = toParticle.unit();
                contactNormal.z = 0;
                auto const penetration = blobRadius - toParticle.magnitude();
                contacts.emplace_back(
                    particles[i], nullptr, restitution, contactNormal, penetration);
                ++used;
            }

        } else if (projected >= platformSqLength) {
            toParticle = particles[i]->getPosition() - end;
            if (toParticle.squareMagnitude() < blobRadius * blobRadius) {
                auto contactNormal = toParticle.unit();
                contactNormal.z = 0;
                auto const penetration = blobRadius - toParticle.magnitude();
                contacts.emplace_back(
                    particles[i], nullptr, restitution, contactNormal, penetration);
                ++used;
            }
        } else {
            auto distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < blobRadius * blobRadius) {
                auto closestPoint = start + lineDirection * (projected / platformSqLength);
                auto contactNormal = (particles[i]->getPosition() - closestPoint).unit();
                contactNormal.z = 0;
                auto const penetration = blobRadius - sqrt(distanceToPlatform);
                contacts.emplace_back(
                    particles[i], nullptr, restitution, contactNormal, penetration);
                ++used;
            }
        }
    }
    return used;
}
