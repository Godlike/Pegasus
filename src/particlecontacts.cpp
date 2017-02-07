#include "Pegas/include/particlecontacts.hpp"


pegas::ParticleContact::ParticleContact(
        pegas::Particle::Ptr const & a,
        pegas::Particle::Ptr const & b,
        pegas::real const restitution,
        pegas::Vector3 const & contactNormal
    ) : mA(a), mB(b), mRestitution(restitution), mContactNormal(contactNormal)
{
}

void pegas::ParticleContact::resolve(pegas::real const duration)
{
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

pegas::real pegas::ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = mA->getVelocity();
    if (mB)
    {
        relativeVelocity -= mB->getVelocity();
    }

    return relativeVelocity * mContactNormal;
}

void pegas::ParticleContact::resolveVelocity(pegas::real const duration)
{
    real const separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0)
    {
        return;
    }


    real newSepVelocity = -separatingVelocity * mRestitution;
    Vector3 accCausedVelocity = mA->getAcceleration();
    if (mB)
    {
        accCausedVelocity -= mB->getAcceleration();
    }
    real const accCausedSepVelocity = accCausedVelocity * mContactNormal * duration;
    if (accCausedSepVelocity < 0)
    {
        newSepVelocity += mRestitution * accCausedSepVelocity;

        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }
    real const deltaVelocity = newSepVelocity - separatingVelocity;


    real totalInverseMass = mA->getInverseMass();
    if (mB)
    {
        totalInverseMass += mB->getInverseMass();
    }

    if(totalInverseMass <= 0)
    {
        return;
    }

    real const impulse = deltaVelocity / totalInverseMass;
    Vector3 const impulsePerIMass = mContactNormal * impulse;

    mA->setVelocity(mA->getVelocity() + impulsePerIMass * mA->getInverseMass());
    if (mB)
    {
        mB->setVelocity(mB->getVelocity() + impulsePerIMass * -mB->getInverseMass());
    }
}

void pegas::ParticleContact::resolveInterpenetration(pegas::real const duration)
{
    if (mPenetration <= 0)
    {
        return;
    }

    real totalInverseMass = mA->getInverseMass();
    if (mB)
    {
        totalInverseMass += mB->getInverseMass();
    }

    if (totalInverseMass <= 0)
    {
        return;
    }

    Vector3 const movePerIMass = mContactNormal * (-mPenetration / totalInverseMass);
    mA->setPosition(mA->getPosition() + movePerIMass * mA->getInverseMass());
    if (mB)
    {
        mB->setPosition(mB->getPosition() + movePerIMass * mB->getInverseMass());
    }
}

pegas::ParticleContactResolver::ParticleContactResolver(unsigned int const iterations)
    : mIterations(iterations)
{
}

void pegas::ParticleContactResolver::setIterations(unsigned int const iterations)
{
    mIterations = iterations;
}

void pegas::ParticleContactResolver::resolveContacts(pegas::ParticleContactsArray const & contacts, pegas::real const duration)
{
    mIterationsUsed = 0;

    while (mIterationsUsed < mIterations)
    {
        //todo: keep it sorted
        auto max_it = std::max_element(contacts.begin(), contacts.end(),
                                       [](ParticleContact::Ptr const & a, ParticleContact::Ptr const & b){
                return a->calculateSeparatingVelocity() < b->calculateSeparatingVelocity();
        });
        (*max_it)->resolve(duration);
    }
}
