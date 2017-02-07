#include "Pegas/include/particlecontacts.hpp"


pegas::ParticleContact::ParticleContact(
        pegas::Particle::Ptr const & a,
        pegas::Particle::Ptr const & b,
        pegas::real const restitution,
        pegas::Vector3 const & contactNormal
    ) : a(a), b(b), restitution(restitution), contactNormal(contactNormal)
{
}

void pegas::ParticleContact::resolve(pegas::real const duration)
{
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

pegas::real pegas::ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = a->getVelocity();
    if (b)
    {
        relativeVelocity -= b->getVelocity();
    }

    return relativeVelocity * contactNormal;
}

void pegas::ParticleContact::resolveVelocity(pegas::real const duration)
{
    real const separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0)
    {
        return;
    }


    real newSepVelocity = -separatingVelocity * restitution;
    Vector3 accCausedVelocity = a->getAcceleration();
    if (b)
    {
        accCausedVelocity -= b->getAcceleration();
    }
    real const accCausedSepVelocity = accCausedVelocity * contactNormal * duration;
    if (accCausedSepVelocity < 0)
    {
        newSepVelocity += restitution * accCausedSepVelocity;

        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }
    real const deltaVelocity = newSepVelocity - separatingVelocity;


    real totalInverseMass = a->getInverseMass();
    if (b)
    {
        totalInverseMass += b->getInverseMass();
    }

    if(totalInverseMass <= 0)
    {
        return;
    }

    real const impulse = deltaVelocity / totalInverseMass;
    Vector3 const impulsePerIMass = contactNormal * impulse;

    a->setVelocity(a->getVelocity() + impulsePerIMass * a->getInverseMass());
    if (b)
    {
        b->setVelocity(b->getVelocity() + impulsePerIMass * -b->getInverseMass());
    }
}

void pegas::ParticleContact::resolveInterpenetration(const pegas::real duration)
{
    if (penetration <= 0)
    {
        return;
    }

    real totalInverseMass = a->getInverseMass();
    if (b)
    {
        totalInverseMass += b->getInverseMass();
    }

    if (totalInverseMass <= 0)
    {
        return;
    }

    Vector3 const movePerIMass = contactNormal * (-penetration / totalInverseMass);
    a->setPosition(a->getPosition() + movePerIMass * a->getInverseMass());
    if (b)
    {
        b->setPosition(b->getPosition() + movePerIMass * b->getInverseMass());
    }
}
