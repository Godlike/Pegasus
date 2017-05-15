#include "Pegasus/include/particlelinks.hpp"

double pegasus::ParticleLink::currentLenght() const
{
    auto const relativePos = mA->getPosition() - mB->getPosition();
    return relativePos.magnitude();
}

pegasus::ParticleCabel::ParticleCabel(
    Particle::Ptr a,
    Particle::Ptr b,
    double const maxLength,
    double const restutuition)
    : ParticleLink(a, b)
    , maxLength(maxLength)
    , restitution(restutuition)
{
}

unsigned int
pegasus::ParticleCabel::addContact(ParticleContacts& contacts,
    unsigned int const limit) const
{
    auto const length = currentLenght();

    if (length < maxLength) {
        return 0;
    }

    auto normal = (mB->getPosition() - mA->getPosition());
    normal.normalize();

    contacts.emplace_back(mA, mB, restitution, normal, length - maxLength);

    return 1;
}

pegasus::ParticleRod::ParticleRod(Particle::Ptr a, Particle::Ptr b, double const length)
    : ParticleLink(a, b)
    , length(length)
{
}

unsigned int
pegasus::ParticleRod::addContact(ParticleContacts& contacts,
    unsigned int const limit) const
{
    auto const currentLen = currentLenght();

    if (currentLen == length) {
        return 0;
    }

    auto normal = (mB->getPosition() - mA->getPosition());
    normal.normalize();

    contacts.emplace_back(mA, mB, double(0),
        (currentLen > length ? normal : normal * -1), 
        (currentLen > length ? currentLen - length : length - currentLen)
    );

    return 1;
}
