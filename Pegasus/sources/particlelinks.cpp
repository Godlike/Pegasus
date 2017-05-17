#include "Pegasus/include/particlelinks.hpp"

double pegasus::ParticleLink::currentLenght() const
{
    auto const relativePos = mA.getPosition() - mB.getPosition();
    return relativePos.magnitude();
}

pegasus::ParticleCabel::ParticleCabel(
    Particle & a,
    Particle & b,
    double maxLength,
    double restutuition)
    : ParticleLink(a, b)
    , maxLength(maxLength)
    , restitution(restutuition)
{
}

unsigned int
pegasus::ParticleCabel::addContact(ParticleContacts & contacts, unsigned int limit) const
{
    auto const length = currentLenght();

    if (length < maxLength) {
        return 0;
    }

    Vector3 normal = (mB.getPosition() - mA.getPosition());
    normal.normalize();

    contacts.emplace_back(mA, &mB, restitution, normal, length - maxLength);
    return 1;
}

pegasus::ParticleRod::ParticleRod(Particle & a, Particle & b, double length)
    : ParticleLink(a, b)
    , length(length)
{
}

unsigned int
pegasus::ParticleRod::addContact(ParticleContacts& contacts, unsigned int limit) const
{
    double const currentLen = currentLenght();

    if (currentLen == length) {
        return 0;
    }

    Vector3 normal = (mB.getPosition() - mA.getPosition());
    normal.normalize();

    contacts.emplace_back(mA, &mB, 0.0,
        (currentLen > length ? normal : normal * -1), 
        (currentLen > length ? currentLen - length : length - currentLen)
    );
    return 1;
}
