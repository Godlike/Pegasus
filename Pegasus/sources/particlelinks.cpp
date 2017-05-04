#include "Pegasus/include/particlelinks.hpp"

pegasus::real pegasus::ParticleLink::currentLenght() const
{
    auto const relativePos = mA->getPosition() - mB->getPosition();
    return relativePos.magnitude();
}

pegasus::ParticleCabel::ParticleCabel(
    pegasus::Particle::Ptr& a,
    pegasus::Particle::Ptr& b,
    pegasus::real const maxLength,
    pegasus::real const restutuition)
    : ParticleLink(a, b)
    , maxLength(maxLength)
    , restitution(restutuition)
{
}

unsigned int
pegasus::ParticleCabel::addContact(Contacts& contacts,
    unsigned int const limit) const
{
    auto const length = currentLenght();

    if (length < maxLength) {
        return 0;
    }

    auto normal = (mB->getPosition() - mA->getPosition());
    normal.normalize();

    contacts.push_back(std::make_shared<ParticleContact>(mA, mB, restitution, normal,
        length - maxLength));

    return 1;
}

pegasus::ParticleRod::ParticleRod(pegasus::Particle::Ptr& a, pegasus::Particle::Ptr& b, pegasus::real const length)
    : ParticleLink(a, b)
    , length(length)
{
}

unsigned int
pegasus::ParticleRod::addContact(Contacts& contacts,
    unsigned int const limit) const
{
    auto const currentLen = currentLenght();

    if (currentLen == length) {
        return 0;
    }

    auto normal = (mB->getPosition() - mA->getPosition());
    normal.normalize();

    contacts.push_back(std::make_shared<ParticleContact>(
        mA, mB, static_cast<real>(0), (currentLen > length ? normal : normal * -1),
        (currentLen > length ? currentLen - length : length - currentLen)));

    return 1;
}
