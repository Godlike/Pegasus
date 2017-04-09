#include "Pegas/include/particlelinks.hpp"

pegas::real pegas::ParticleLink::currentLenght() const
{
    auto const relativePos = mA->getPosition() - mB->getPosition();
    return relativePos.magnitude();
}

pegas::ParticleCabel::ParticleCabel(
    pegas::Particle::Ptr& a,
    pegas::Particle::Ptr& b,
    pegas::real const maxLength,
    pegas::real const restutuition)
    : ParticleLink(a, b)
    , maxLength(maxLength)
    , restitution(restutuition)
{
}

unsigned int
pegas::ParticleCabel::addContact(Contacts& contacts,
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

pegas::ParticleRod::ParticleRod(pegas::Particle::Ptr& a, pegas::Particle::Ptr& b, pegas::real const length)
    : ParticleLink(a, b)
    , length(length)
{
}

unsigned int
pegas::ParticleRod::addContact(Contacts& contacts,
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
