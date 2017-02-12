#include "Pegas/include/particlelinks.hpp"

pegas::ParticleLink::~ParticleLink() {}

pegas::real pegas::ParticleLink::currentLenght() const {
  Vector3 const relativePos = a->getPosition() - b->getPosition();
  return relativePos.magnitude();
}

unsigned int
pegas::ParticleCabel::fillContact(pegas::ParticleContact::Ptr &contact,
                                  unsigned int const limit) const {
  if (!contact) {
    throw std::invalid_argument("ParticleCabel::fillContact !contact");
  }

  real const length = currentLenght();

  if (length < maxLength) {
    return 0;
  }

  Vector3 normal = (b->getPosition() - b->getPosition());
  normal.normalize();

  contact = std::make_shared<ParticleContact>(a, b, restitution, normal,
                                              length - maxLength);

  return 1;
}

unsigned int
pegas::ParticleRod::fillContact(pegas::ParticleContact::Ptr &contact,
                                unsigned int const limit) const {
  if (!contact) {
    throw std::invalid_argument("ParticleRod::fillContact !contact");
  }

  real const currentLen = currentLenght();

  if (currentLen == length) {
    return 0;
  }

  Vector3 normal = (b->getPosition() - b->getPosition());
  normal.normalize();

  contact = std::make_shared<ParticleContact>(
      a, b, 0, (currentLen > length ? normal : normal * -1),
      (currentLen > length ? currentLen - length : length - currentLen));

  return 1;
}
