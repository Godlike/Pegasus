#ifndef PARTICLE_LINKS_HPP
#define PARTICLE_LINKS_HPP

#include "Pegas/include/particle.hpp"
#include "Pegas/include/particlecontacts.hpp"

namespace pegas {
class ParticleLink {
public:
  virtual ~ParticleLink();

  virtual unsigned int fillContact(ParticleContact::Ptr &contact,
                                   unsigned int const limit) const = 0;

  real currentLenght() const;

  Particle::Ptr a;
  Particle::Ptr b;
};

class ParticleCabel : public ParticleLink {
public:
  real maxLength;
  real restitution;

  virtual unsigned int fillContact(ParticleContact::Ptr &contact,
                                   unsigned int const limit) const override;
};

class ParticleRod : public ParticleLink {
public:
  real length;

  virtual unsigned int fillContact(ParticleContact::Ptr &contact,
                                   unsigned int const limit) const override;
};

} // namespace pegas

#endif // PARTICLE_LINKS_HPP
