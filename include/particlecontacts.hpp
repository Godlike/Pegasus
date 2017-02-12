#ifndef PEGAS_PARTICLE_CONTACTS_HPP
#define PEGAS_PARTICLE_CONTACTS_HPP

#include "Pegas/include/particle.hpp"
#include <vector>

namespace pegas {
class ParticleContact {
public:
  using Ptr = std::shared_ptr<ParticleContact>;

  ParticleContact(Particle::Ptr const &a, Particle::Ptr const &b,
                  real const restitution, Vector3 const &contactNormal,
                  real const penetration);

  void resolve(real const duration);

  real calculateSeparatingVelocity() const;

private:
  Particle::Ptr mA;
  Particle::Ptr mB;
  real mRestitution;
  Vector3 const mContactNormal;
  real mPenetration;

  void resolveVelocity(real const duration);

  void resolveInterpenetration(real const duration);
};

using ParticleContactsArray = std::vector<ParticleContact::Ptr>;

class ParticleContactResolver {
public:
  ParticleContactResolver(unsigned int const iterations = 0);

  void setIterations(unsigned int const iterations);

  void resolveContacts(ParticleContactsArray const &contacts,
                       real const duration);

private:
  unsigned int mIterations;
  unsigned int mIterationsUsed;
};

class ParticleContactGenerator {
public:
  using Ptr = std::shared_ptr<ParticleContactGenerator>;

  virtual ~ParticleContactGenerator();
  virtual unsigned int addContact(ParticleContact::Ptr &contact,
                                  unsigned int const limit) const = 0;
};
} // namespace pegas

#endif // PEGAS_PARTICLE_CONTACTS_HPP
