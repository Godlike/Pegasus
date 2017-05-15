#ifndef PARTICLE_LINKS_HPP
#define PARTICLE_LINKS_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"

namespace pegasus {
class ParticleLink : public ParticleContactGenerator {
public:
    using Ptr = std::shared_ptr<ParticleLink>;

    ParticleLink(Particle::Ptr a, Particle::Ptr b)
        : mA(a)
        , mB(b)
    {
        if (!mA || !mB) {
            throw std::invalid_argument("ParticleLink::ParticleLink !mA || !mB");
        }
    }

    virtual unsigned int addContact(ParticleContacts& contacts,
        unsigned int const limit) const override = 0;

    double currentLenght() const;

protected:
    Particle::Ptr const mA;
    Particle::Ptr const mB;
};

class ParticleCabel : public ParticleLink {
public:
    using Ptr = std::shared_ptr<ParticleCabel>;

    ParticleCabel(
        Particle::Ptr a, Particle::Ptr b, double const maxLength, double const restutuition);

    virtual unsigned int addContact(ParticleContacts& contacts,
        unsigned int const limit) const override;

private:
    double maxLength;
    double restitution;
};

class ParticleRod : public ParticleLink {
public:
    using Ptr = std::shared_ptr<ParticleRod>;

    ParticleRod(Particle::Ptr a, Particle::Ptr b, double const length);

    virtual unsigned int addContact(ParticleContacts& contacts,
        unsigned int const limit) const override;

private:
    double length;
};

} // namespace pegasus

#endif // PARTICLE_LINKS_HPP
