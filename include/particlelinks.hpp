#ifndef PARTICLE_LINKS_HPP
#define PARTICLE_LINKS_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"

namespace pegas {
class ParticleLink : public ParticleContactGenerator {
public:
    using Ptr = std::shared_ptr<ParticleLink>;

    ParticleLink(Particle::Ptr& a, Particle::Ptr& b)
        : mA(a)
        , mB(b)
    {
        if (!mA || !mB) {
            throw std::invalid_argument("ParticleLink::ParticleLink !mA || !mB");
        }
    }

    virtual unsigned int addContact(Contacts& contacts,
        unsigned int const limit) const override = 0;

    real currentLenght() const;

protected:
    Particle::Ptr& mA;
    Particle::Ptr& mB;
};

class ParticleCabel : public ParticleLink {
public:
    using Ptr = std::shared_ptr<ParticleCabel>;

    ParticleCabel(
        Particle::Ptr& a, Particle::Ptr& b, real const maxLength, real const restutuition);

    virtual unsigned int addContact(Contacts& contacts,
        unsigned int const limit) const override;

private:
    real maxLength;
    real restitution;
};

class ParticleRod : public ParticleLink {
public:
    using Ptr = std::shared_ptr<ParticleRod>;

    ParticleRod(Particle::Ptr& a, Particle::Ptr& b, real const length);

    virtual unsigned int addContact(Contacts& contacts,
        unsigned int const limit) const override;

private:
    real length;
};

} // namespace pegas

#endif // PARTICLE_LINKS_HPP
