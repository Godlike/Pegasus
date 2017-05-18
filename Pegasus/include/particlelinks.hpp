#ifndef PARTICLE_LINKS_HPP
#define PARTICLE_LINKS_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"

namespace pegasus {

class ParticleLink : public ParticleContactGenerator {
public:
    ParticleLink(Particle & a, Particle & b)
        : mA(a)
        , mB(b)
    {
    }

    virtual uint32_t addContact(ParticleContacts& contacts, uint32_t limit) const override = 0;
    double currentLenght() const;

protected:
    Particle & mA;
    Particle & mB;
};

class ParticleCabel : public ParticleLink {
public:
    ParticleCabel(Particle & a, Particle & b, double maxLength, double restutuition);

    virtual uint32_t addContact(ParticleContacts & contacts, uint32_t limit) const override;

private:
    double const maxLength;
    double const restitution;
};

class ParticleRod : public ParticleLink {
public:
    ParticleRod(Particle & a, Particle & b, double length);

    virtual uint32_t addContact(ParticleContacts& contacts, uint32_t limit) const override;

private:
    double const length;
};

} // namespace pegasus

#endif // PARTICLE_LINKS_HPP
