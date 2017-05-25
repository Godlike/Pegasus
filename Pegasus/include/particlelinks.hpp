/*
* Implementation file for particle links.
*
* Part of the Cyclone physics system.
*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_LINKS_HPP
#define PEGASUS_PARTICLE_LINKS_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"

namespace pegasus {

class ParticleLink : public ParticleContactGenerator {
public:
    ParticleLink(Particle& a, Particle& b);

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

#endif // PEGASUS_PARTICLE_LINKS_HPP
