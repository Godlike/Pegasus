/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_LINKS_HPP
#define PEGASUS_PARTICLE_LINKS_HPP

#include <pegasus/Integration.hpp>
#include <pegasus/ParticleContacts.hpp>

namespace pegasus
{
class ParticleLink : public ParticleContactGenerator
{
public:
    ParticleLink(integration::DynamicBody& a, integration::DynamicBody& b);

    virtual uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override = 0;
    double CurrentLength() const;

protected:
    integration::DynamicBody& m_aParticle;
    integration::DynamicBody& m_bParticle;
};

class ParticleCabel : public ParticleLink
{
public:
    ParticleCabel(integration::DynamicBody& a, integration::DynamicBody& b, double maxLength, double restutuition);

    virtual uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override;

private:
    double const m_maxLength;
    double const m_restitution;
};

class ParticleRod : public ParticleLink
{
public:
    ParticleRod(integration::DynamicBody& a, integration::DynamicBody& b, double length);

    virtual uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override;

private:
    double const m_length;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_LINKS_HPP
