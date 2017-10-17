/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_WORLD_HPP
#define PEGASUS_PARTICLE_WORLD_HPP

#include <pegasus/Integration.hpp>
#include <pegasus/ParticleContacts.hpp>
#include <pegasus/ParticleForceGenerator.hpp>

#include <pegasus/SharedMacros.hpp>

#include <vector>
#include <list>
#include <memory>

namespace pegasus
{
using ParticleContactGenerators = std::list<std::unique_ptr<ParticleContactGenerator>>;
using Particles = std::list<integration::DynamicBody>;

class ParticleWorld
{
public:
    PEGASUS_EXPORT ParticleWorld(Particles& particles,
        ParticleForceRegistry& forceRegistry,
        ParticleContactGenerators& contactGenerators,
        uint32_t maxContacts,
        uint32_t iterations = 0);

    PEGASUS_EXPORT void StartFrame() const;
    PEGASUS_EXPORT void RunPhysics(double duration);

private:
    Particles& m_particles;
    ParticleForceRegistry& m_forceRegistry;

    ParticleContacts m_contacts;
    ParticleContactGenerators& m_contactGenerators;
    ParticleContactResolver m_contactResolver;

    bool m_calculateIterations;
    uint32_t m_maxContacts;

    uint32_t GenerateContacts();
    void Integrate(double duration) const;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_WORLD_HPP
