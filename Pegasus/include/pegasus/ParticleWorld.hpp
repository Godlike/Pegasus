/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_WORLD_HPP
#define PEGASUS_PARTICLE_WORLD_HPP

#include "pegasus/Particle.hpp"
#include "pegasus/ParticleContacts.hpp"
#include "pegasus/ParticleForceGenerator.hpp"

#include <vector>
#include <list>
#include <memory>

namespace pegasus
{
using ParticleContactGenerators = std::list<std::unique_ptr<ParticleContactGenerator>>;
using Particles = std::list<Particle>;

class ParticleWorld
{
public:
    ParticleWorld(Particles& particles,
                  ParticleForceRegistry& forceRegistry,
                  ParticleContactGenerators& contactGenerators,
                  uint32_t maxContacts,
                  uint32_t iterations = 0);

    void StartFrame() const;
    void RunPhysics(double duration);

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
