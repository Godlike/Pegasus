/*
* Implementation file for random number generation.
*
* Part of the Cyclone physics system.
*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_WORLD_HPP
#define PEGASUS_PARTICLE_WORLD_HPP

#include "Pegasus/include/particle.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"

#include <vector>
#include <list>
#include <memory>

namespace pegasus {

using ParticleContactGenerators = std::list<std::unique_ptr<ParticleContactGenerator>>;
using Particles = std::list<Particle>;

class ParticleWorld {
public:
    ParticleWorld(Particles & particles,
                  ParticleForceRegistry & forceRegistry,
                  ParticleContactGenerators & contactGenerators,
                  uint32_t maxContacts,
                  uint32_t iterations = 0);

    void startFrame() const;
    void runPhysics(double duration);

private:
    Particles & mParticles;
    ParticleForceRegistry & mForceRegistry;

    ParticleContacts mContacts;
    ParticleContactGenerators & mContactGenerators;
    ParticleContactResolver mContactResolver;

    bool mCalculateIterations;
    uint32_t mMaxContacts;

private:
    uint32_t generateContacts();
    void integrate(double duration) const;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_WORLD_HPP
