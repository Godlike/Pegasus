#include "Pegas/include/particleforcegenerator.hpp"
#include <stdexcept>

void pegas::ParticleForceRegistry::add(pegas::Particle::Ptr & p, pegas::ParticleForceGenerator::Ptr & pfg)
{
    if (!p || !pfg)
    {
        throw std::invalid_argument("ParticleForceRegistry::add !p || !pfg");
    }

    auto particle = registrations.find(p);
    if (particle != registrations.end())
    {
        particle->second.insert(pfg);
    }
}

void pegas::ParticleForceRegistry::remove(const pegas::Particle::Ptr & p)
{
    if (!p)
    {
        throw std::invalid_argument("ParticleForceRegistry::remove !p");
    }

    auto entry = registrations.find(p);
    if (entry != registrations.end())
    {
        registrations.erase(entry);
    }
}

void pegas::ParticleForceRegistry::remove(const pegas::Particle::Ptr & p, const pegas::ParticleForceGenerator::Ptr & pfg)
{
    if (!p || !pfg)
    {
        throw std::invalid_argument("ParticleForceRegistry::remove !p || !pfg");
    }

    auto entry = registrations.find(p);
    if (entry != registrations.end())
    {
        auto force = entry->second.find(pfg);
        if (force != entry->second.end())
        {
            entry->second.erase(force);
        }
    }
}

void pegas::ParticleForceRegistry::clear()
{
    registrations.clear();
}

void pegas::ParticleForceRegistry::updateForces()
{
    for (auto & entry : registrations)
    {
        for(auto & force : entry.second)
        {
            force->updateForce(entry.first);
        }
    }

}
