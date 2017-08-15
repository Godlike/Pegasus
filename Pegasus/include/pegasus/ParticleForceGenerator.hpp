/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGASUS_PARTICLE_FORCE_GENERATOR_HPP

#include "pegasus/Particle.hpp"

#include <glm/glm.hpp>

#include <map>
#include <set>

namespace pegasus
{
class ParticleForceGenerator
{
public:
    PEGASUS_EXPORT virtual ~ParticleForceGenerator()
    {
    }

    PEGASUS_EXPORT virtual void UpdateForce(Particle& p) = 0;
};

class ParticleForceRegistry
{
public:
    PEGASUS_EXPORT void Add(Particle& p, ParticleForceGenerator& pfg);
    PEGASUS_EXPORT void Remove(Particle& p);
    PEGASUS_EXPORT void Remove(Particle& p, ParticleForceGenerator& pfg);
    PEGASUS_EXPORT void Clear();
    PEGASUS_EXPORT void UpdateForces();

private:
    std::map<Particle*, std::set<ParticleForceGenerator*>> mRegistrations;
};

class ParticleGravity : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT explicit ParticleGravity(glm::dvec3 const& g);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    glm::dvec3 const m_gravity;
};

class ParticleDrag : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleDrag(double k1, double k2);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    double const m_k1;
    double const m_k2;
};

class ParticleSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleSpring(Particle& other, double springConstant, double restLength);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    Particle& m_other;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleAnchoredSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleAnchoredSpring(glm::dvec3 const& anchor, double springConstant, double restLength);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    glm::dvec3 const m_anchor;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleBungee : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleBungee(Particle& other, double springConstant, double restLength);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    Particle& m_other;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleBuoyancy : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleBuoyancy(double maxDepth, double volume,
        double waterWight, double liquidDensity);
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    double const m_maxDepth;
    double const m_volume;
    double const m_waterHeight;
    double const m_liquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleFakeSpring(glm::dvec3 const& anchor, double springConstant, double damping);
    PEGASUS_EXPORT void UpdateForce(Particle& p, double duration) const;
    PEGASUS_EXPORT void UpdateForce(Particle& p) override;

private:
    glm::dvec3 const m_anchor;
    double const m_springConstant;
    double const m_damping;
    double m_duration;
};

template <typename Particles>
class BlobForceGenerator2D : public ParticleForceGenerator
{
public:
    explicit BlobForceGenerator2D(
        Particles& particles,
        double maxRepulsion = 0,
        double maxAttraction = 0,
        double minNaturalDistance = 0,
        double maxNaturalDistance = 0,
        double floatHead = 0,
        unsigned int maxFloat = 0,
        double maxDistance = 0
    )
        : m_particles(particles)
        , m_maxRepulsion(maxRepulsion)
        , m_maxAttraction(maxAttraction)
        , m_minNaturalDistance(minNaturalDistance)
        , m_maxNaturalDistance(maxNaturalDistance)
        , m_floatHead(floatHead)
        , m_maxFloat(maxFloat)
        , m_maxDistance(maxDistance)
    {
    }

    void UpdateForce(Particle& particle) override
    {
        unsigned int joinCount = 0;

        for (auto const& currentParticle : m_particles)
        {
            if (&currentParticle == &particle)
                continue;

            // Work out the separation distance
            glm::dvec3 separation = currentParticle.GetPosition() - particle.GetPosition();
            separation.z = 0.0f;
            double distance = glm::length(separation);

            if (distance < m_minNaturalDistance)
            {
                // Use a repulsion force.
                distance = 1.0f - distance / m_minNaturalDistance;
                particle.AddForce(glm::normalize(separation) * (1.0f - distance) * m_maxRepulsion * -1.0);
                ++joinCount;
            }
            else if (distance > m_maxNaturalDistance && distance < m_maxDistance)
            {
                // Use an attraction force.
                distance = (distance - m_maxNaturalDistance) / (m_maxDistance - m_maxNaturalDistance);
                particle.AddForce(glm::normalize(separation) * distance * m_maxAttraction);
                ++joinCount;
            }
        }

        // If the particle is the head, and we've got a join count, then float it.
        if (&particle == &(*m_particles.begin())
            && joinCount > 0 && m_maxFloat > 0)
        {
            auto force = (static_cast<double>(joinCount) / static_cast<double>(m_maxFloat)) * m_floatHead;
            if (force > m_floatHead)
            {
                force = m_floatHead;
            }

            particle.AddForce(glm::dvec3(0, force, 0));
        }
    }

private:
    Particles& m_particles;
    double const m_maxRepulsion;
    double const m_maxAttraction;
    double const m_minNaturalDistance;
    double const m_maxNaturalDistance;
    double const m_floatHead;
    uint32_t const m_maxFloat;
    double m_maxDistance;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
