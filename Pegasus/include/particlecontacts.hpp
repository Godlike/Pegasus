/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_CONTACTS_HPP
#define PEGASUS_PARTICLE_CONTACTS_HPP

#include "Pegasus/include/Geometry.hpp"
#include "Pegasus/include/Mechanics.hpp"
#include "Pegasus/include/Particle.hpp"

#include <vector>

namespace pegasus
{
class ParticleContact
{
public:
    ParticleContact(Particle& a, Particle* b,
                    double restitution, glm::dvec3 const& contactNormal, double penetration);
    void Resolve(double duration) const;
    double CalculateSeparatingVelocity() const;

private:
    Particle* m_pParticleA;
    Particle* m_pParticleB;
    double m_restitution;
    glm::dvec3 m_contactNormal;
    double m_penetration;

    void ResolveVelocity(double duration) const;
    void ResolveInterpenetration() const;
};

using ParticleContacts = std::vector<ParticleContact>;

class ParticleContactResolver
{
public:
    explicit ParticleContactResolver(uint32_t iterations = 0);
    void SetIterations(uint32_t iterations);
    void ResolveContacts(ParticleContacts& contacts, double duration);

private:
    uint32_t m_iterations;
    uint32_t m_iterationsUsed;
};

class ParticleContactGenerator
{
public:
    virtual ~ParticleContactGenerator();
    virtual uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const = 0;
};

template <typename Particles>
class Platform : public ParticleContactGenerator
{
public:
    glm::dvec3 const& start;
    glm::dvec3 const& end;
    Particles const& particles;
    double const blobRadius;

    Platform(glm::dvec3 const& start, glm::dvec3 const& end, Particles& particles, double blobRadius)
        : start(start)
        , end(end)
        , particles(particles)
        , blobRadius(blobRadius)
    {
    }

    uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override
    {
        static auto const restitution = 0.0f;

        uint32_t used = 0;
        for (uint32_t i = 0; i < particles.size(); ++i)
        {
            if (used >= limit)
            {
                break;
            }

            auto toParticle = particles[i]->GetPosition() - start;
            auto const lineDirection = end - start;
            auto const projected = toParticle * lineDirection;
            auto const platformSqLength = lineDirection.SquareMagnitude();

            if (projected <= 0)
            {
                if (toParticle.SquareMagnitude() < blobRadius * blobRadius)
                {
                    auto contactNormal = toParticle.Unit();
                    contactNormal.z = 0;
                    auto const penetration = blobRadius - toParticle.Magnitude();
                    contacts.emplace_back(
                        particles[i], nullptr, restitution, contactNormal, penetration);
                    ++used;
                }
            }
            else if (projected >= platformSqLength)
            {
                toParticle = particles[i]->GetPosition() - end;
                if (toParticle.SquareMagnitude() < blobRadius * blobRadius)
                {
                    auto contactNormal = toParticle.Unit();
                    contactNormal.z = 0;
                    auto const penetration = blobRadius - toParticle.Magnitude();
                    contacts.emplace_back(
                        particles[i], nullptr, restitution, contactNormal, penetration);
                    ++used;
                }
            }
            else
            {
                auto distanceToPlatform = toParticle.SquareMagnitude() - projected * projected / platformSqLength;
                if (distanceToPlatform < blobRadius * blobRadius)
                {
                    auto closestPoint = start + lineDirection * (projected / platformSqLength);
                    auto contactNormal = (particles[i]->GetPosition() - closestPoint).Unit();
                    contactNormal.z = 0;
                    auto const penetration = blobRadius - sqrt(distanceToPlatform);
                    contacts.emplace_back(
                        particles[i], nullptr, restitution, contactNormal, penetration);
                    ++used;
                }
            }
        }
        return used;
    }
};

template <typename RigidBodies>
class ShapeContactGenerator : public ParticleContactGenerator
{
public:
    ShapeContactGenerator(RigidBody& rBody, RigidBodies& rBodies, double restitution)
        : m_rigidBody(rBody)
        , m_rigidBodies(rBodies)
        , m_restitution(restitution)
    {
    }

    uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override
    {
        uint32_t used = 0;

        for (RigidBody& body : m_rigidBodies)
        {
            if (&body == &m_rigidBody)
            {
                continue;
            }

            if (used > limit)
            {
                break;
            }

            static geometry::IntersectionQuery intersection;
            intersection.Initialize(m_rigidBody.s.get(), body.s.get());

            if (intersection.Overlap(m_rigidBody.s.get(), body.s.get()) && ++used)
            {
                glm::dvec3 const contactNormal = intersection.CalculateContactNormal(m_rigidBody.s.get(), body.s.get());
                double const penetration = intersection.CalculatePenetration(m_rigidBody.s.get(), body.s.get());
                contacts.emplace_back(m_rigidBody.p, &body.p, m_restitution, contactNormal, penetration);
            }
        }

        return used;
    }

private:
    RigidBody& m_rigidBody;
    RigidBodies& m_rigidBodies;
    double const m_restitution;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_CONTACTS_HPP
