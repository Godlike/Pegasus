/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_CONTACTS_HPP
#define PEGASUS_PARTICLE_CONTACTS_HPP

#include <pegasus/Geometry.hpp>
#include <pegasus/Mechanics.hpp>
#include <pegasus/Integration.hpp>

#include <pegasus/SharedMacros.hpp>

#include <vector>

namespace pegasus
{
class ParticleContact
{
public:
    PEGASUS_EXPORT ParticleContact(integration::DynamicBody& a, integration::DynamicBody* b,
                    double restitution, glm::dvec3 const& contactNormal, double penetration);
    PEGASUS_EXPORT void Resolve(double duration) const;
    PEGASUS_EXPORT double CalculateSeparatingVelocity() const;

private:
    integration::DynamicBody* m_pBodyA;
    integration::DynamicBody* m_pBodyB;
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
    PEGASUS_EXPORT explicit ParticleContactResolver(uint32_t iterations = 0);
    PEGASUS_EXPORT void SetIterations(uint32_t iterations);
    PEGASUS_EXPORT void ResolveContacts(ParticleContacts& contacts, double duration);

private:
    uint32_t m_iterations;
    uint32_t m_iterationsUsed;
};

class ParticleContactGenerator
{
public:
    PEGASUS_EXPORT virtual ~ParticleContactGenerator() = default;
    PEGASUS_EXPORT virtual uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const = 0;
};

template <typename RigidBodies>
class ShapeContactGenerator : public ParticleContactGenerator
{
public:
    ShapeContactGenerator(RigidBody& rBody, RigidBodies& rBodies, double restitution)
        : rigidBody(rBody)
        , rigidBodies(rBodies)
        , restitution(restitution)
    {
    }

    uint32_t AddContact(ParticleContacts& contacts, uint32_t limit) const override
    {
        uint32_t const contactCount = static_cast<uint32_t>(contacts.size());

        for (RigidBody& body : rigidBodies)
        {
            if (&body == &rigidBody)
            {
                continue;
            }

            static geometry::SimpleShapeIntersectionDetector intersection;

            if (intersection.CalculateIntersection(rigidBody.shape.get(), body.shape.get()))
            {
                glm::dvec3 const contactNormal = intersection.CalculateContactNormal(rigidBody.shape.get(), body.shape.get());
                double const penetration = intersection.CalculatePenetration(rigidBody.shape.get(), body.shape.get());
                contacts.emplace_back(rigidBody.pointMass, &body.pointMass, restitution, contactNormal, penetration);

                if (contacts.size() == limit)
                {
                    break;
                }
            }
        }

        return static_cast<uint32_t>(contacts.size() - contactCount);
    }

    RigidBody& rigidBody;
    RigidBodies& rigidBodies;
    double const restitution;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_CONTACTS_HPP
