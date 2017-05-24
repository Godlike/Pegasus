#ifndef PEGASUS_PARTICLE_CONTACTS_HPP
#define PEGASUS_PARTICLE_CONTACTS_HPP

#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/mechanics.hpp"
#include "Pegasus/include/particle.hpp"

#include <vector>

namespace pegasus {

class ParticleContact {
public:
    ParticleContact(Particle & a, Particle * b,
                    double restitution, Vector3 const& contactNormal, double penetration);

    void resolve(double duration) const;
    double calculateSeparatingVelocity() const;

private:
    Particle * mParticleA;
    Particle * mParticleB;
    double  mRestitution;
    Vector3 mContactNormal;
    double  mPenetration;

private:
    void resolveVelocity(double duration) const;
    void resolveInterpenetration() const;
};

using ParticleContacts = std::vector<ParticleContact>;

class ParticleContactResolver {
public:
    explicit ParticleContactResolver(uint32_t iterations = 0);

    void setIterations(uint32_t iterations);
    void resolveContacts(ParticleContacts & contacts, double duration);

private:
    uint32_t mIterations;
    uint32_t mIterationsUsed;
};

class ParticleContactGenerator {
public:
    virtual ~ParticleContactGenerator();
    virtual uint32_t addContact(ParticleContacts & contacts, uint32_t limit) const = 0;
};

template < typename Particles >
class Platform : public ParticleContactGenerator {
public:
    Vector3 const & start;
    Vector3 const & end;
    Particles const & particles;
    double const blobRadius;

public:
    Platform(Vector3 const & start, Vector3 const & end, Particles & particles, double blobRadius)
        : start(start)
        , end(end)
        , particles(particles)
        , blobRadius(blobRadius)
    {
    }

    uint32_t addContact(ParticleContacts& contacts, uint32_t limit) const override
    {
        static auto const restitution = 0.0f;

        uint32_t used = 0;
        for (uint32_t i = 0; i < particles.size(); ++i) {
            if (used >= limit) {
                break;
            }

            auto toParticle = particles[i]->getPosition() - start;
            auto const lineDirection = end - start;
            auto const projected = toParticle * lineDirection;
            auto const platformSqLength = lineDirection.squareMagnitude();

            if (projected <= 0) {
                if (toParticle.squareMagnitude() < blobRadius * blobRadius) {
                    auto contactNormal = toParticle.unit();
                    contactNormal.z = 0;
                    auto const penetration = blobRadius - toParticle.magnitude();
                    contacts.emplace_back(
                        particles[i], nullptr, restitution, contactNormal, penetration);
                    ++used;
                }

            } else if (projected >= platformSqLength) {
                toParticle = particles[i]->getPosition() - end;
                if (toParticle.squareMagnitude() < blobRadius * blobRadius) {
                    auto contactNormal = toParticle.unit();
                    contactNormal.z = 0;
                    auto const penetration = blobRadius - toParticle.magnitude();
                    contacts.emplace_back(
                        particles[i], nullptr, restitution, contactNormal, penetration);
                    ++used;
                }
            } else {
                auto distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
                if (distanceToPlatform < blobRadius * blobRadius) {
                    auto closestPoint = start + lineDirection * (projected / platformSqLength);
                    auto contactNormal = (particles[i]->getPosition() - closestPoint).unit();
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

template < typename RigidBodies >
class ShapeContactGenerator : public ParticleContactGenerator {
public:
    ShapeContactGenerator(RigidBody & rBody, RigidBodies & rBodies, double restitution)
        : mRigidBody(rBody)
        , mRigidBodies(rBodies)
        , mRestitution(restitution)
    {
    }

    uint32_t addContact(ParticleContacts & contacts, uint32_t limit) const override
    {
        uint32_t used = 0;

        for (RigidBody & body : mRigidBodies)
        {
            if (&body == &mRigidBody) {
                continue;
            }

            if (used > limit) {
                break;
            }

            static geometry::IntersectionQuery intersection;
            intersection.initialize(mRigidBody.s.get(), body.s.get());

            if (intersection.overlap(mRigidBody.s.get(), body.s.get()) && ++used)
            {
                Vector3 const contactNormal = intersection.calculateContactNormal(mRigidBody.s.get(), body.s.get());
                double  const penetration   = intersection.calculatePenetration(mRigidBody.s.get(), body.s.get());
                contacts.emplace_back(*mRigidBody.p, body.p, mRestitution, contactNormal, penetration);
            }
        }

        return used;
    }

private:
    RigidBody & mRigidBody;
    RigidBodies & mRigidBodies;
    double const mRestitution;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_CONTACTS_HPP
