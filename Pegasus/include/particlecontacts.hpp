#ifndef PEGASUS_PARTICLE_CONTACTS_HPP
#define PEGASUS_PARTICLE_CONTACTS_HPP

#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/mechanics.hpp"
#include "Pegasus/include/particle.hpp"

#include <vector>

namespace pegasus {

class ParticleContact {
public:
    ParticleContact(Particle::Ptr const a, Particle::Ptr const b,
        real const restitution, Vector3 const& contactNormal,
        real const penetration);

    void resolve(real const duration) const;
    real calculateSeparatingVelocity() const;

private:
    Particle::Ptr mA;
    Particle::Ptr mB;
    real mRestitution;
    Vector3 mContactNormal;
    real mPenetration;

private:
    void resolveVelocity(real const duration) const;

    void resolveInterpenetration(real const duration) const;
};

using ParticleContacts = std::vector<ParticleContact>;

class ParticleContactResolver {
public:
    explicit ParticleContactResolver(unsigned int const iterations = 0);

    void setIterations(unsigned int const iterations);

    void resolveContacts(ParticleContacts& contacts,
        real const duration);

private:
    unsigned int mIterations;
    unsigned int mIterationsUsed;
};

class ParticleContactGenerator {
public:
    using Ptr = std::shared_ptr<ParticleContactGenerator>;

public:
    virtual ~ParticleContactGenerator();
    virtual unsigned int addContact(ParticleContacts& contacts, unsigned int const limit) const = 0;
};

class Platform : public ParticleContactGenerator {
public:
    Vector3 start;
    Vector3 end;
    Particles& particles;
    real const blobRadius;

    Platform(Vector3 start, Vector3 end, Particles& particles, real const blobRadius);

    unsigned int addContact(ParticleContacts& contacts, unsigned int const limit) const override;
};

class ShapeContactGenerator : public ParticleContactGenerator {
public:
    ShapeContactGenerator(RigidBody::Ptr const rBody, RigidBodies const& rBodies, real const restitution)
        : mRigidBody(rBody)
        , mRigidBodies(rBodies)
        , mRestitution(restitution)
    {
    }

    unsigned int addContact(ParticleContacts& contacts, unsigned int const limit) const override
    {
        unsigned int used = 0;

        for (auto body : mRigidBodies) {
            if (body == mRigidBody)
                continue;

            if (used++ > limit) {
                break;
            }

            static geometry::IntersectionQuery intersection;
            intersection.initialize(body->s.get(), mRigidBody->s.get());

            if (intersection.overlap(body->s.get(), mRigidBody->s.get())) {
                contacts.emplace_back(
                    body->p, mRigidBody->p, mRestitution,
                    intersection.calculateContactNormal(body->s.get(), mRigidBody->s.get()),
                    intersection.calculatePenetration(body->s.get(), mRigidBody->s.get()));
            }
        }

        return used;
    }

private:
    RigidBody::Ptr const mRigidBody;
    RigidBodies const& mRigidBodies;
    real const mRestitution;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_CONTACTS_HPP
