#ifndef PEGAS_PARTICLE_CONTACTS_HPP
#define PEGAS_PARTICLE_CONTACTS_HPP

#include "Pegas/include/particle.hpp"
#include <vector>
#include <algorithm>

namespace pegas
{
    class ParticleContact
    {
    public:
        using Ptr = std::shared_ptr<ParticleContact>;

        ParticleContact(Particle::Ptr const & a, Particle::Ptr const & b, real const restitution, Vector3 const & contactNormal);

        void resolve(real const duration);

        real calculateSeparatingVelocity() const;

    private:
        Particle::Ptr const a;
        Particle::Ptr const b;
        real const restitution;
        Vector3 const contactNormal;
        real penetration;

        void resolveVelocity(real const duration);

        void resolveInterpenetration(real const duration);
    };

    using ParticleContactsArray = std::vector<ParticleContact::Ptr>;

    class ParticleContactResolver
    {
    public:
        ParticleContactResolver(unsigned int const iterations);

        void setIterations(unsigned int const iterations);

        void resolveContacts(ParticleContactsArray const & contacts, real const duration);

    private:
        unsigned int mIterations;
        unsigned int mIterationsUsed;
    };
}

#endif //PEGAS_PARTICLE_CONTACTS_HPP
