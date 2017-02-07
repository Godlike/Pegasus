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

        ParticleContact(Particle::Ptr const & mA, Particle::Ptr const & mB, real const mRestitution, Vector3 const & mContactNormal);

        void resolve(real const duration);

        real calculateSeparatingVelocity() const;

    private:
        Particle::Ptr const mA;
        Particle::Ptr const mB;
        real const mRestitution;
        Vector3 const mContactNormal;
        real mPenetration;

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
}//namespace pegas

#endif //PEGAS_PARTICLE_CONTACTS_HPP
