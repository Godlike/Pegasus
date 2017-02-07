#ifndef PEGAS_PARTICLE_CONTACTS_HPP
#define PEGAS_PARTICLE_CONTACTS_HPP

#include "Pegas/include/particle.hpp"

namespace pegas
{
    class ParticleContact
    {
    public:
        ParticleContact(Particle::Ptr const & a, Particle::Ptr const & b, real const restitution, Vector3 const & contactNormal);

        void resolve(real const duration);

    private:
        Particle::Ptr const a;
        Particle::Ptr const b;
        real const restitution;
        Vector3 const contactNormal;
        real penetration;

        real calculateSeparatingVelocity() const;

        void resolveVelocity(real const duration);

        void resolveInterpenetration(real const duration);
    };
}



#endif //PEGAS_PARTICLE_CONTACTS_HPP
