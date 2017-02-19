#ifndef PEGAS_PARTICLE_HPP
#define PEGAS_PARTICLE_HPP

#include "Pegas/include/core.hpp"
#include "Pegas/include/math.hpp"
#include <memory>
#include <vector>

namespace pegas {

class Particle {
public:
    using Ptr = std::shared_ptr<Particle>;
    using ConstPtr = std::shared_ptr<Particle const>;

    void integrate(real const duration);

    Vector3 getPosition() const;

    void setPosition(Vector3 const& position);

    void setPosition(real const x, real const y, real const z);

    Vector3 getVelocity() const;

    void setVelocity(Vector3 const& velocity);

    void setVelocity(real const x, real const y, real const z);

    Vector3 getAcceleration() const;

    void setAcceleration(Vector3 const& acceleration);

    void setAcceleration(real const x, real const y, real const z);

    real getDamping() const;

    void setDamping(real const damping);

    real getMass() const;

    void setMass(real const mass);

    bool hasFiniteMass() const;

    real getInverseMass() const;

    void setInverseMass(real const inverseMass);

    void addForce(Vector3 const& force);

    void clearForceAccum();

private:
    Vector3 mPosition;

    Vector3 mVelocity;

    Vector3 mAcceleration;

    real mDamping;

    real mMass;

    real mInverseMass;

    Vector3 mForceAccum;
};

using Particles = std::vector<Particle::Ptr>;

} // namespace pegas

#endif // PEGAS_PARTICLE_HPP
