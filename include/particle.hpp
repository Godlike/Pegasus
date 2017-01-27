#ifndef PEGAS_PARTICLE_HPP
#define PEGAS_PARTICLE_HPP

#include "Pegas/include/core.hpp"
#include "Pegas/include/math.hpp"

namespace pegas {

class Particle
{
public:
    void integrate(const real duration);

    Vector3 getPosition() const;

    void setPosition(const Vector3 & position);

    void setPosition(const real x, const real y, const real z);

    Vector3 getVelocity() const;

    void setVelocity(const Vector3 & velocity);

    void setVelocity(const real x, const real y, const real z);

    Vector3 getAcceleration() const;

    void setAcceleration(const Vector3 & acceleration);

    void setAcceleration(const real x, const real y, const real z);

    real getDamping() const;

    void setDamping(const real damping);

    real getMass() const;

    void setMass(const real mass);

    bool hasFiniteMass() const;

    real getInverseMass() const;

    void setInverseMass(const real inverseMass);

    void addForce(const Vector3 & force);

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

} //namespace pegas

#endif //PEGAS_PARTICLE_HPP
