/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_HPP
#define PEGASUS_PARTICLE_HPP

#include "Pegasus/include/math.hpp"

namespace pegasus {

class Particle 
{
public:
    Particle();

    void integrate(double duration);

    Vector3 getPosition() const;
    void setPosition(Vector3 const& position);
    void setPosition(double x, double y, double z);

    Vector3 getVelocity() const;
    void setVelocity(Vector3 const& velocity);
    void setVelocity(double x, double y, double z);

    Vector3 getAcceleration() const;
    void setAcceleration(Vector3 const& acceleration);
    void setAcceleration(double x, double y, double z);

    double getDamping() const;
    void setDamping(double damping);

    double getMass() const;
    void setMass(double mass);
    bool hasFiniteMass() const;
    double getInverseMass() const;
    void setInverseMass(double inverseMass);

    void addForce(Vector3 const& force);
    void clearForceAccum();

private:
    Vector3 mPosition;
    Vector3 mVelocity;
    Vector3 mAcceleration;
    double mDamping;
    double mMass;
    double mInverseMass;
    Vector3 mForceAccum;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_HPP
