/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_HPP
#define PEGASUS_PARTICLE_HPP

#include "Pegasus/include/Math.hpp"

namespace pegasus
{
class Particle
{
public:
    Particle();

    void Integrate(double duration);

    Vector3 GetPosition() const;
    void SetPosition(Vector3 const& position);
    void SetPosition(double x, double y, double z);

    Vector3 GetVelocity() const;
    void SetVelocity(Vector3 const& velocity);
    void SetVelocity(double x, double y, double z);

    Vector3 GetAcceleration() const;
    void SetAcceleration(Vector3 const& acceleration);
    void SetAcceleration(double x, double y, double z);

    double GetDamping() const;
    void SetDamping(double damping);

    double GetMass() const;
    void SetMass(double mass);
    bool HasFiniteMass() const;
    double GetInverseMass() const;
    void SetInverseMass(double inverseMass);

    void AddForce(Vector3 const& force);
    void ClearForceAccumulator();

private:
    Vector3 m_position;
    Vector3 m_velocity;
    Vector3 m_acceleration;
    double m_damping;
    double m_mass;
    double m_inverseMass;
    Vector3 m_forceAccumulator;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_HPP
