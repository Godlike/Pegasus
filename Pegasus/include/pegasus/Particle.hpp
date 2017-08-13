/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_HPP
#define PEGASUS_PARTICLE_HPP

#include <glm/glm.hpp>

namespace pegasus
{
class Particle
{
public:
    Particle();

    void Integrate(double duration);

    glm::dvec3 GetPosition() const;
    void SetPosition(glm::dvec3 const& position);
    void SetPosition(double x, double y, double z);

    glm::dvec3 GetVelocity() const;
    void SetVelocity(glm::dvec3 const& velocity);
    void SetVelocity(double x, double y, double z);

    glm::dvec3 GetAcceleration() const;
    void SetAcceleration(glm::dvec3 const& acceleration);
    void SetAcceleration(double x, double y, double z);

    double GetDamping() const;
    void SetDamping(double damping);

    double GetMass() const;
    void SetMass(double mass);
    bool HasFiniteMass() const;
    double GetInverseMass() const;
    void SetInverseMass(double inverseMass);

    void AddForce(glm::dvec3 const& force);
    void ClearForceAccumulator();

private:
    glm::dvec3 m_position;
    glm::dvec3 m_velocity;
    glm::dvec3 m_acceleration;
    double m_damping;
    double m_mass;
    double m_inverseMass;
    glm::dvec3 m_forceAccumulator;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_HPP
