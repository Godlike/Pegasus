/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_HPP
#define PEGASUS_PARTICLE_HPP

#include <pegasus/SharedMacros.hpp>

#include <glm/glm.hpp>

namespace pegasus
{
class Particle
{
public:
    PEGASUS_EXPORT Particle();

    PEGASUS_EXPORT void Integrate(double duration);

    PEGASUS_EXPORT glm::dvec3 GetPosition() const;
    PEGASUS_EXPORT void SetPosition(glm::dvec3 const& position);
    PEGASUS_EXPORT void SetPosition(double x, double y, double z);

    PEGASUS_EXPORT glm::dvec3 GetVelocity() const;
    PEGASUS_EXPORT void SetVelocity(glm::dvec3 const& velocity);
    PEGASUS_EXPORT void SetVelocity(double x, double y, double z);

    PEGASUS_EXPORT glm::dvec3 GetAcceleration() const;
    PEGASUS_EXPORT void SetAcceleration(glm::dvec3 const& acceleration);
    PEGASUS_EXPORT void SetAcceleration(double x, double y, double z);

    PEGASUS_EXPORT double GetDamping() const;
    PEGASUS_EXPORT void SetDamping(double damping);

    PEGASUS_EXPORT double GetMass() const;
    PEGASUS_EXPORT void SetMass(double mass);
    PEGASUS_EXPORT bool HasFiniteMass() const;
    PEGASUS_EXPORT double GetInverseMass() const;
    PEGASUS_EXPORT void SetInverseMass(double inverseMass);

    PEGASUS_EXPORT void AddForce(glm::dvec3 const& force);
    PEGASUS_EXPORT void ClearForceAccumulator();

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
