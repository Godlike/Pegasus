/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>

using namespace pegasus;

namespace
{
glm::dvec3 IntegratePosition(glm::dvec3 position, glm::dvec3 velocity, double duration)
{
    return position + velocity * duration;
}

glm::dvec3 IntegrateAcceleration(glm::dvec3 acceleration, glm::dvec3 force, double inverseMass)
{
    return acceleration + force * inverseMass;
}

glm::dvec3 IntegrateVelocity(glm::dvec3 velocity, glm::dvec3 acceleration, double duration)
{
    return velocity + acceleration * duration;
}

glm::dvec3 IntegrateDamping(glm::dvec3 velocity, double damping, double duration)
{
    return velocity * glm::pow(damping, duration);
}

void IntegrateLinearMotion(mechanics::Material& material, mechanics::LinearMotion& linearMotion, double duration)
{
    glm::dvec3 const resultingAcceleration = ::IntegrateAcceleration(linearMotion.acceleration, linearMotion.force, material.GetInverseMass());
    linearMotion.position = ::IntegratePosition(linearMotion.position, linearMotion.velocity, duration);
    linearMotion.velocity = ::IntegrateVelocity(linearMotion.velocity, resultingAcceleration, duration);
    linearMotion.velocity = ::IntegrateDamping(linearMotion.velocity, material.damping, duration);
    linearMotion.force = glm::dvec3(0);
}
} // namespace ::

glm::dvec3 pegasus::IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce)
{
    return accumulatedForce + appliedForce;
}

void pegasus::Integrate(mechanics::Body& body, double duration)
{
    ::IntegrateLinearMotion(body.material, body.linearMotion, duration);
}
