/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>

using namespace pegasus;
using namespace integration;

Material::Material()
    : mass(1)
    , inverseMass(1)
    , damping(1)
{
}

const double Material::s_infiniteMass = 0.0;

LinearMotion::LinearMotion()
    : position(0, 0, 0)
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , force(0, 0, 0)
{
}

Body::Body()
    : material()
    , linearMotion()
{
}

bool integration::FiniteMass(double inverseMass)
{
    return inverseMass != Material::s_infiniteMass;
}

glm::dvec3 integration::IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce)
{
    return accumulatedForce + appliedForce;
}

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
} // namespace ::

void integration::Integrate(Material& material, LinearMotion& linearMotion, double duration)
{
    if (FiniteMass(material.inverseMass))
    {
        glm::dvec3 const resultingAcceleration = ::IntegrateAcceleration(linearMotion.acceleration, linearMotion.force, material.inverseMass);
        linearMotion.position = ::IntegratePosition(linearMotion.position, linearMotion.velocity, duration);
        linearMotion.velocity = ::IntegrateVelocity(linearMotion.velocity, resultingAcceleration, duration);
        linearMotion.velocity = ::IntegrateDamping(linearMotion.velocity, material.damping, duration);
        linearMotion.force = glm::dvec3(0);
    }
}
