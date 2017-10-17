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
    , damping(1)
{
}

LinearMotion::LinearMotion()
    : position(0, 0, 0)
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , force(0, 0, 0)
{
}

StaticBody::StaticBody()
    : material()
{
}

DynamicBody::DynamicBody()
    : StaticBody()
    , linearMotion()
{
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

glm::dvec3 IntegrateAcceleration(glm::dvec3 acceleration, glm::dvec3 force, double mass)
{
    return acceleration + force / mass;
}

glm::dvec3 IntegrateVelocity(glm::dvec3 velocity, glm::dvec3 acceleration, double duration)
{
    return velocity + acceleration * duration;
}

glm::dvec3 IntegrateDamping(glm::dvec3 velocity, double damping, double duration)
{
    return velocity * glm::pow(damping, duration);
}

void Integrate(Material& material, LinearMotion& linearMotion, double duration)
{
    glm::dvec3 const resultingAcceleration = ::IntegrateAcceleration(linearMotion.acceleration, linearMotion.force, material.mass);
    linearMotion.position = ::IntegratePosition(linearMotion.position, linearMotion.velocity, duration);
    linearMotion.velocity = ::IntegrateVelocity(linearMotion.velocity, resultingAcceleration, duration);
    linearMotion.velocity = ::IntegrateDamping(linearMotion.velocity, material.damping, duration);
    linearMotion.force = glm::dvec3(0);
}
} // namespace ::

void integration::Integrate(DynamicBody& body, double duration)
{   
    ::Integrate(body.material, body.linearMotion, duration);
}

void integration::Integrate(KinematicBody& body, double duration)
{
    ::Integrate(body.material, body.linearMotion, duration);
}
