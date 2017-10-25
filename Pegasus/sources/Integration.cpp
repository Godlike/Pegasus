/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>

using namespace pegasus;
using namespace integration;

namespace
{
/**
 * @brief Recalculates position of the body
 * @param position current position 
 * @param velocity current velocity
 * @param duration delta time
 * @return new position
 */
glm::dvec3 IntegratePosition(glm::dvec3 position, glm::dvec3 velocity, double duration)
{
    return position + velocity * duration;
}

/**
 * @brief Recalculates acceleration of the body
 * @param acceleration currant acceleration
 * @param force total force acting on the body
 * @param inverseMass one devided by mass
 * @return new acceleration
 */
glm::dvec3 IntegrateAcceleration(glm::dvec3 acceleration, glm::dvec3 force, double inverseMass)
{
    return acceleration + force * inverseMass;
}

/**
 * @brief Recalculates veloctity of the body
 * @param velocity current velocity
 * @param acceleration current acceleration
 * @param duration delta time
 * @return new velocity
 */
glm::dvec3 IntegrateVelocity(glm::dvec3 velocity, glm::dvec3 acceleration, double duration)
{
    return velocity + acceleration * duration;
}

/**
 * @brief Calculates damping velocity decrease
 * @param velocity current velocity
 * @param damping damping factor
 * @param duration delta time
 * @return new velocity
 */
glm::dvec3 IntegrateDamping(glm::dvec3 velocity, double damping, double duration)
{
    return velocity * glm::pow(damping, duration);
}

/**
 * @brief Updates body position 
 * @param[in,out] material body material data
 * @param[in,out] linearMotion body linear motion data
 * @param[in] duration delta time
 */
void IntegrateBody(mechanics::Body::Material& material, mechanics::Body::LinearMotion& linearMotion, double duration)
{
    glm::dvec3 const resultingAcceleration = ::IntegrateAcceleration(linearMotion.acceleration, linearMotion.force, material.GetInverseMass());
    linearMotion.position = ::IntegratePosition(linearMotion.position, linearMotion.velocity, duration);
    linearMotion.velocity = ::IntegrateVelocity(linearMotion.velocity, resultingAcceleration, duration);
    linearMotion.velocity = ::IntegrateDamping(linearMotion.velocity, material.damping, duration);
    linearMotion.force = glm::dvec3(0);
}
} // namespace ::

glm::dvec3 integration::IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce)
{
    return accumulatedForce + appliedForce;
}

void integration::Integrate(mechanics::Body& body, double duration)
{
    ::IntegrateBody(body.material, body.linearMotion, duration);
}
