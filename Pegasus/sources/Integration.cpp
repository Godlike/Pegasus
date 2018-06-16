/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>
#include <Epona/FloatingPoint.hpp>
#include <glm/gtx/quaternion.hpp>

namespace
{

/**
* @brief Recalculates position of the body
* @param position current position
* @param velocity current velocity
* @param duration delta time
* @return new position
*/
glm::dvec3 IntegrateLinearPosition(glm::dvec3 position, glm::dvec3 velocity, double duration)
{
    return position + velocity * duration;
}

/**
* @brief Recalculates acceleration of the body
* @param acceleration currant acceleration
* @param force total force acting on the body
* @param inverseMass one divided by mass
* @return new acceleration
*/
glm::dvec3 IntegrateLinearAcceleration(glm::dvec3 acceleration, glm::dvec3 force, double inverseMass)
{
    return acceleration + force * inverseMass;
}

/**
* @brief Recalculates velocity of the body
* @param velocity current velocity
* @param acceleration current acceleration
* @param duration delta time
* @return new velocity
*/
glm::dvec3 IntegrateLinearVelocity(glm::dvec3 velocity, glm::dvec3 acceleration, double duration)
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
glm::dvec3 IntegrateLinearDamping(glm::dvec3 velocity, double damping, double duration)
{
    return velocity * glm::pow(damping, duration);
}

/**
* @brief Updates body position
* @param[in,out] material body material data
* @param[in,out] linearMotion body linear motion data
* @param[in] duration delta time
*/
void IntegrateBody(
        pegasus::mechanics::Material& material,
        pegasus::mechanics::Body::LinearMotion& linearMotion,
        double duration
    )
{
    {
        double const maxSpeed = 100;
        double const speed = glm::length(linearMotion.velocity);
        if (epona::fp::IsGreater(speed, epona::fp::g_floatingPointThreshold))
        {
            linearMotion.velocity = glm::normalize(linearMotion.velocity) * glm::min(speed, maxSpeed);
        }
    }

    glm::dvec3 const resultingAcceleration = ::IntegrateLinearAcceleration(
        linearMotion.acceleration, linearMotion.force, material.GetInverseMass());
    linearMotion.position = ::IntegrateLinearPosition(
        linearMotion.position, linearMotion.velocity, duration);
    linearMotion.velocity = ::IntegrateLinearVelocity(
        linearMotion.velocity, resultingAcceleration, duration);
    linearMotion.velocity = ::IntegrateLinearDamping(
        linearMotion.velocity, material.damping, duration);
    linearMotion.force = glm::dvec3(0);

    {
        if (epona::fp::IsZero(linearMotion.velocity.x))
            linearMotion.velocity.x = 0;
        if (epona::fp::IsZero(linearMotion.velocity.y))
            linearMotion.velocity.y = 0;
        if (epona::fp::IsZero(linearMotion.velocity.z))
            linearMotion.velocity.z = 0;
    }
}

/**
 * @brief Recalculates angular acceleration of the body
 * @param acceleration angular acceleration
 * @param torque torque applied
 * @param inverseMomentOfInertia inverse moment of inertia
 * @return angular acceleration
 */
glm::dvec3 IntegrateAngularAcceleration(glm::dvec3 acceleration, glm::dvec3 torque, glm::dmat3 inverseMomentOfInertia)
{
    return acceleration + inverseMomentOfInertia * torque;
}

/**
 * @brief Recalculates angular displacement of the body
 * @param orientation body orientation quatrenion
 * @param velocity angular velocity
 * @param duration delta time
 * @return angular displacement
 */
glm::dquat IntegrateAngularDisplacement(glm::dquat orientation, glm::dvec3 velocity, double duration)
{
    glm::dquat const velocityQuad{ 0, velocity.x, velocity.y, velocity.z };
    glm::dquat const displacement{ glm::normalize(orientation + duration * velocityQuad * 0.5 * orientation) };

    return displacement;
}

/**
 * @brief Recalculates angular velocity of the body
 * @param velocity angular velocity
 * @param resultingAcceleration total acceleration change
 * @param duration delta time
 * @return angular velocity
 */
glm::dvec3 IntegrateAngularVelocity(glm::dvec3 velocity, glm::dvec3 resultingAcceleration, double duration)
{
    return velocity + resultingAcceleration * duration;
}

/**
 * @brief Calculates angular damping velocity decrease
 * @param velocity angular velocity
 * @param damping damping factor
 * @param duration delta time
 * @return angular velocity
 */
glm::dvec3 IntegrateAngularDamping(glm::dvec3 velocity, double damping, double duration)
{
    return velocity * glm::pow(damping, duration);
}

/**
 * @brief Updates body displacement
 * @param[in, out] material body material data
 * @param[in, out] angularMotion body angular motion data
 * @param[in] duration delta time
 */
void IntegrateBody(
        pegasus::mechanics::Material& material,
        pegasus::mechanics::Body::AngularMotion& angularMotion,
        double duration
    )
{
    {
        double const maxSpeed = 100;
        double const speed = glm::length(angularMotion.velocity);
        if (epona::fp::IsGreater(speed, epona::fp::g_floatingPointThreshold))
        {
            angularMotion.velocity = glm::normalize(angularMotion.velocity) * glm::min(speed, maxSpeed);
        }
    }

    glm::dvec3 const resultingAcceleration = ::IntegrateAngularAcceleration(
        angularMotion.acceleration, angularMotion.torque, material.GetInverseMomentOfInertia());
    angularMotion.orientation = ::IntegrateAngularDisplacement(
        angularMotion.orientation, angularMotion.velocity, duration);
    angularMotion.velocity = ::IntegrateAngularVelocity(
        angularMotion.velocity, resultingAcceleration, duration);
    angularMotion.velocity = ::IntegrateAngularDamping(
        angularMotion.velocity, material.damping, duration);
    angularMotion.torque = glm::dvec3(0, 0, 0);

    {
        if (epona::fp::IsZero(angularMotion.velocity.x))
            angularMotion.velocity.x = 0;
        if (epona::fp::IsZero(angularMotion.velocity.y))
            angularMotion.velocity.y = 0;
        if (epona::fp::IsZero(angularMotion.velocity.z))
            angularMotion.velocity.z = 0;
    }
}
} // namespace ::

namespace pegasus
{
namespace integration
{
void Integrate(mechanics::Body& body, double duration)
{
    ::IntegrateBody(body.material, body.linearMotion, duration);
    ::IntegrateBody(body.material, body.angularMotion, duration);
}
} // namespace integration
} // namespace pegasus
