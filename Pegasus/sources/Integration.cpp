/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>
#include <Epona/FloatingPoint.hpp>
#include <glm/gtx/quaternion.hpp>

namespace
{

/**
* @brief Calculates position of the body
* @param position current position
* @param velocity current velocity
* @param duration delta time
* @return new position
*/
glm::vec3 IntegrateLinearPosition(glm::vec3 position, glm::vec3 velocity, float duration)
{
    return position + velocity * duration;
}

/**
* @brief Calculates acceleration of the body
* @param acceleration currant acceleration
* @param force total force acting on the body
* @param inverseMass one divided by mass
* @return new acceleration
*/
glm::vec3 IntegrateLinearAcceleration(glm::vec3 acceleration, glm::vec3 force, float inverseMass)
{
    return acceleration + force * inverseMass;
}

/**
* @brief Calculates velocity of the body
* @param velocity current velocity
* @param acceleration current acceleration
* @param duration delta time
* @return new velocity
*/
glm::vec3 IntegrateLinearVelocity(glm::vec3 velocity, glm::vec3 acceleration, float duration)
{
    return velocity + acceleration * duration;
}

/**
* @brief Calculates damping velocity decrease
* @param velocity current velocity
* @param damping damping factor
* @param duration delta time
* @param minApllySpeed minimum speed required to apply damping
* @return new velocity
*/
glm::vec3 IntegrateLinearDamping(glm::vec3 velocity, float damping, float duration, float minApllySpeed = 1.f)
{
    return (glm::length2(velocity) > minApllySpeed) ? (velocity * glm::pow(damping, duration)) : velocity;
}

/**
* @brief Updates body position
* @param[in,out] material body material data
* @param[in,out] linearMotion body linear motion data
* @param[in] duration delta time
* @param[in] maxSpeed upper speed bound
*/
void IntegrateBody(
        pegasus::mechanics::Material& material,
        pegasus::mechanics::Body::LinearMotion& linearMotion,
        float duration,
        float maxSpeed = 100
    )
{
    {
        float const speed = glm::length(linearMotion.velocity);
        if (epona::fp::IsGreater(speed, maxSpeed))
        {
            float const minSpeed = glm::min(speed, maxSpeed);
            glm::vec3 const direction = glm::normalize(linearMotion.velocity);
            linearMotion.velocity = direction * minSpeed;
        }
    }

    glm::vec3 const resultingAcceleration = ::IntegrateLinearAcceleration(
        linearMotion.acceleration, linearMotion.force, material.GetInverseMass());
    linearMotion.position = ::IntegrateLinearPosition(
        linearMotion.position, linearMotion.velocity, duration);
    linearMotion.velocity = ::IntegrateLinearVelocity(
        linearMotion.velocity, resultingAcceleration, duration);
    linearMotion.velocity = ::IntegrateLinearDamping(
        linearMotion.velocity, material.damping, duration);
    linearMotion.force = glm::vec3(0);

    {
        if (epona::fp::IsZero(linearMotion.velocity.x))
            linearMotion.velocity.x = 0;
        if (epona::fp::IsZero(linearMotion.velocity.y))
            linearMotion.velocity.y = 0;
        if (epona::fp::IsZero(linearMotion.velocity.z))
            linearMotion.velocity.z = 0;
    }

    assert(!std::isinf(linearMotion.position.x)
        && !std::isinf(linearMotion.position.y)
        && !std::isinf(linearMotion.position.z));
    assert(!std::isinf(linearMotion.velocity.x)
        && !std::isinf(linearMotion.velocity.y)
        && !std::isinf(linearMotion.velocity.z));
}

/**
 * @brief Calculates angular acceleration of the body
 * @param acceleration angular acceleration
 * @param torque torque applied
 * @param inverseMomentOfInertia inverse moment of inertia
 * @return angular acceleration
 */
glm::vec3 IntegrateAngularAcceleration(glm::vec3 acceleration, glm::vec3 torque, glm::mat3 inverseMomentOfInertia)
{
    return acceleration + inverseMomentOfInertia * torque;
}

/**
 * @brief Calculates angular displacement of the body
 * @param orientation body orientation quatrenion
 * @param velocity angular velocity
 * @param duration delta time
 * @return angular displacement
 */
glm::quat IntegrateAngularDisplacement(glm::quat orientation, glm::vec3 velocity, float duration)
{
    glm::quat const velocityQuad{ 0, velocity.x, velocity.y, velocity.z };
    glm::quat const displacement{ glm::normalize(orientation + duration * velocityQuad * 0.5f * orientation) };

    return displacement;
}

/**
 * @brief Calculates angular velocity of the body
 * @param velocity angular velocity
 * @param resultingAcceleration total acceleration change
 * @param duration delta time
 * @return angular velocity
 */
glm::vec3 IntegrateAngularVelocity(glm::vec3 velocity, glm::vec3 resultingAcceleration, float duration)
{
    return velocity + resultingAcceleration * duration;
}

/**
 * @brief Calculates angular damping velocity decrease
 * @param velocity angular velocity
 * @param damping damping factor
 * @param duration delta time
 * @param minApllySpeed minimum speed required to apply damping
 * @return angular velocity
 */
glm::vec3 IntegrateAngularDamping(glm::vec3 velocity, float damping, float duration, float minApllySpeed = 1.f)
{
    return (glm::length2(velocity) > minApllySpeed) ? (velocity * glm::pow(damping, duration)) : velocity;
}

/**
 * @brief Updates body displacement
 * @param[in, out] material body material data
 * @param[in, out] angularMotion body angular motion data
 * @param[in] duration delta time
 * @param[in] maxSpeed upper speed bound
 */
void IntegrateBody(
        pegasus::mechanics::Material& material,
        pegasus::mechanics::Body::AngularMotion& angularMotion,
        float duration,
        float maxSpeed = 100
    )
{
    {
        float const speed = glm::length(angularMotion.velocity);
        if (epona::fp::IsGreater(speed, maxSpeed))
        {
            float const minSpeed = glm::min(speed, maxSpeed);
            glm::vec3 const direction = glm::normalize(angularMotion.velocity);
            angularMotion.velocity = direction * minSpeed;
        }
    }

    glm::vec3 const resultingAcceleration = ::IntegrateAngularAcceleration(
        angularMotion.acceleration, angularMotion.torque, material.GetInverseMomentOfInertia());
    angularMotion.orientation = ::IntegrateAngularDisplacement(angularMotion.orientation, angularMotion.velocity, duration);
    angularMotion.velocity = ::IntegrateAngularVelocity(angularMotion.velocity, resultingAcceleration, duration);
    angularMotion.velocity = ::IntegrateAngularDamping(angularMotion.velocity, material.damping, duration);
    angularMotion.torque = glm::vec3(0, 0, 0);

    {
        if (epona::fp::IsZero(angularMotion.velocity.x))
            angularMotion.velocity.x = 0;
        if (epona::fp::IsZero(angularMotion.velocity.y))
            angularMotion.velocity.y = 0;
        if (epona::fp::IsZero(angularMotion.velocity.z))
            angularMotion.velocity.z = 0;
    }

    assert(!std::isinf(angularMotion.orientation.x)
        && !std::isinf(angularMotion.orientation.y)
        && !std::isinf(angularMotion.orientation.z)
        && !std::isinf(angularMotion.orientation.w));
    assert(!std::isinf(angularMotion.velocity.x)
        && !std::isinf(angularMotion.velocity.y)
        && !std::isinf(angularMotion.velocity.z));
}
} // namespace ::

namespace pegasus
{
namespace integration
{

void Integrate(mechanics::Body& body, float duration)
{
    ::IntegrateBody(body.material, body.linearMotion, duration);
    ::IntegrateBody(body.material, body.angularMotion, duration);
}

} // namespace integration
} // namespace pegasus
