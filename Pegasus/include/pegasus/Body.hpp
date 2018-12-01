/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BODY_HPP
#define PEGASUS_BODY_HPP

#include <pegasus/Material.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/optimum_pow.hpp>

namespace pegasus
{
namespace mechanics
{
/**
 * @brief Represents a point mass physical body
 */
struct Body
{
    /**
     * @brief Stores linear motion data
     */
    struct LinearMotion
    {
        LinearMotion();

        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 acceleration;
        glm::vec3 force;
    };

    /**
     * @brief Stores angular motion data
     */
    struct AngularMotion
    {
        AngularMotion();

        glm::quat orientation;
        glm::vec3 velocity;
        glm::vec3 acceleration;
        glm::vec3 torque;
    };

    Body();

    Material material;
    LinearMotion linearMotion;
    AngularMotion angularMotion;
};

/**
 * @brief  Calculates moment of inertia for the given sphere
 * @param  radius sphere's radius
 * @param  mass   sphere's mass
 * @return 3d moment of inertia
 */
inline glm::mat3 CalculateSolidSphereMomentOfInertia(float radius, float mass)
{
    float const factor = 2.0f / 5.0f;
    float const rSq = glm::pow2(radius);
    float const inertia = (factor * mass * rSq);

    return glm::mat3{
        inertia, 0, 0,
        0, inertia, 0,
        0, 0, inertia,
    };
}

/**
 * @brief  Calculates 3d moment of inertia for the give solid cuboid
 * @param  width  cuboid's width
 * @param  height cuboid's height
 * @param  depth  cuboid's depth
 * @param  mass   cuboid's mass
 * @return 3d moment of inertia
 */
inline glm::mat3 CalculateSolidCuboidMomentOfInertia(float width, float height, float depth, float mass)
{
    float const massFraction = 1.0f / 12.0f * mass;
    float const widthSq  = glm::pow2(width);
    float const heightSq = glm::pow2(height);
    float const depthSq  = glm::pow2(depth);

    return glm::mat3{
        (massFraction * (heightSq + depthSq)), 0, 0,
        0, (massFraction * (widthSq + depthSq)),  0,
        0, 0, (massFraction * (widthSq + heightSq)),
    };
}

} // namespace mechanics
} // namespace pegasus
#endif // PEGASUS_BODY_HPP
