/*
* Copyright (C) 2018 by Godlike
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

        glm::dvec3 position;
        glm::dvec3 velocity;
        glm::dvec3 acceleration;
        glm::dvec3 force;
    };

    /**
     * @brief Stores angular motion data
     */
    struct AngularMotion
    {
        AngularMotion();

        glm::dquat orientation;
        glm::dvec3 velocity;
        glm::dvec3 acceleration;
        glm::dvec3 torque;
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
inline glm::dmat3 CalculateSolidSphereMomentOfInertia(double radius, double mass)
{
    double const factor = 2.0 / 5.0;
    double const rSq = glm::pow2(radius);
    double const inertia = (factor * mass * rSq);

    return glm::dmat3{
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
inline glm::dmat3 CalculateSolidCuboidMomentOfInertia(double width, double height, double depth, double mass)
{
    double const massFraction = 1.0 / 12.0 * mass;
    double const widthSq  = glm::pow2(width);
    double const heightSq = glm::pow2(height);
    double const depthSq  = glm::pow2(depth);

    return glm::dmat3{
        (massFraction * (heightSq + depthSq)), 0, 0,
        0, (massFraction * (widthSq + depthSq)),  0,
        0, 0, (massFraction * (widthSq + heightSq)),
    };
}

} // namespace mechanics
} // namespace pegasus
#endif // PEGASUS_BODY_HPP
