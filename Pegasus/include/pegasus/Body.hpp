/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_OBJECT_HPP
#define PEGASUS_OBJECT_HPP

#include <pegasus/SharedMacros.hpp>
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
     * @brief Stores material properties of the body
     */
    struct Material
    {
        PEGASUS_EXPORT Material();

        /**
         * @brief Sets mass of the body
         * @param mass mass
         */
        PEGASUS_EXPORT void SetMass(double mass);

        /**
         * @brief Sets body mass, and inverse mass equal to 0
         */
        PEGASUS_EXPORT void SetInfiniteMass();

        /**
        * @brief Sets moment of inertia of the body
        */
        PEGASUS_EXPORT void SetMomentOfInertia(glm::mat3 momentOfInertia)
        {
            m_momentOfInertia = momentOfInertia;
            m_inverseMomentOfInertia = glm::inverse(m_momentOfInertia);
        }

        /**
        * @brief Sets inverse moment of inertia of the body
        */
        PEGASUS_EXPORT void SetInverseMomentOfInertia(glm::mat3 inverseMomentOfInertia)
        {
            m_momentOfInertia = glm::inverse(inverseMomentOfInertia);
            m_inverseMomentOfInertia = inverseMomentOfInertia;
        }

        /**
        * @brief Checks if the body has an infinite mass
        * @return @c true if the mass is infinite, @c false otherwise
        */
        PEGASUS_EXPORT bool HasInfiniteMass() const;

        /**
         * @brief Returns mass of the body
         * @return mass of the body
         */
        PEGASUS_EXPORT double GetMass() const;

        /**
         * @brief Returns inverse mass of the body
         * @return inverse mass
         */
        PEGASUS_EXPORT double GetInverseMass() const;

        /**
        * @brief Returns moment of inertia of the body
        * @return moment of inertia
        */
        PEGASUS_EXPORT glm::dmat3 GetMomentOfInertia() const
        {
            return m_momentOfInertia;
        }

        /**
        * @brief Returns inverse moment of inertia of the body
        * @return inverse moment of inertia
        */
        PEGASUS_EXPORT glm::dmat3 GetInverseMomentOfInertia() const
        {
            return m_inverseMomentOfInertia;
        }

        //!Body's motion bumping factor
        double damping;

    private:
        //!Body's moment of inertia
        glm::dmat3 m_momentOfInertia;

        //!Body's inverse moment of inertia
        glm::dmat3 m_inverseMomentOfInertia;

        //!Body's inverse moment of inertia
        double m_mass;

        //!Used to represent a body with an infinite mass by setting this value to 0
        double m_inverseMass;
    };

    /**
     * @brief Stores linear motion data
     */
    struct LinearMotion
    {
        PEGASUS_EXPORT LinearMotion();

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
        PEGASUS_EXPORT AngularMotion();

        glm::dquat orientation;
        glm::dvec3 velocity;
        glm::dvec3 acceleration;
        glm::dvec3 torque;
    };

    PEGASUS_EXPORT Body();

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
    return glm::dmat3{
        (2.0 / 5.0 * mass * glm::pow2(radius)), 0, 0,
        0, (2.0 / 5.0 * mass * glm::pow2(radius)), 0,
        0, 0, (2.0 / 5.0 * mass * glm::pow2(radius)),
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
    return glm::dmat3{
        (1.0 / 12.0 * mass * (glm::pow2(height) + glm::pow2(depth))), 0, 0,
        0, (1.0 / 12.0 * mass * (glm::pow2(width) + glm::pow2(depth))),  0,
        0, 0, (1.0 / 12.0 * mass * (glm::pow2(width) + glm::pow2(height))),
    };
}

} // namespace mechanics
} // namespace pegasus
#endif // PEGASUS_OBJECT_HPP
