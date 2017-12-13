/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_OBJECT_HPP
#define PEGASUS_OBJECT_HPP

#include <pegasus/SharedMacros.hpp>
#include <glm/glm.hpp>

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
         * @brief Returns mass of the body
         * @return mass of the body
         */
        PEGASUS_EXPORT double GetMass() const;

        /**
         * @brief Returns inverse mass of the body
         * @return inverse mass
         */
        PEGASUS_EXPORT double GetInverseMass() const;

        //!Body motion bumping factor
        double damping;

    private:
        double m_mass;
        //!Is used to represent a body with an infinite mass by setting this value to 0
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

    PEGASUS_EXPORT Body();

    Material material;
    LinearMotion linearMotion;
};
} // namespace mechanics
} // namespace pegasus
#endif // PEGASUS_OBJECT_HPP
