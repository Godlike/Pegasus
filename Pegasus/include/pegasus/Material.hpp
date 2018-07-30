/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATERIAL_HPP
#define PEGASUS_MATERIAL_HPP

#include <glm/glm.hpp>

namespace pegasus
{
namespace mechanics
{

/**
 * @brief Stores material properties of the body
 */
struct Material
{
    Material();

    /**
     * @brief Sets mass of the body
     * @param mass mass
     */
    void SetMass(float mass);

    /**
     * @brief Sets body mass, and inverse mass equal to 0
     */
    void SetInfiniteMass();

    /**
     * @brief Sets moment of inertia of the body
     */
    void SetMomentOfInertia(glm::mat3 momentOfInertia);

    /**
     * @brief Sets inverse moment of inertia of the body
     */
    void SetInverseMomentOfInertia(glm::mat3 inverseMomentOfInertia);

    /**
     * @brief Checks if the body has an infinite mass
     * @return @c true if the mass is infinite, @c false otherwise
     */
    bool HasInfiniteMass() const;

    /**
     * @brief Returns mass of the body
     * @return mass of the body
     */
    float GetMass() const;

    /**
     * @brief Returns inverse mass of the body
     * @return inverse mass
     */
    float GetInverseMass() const;

    /**
     * @brief Returns moment of inertia of the body
     * @return moment of inertia
     */
    glm::mat3 GetMomentOfInertia() const;

    /**
     * @brief Returns inverse moment of inertia of the body
     * @return inverse moment of inertia
     */
    glm::mat3 GetInverseMomentOfInertia() const;

    //!Body's motion bumping factor
    float damping;

private:
    //!Body's moment of inertia
    glm::mat3 m_momentOfInertia;

    //!Body's inverse moment of inertia
    glm::mat3 m_inverseMomentOfInertia;

    //!Body's mass
    float m_mass;

    //!Used to represent a body with an infinite mass by setting this value to 0
    float m_inverseMass;
};

} // namespace mechanics
} // namespace pegasus

#endif // PEGASUS_MATERIAL_HPP
