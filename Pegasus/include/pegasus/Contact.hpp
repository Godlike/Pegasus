/*
 * Copyright (C) 2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#ifndef PEGASUS_CONTACT_HPP
#define PEGASUS_CONTACT_HPP

#include <pegasus/Asset.hpp>
#include <Arion/Intersection.hpp>
#include <glm/glm.hpp>

namespace pegasus
{
namespace collision
{

/**
 * @brief Stores constraint velocity vector of size 12
 */
struct Velocity
{
    glm::vec3 vA;
    glm::vec3 wA;
    glm::vec3 vB;
    glm::vec3 wB;
};

/**
 * @brief Stores Jacobian vector of size 12 and defines operations on it
 */
struct Jacobian
{
    Jacobian& operator+=(Jacobian const& j)
    {
        nA += j.nA;
        nwA += j.nwA;
        nB += j.nB;
        nwB += j.nwB;

        return *this;
    }

    float operator*(Jacobian const& j) const
    {
        return glm::dot(nA, j.nA) + glm::dot(nwA, j.nwA) + glm::dot(nB, j.nB) + glm::dot(nwB, j.nwB);
    }

    float operator*(Velocity const& v) const
    {
        return glm::dot(nA, v.vA) + glm::dot(nwA, v.wA) + glm::dot(nB, v.vB) + glm::dot(nwB, v.wB);
    }

    Jacobian operator*(float s) const
    {
        return {
            nA  * s,
            nwA * s,
            nB  * s,
            nwB * s,
        };
    }

    glm::vec3 nA;
    glm::vec3 nwA;
    glm::vec3 nB;
    glm::vec3 nwB;
};

/**
 * @brief Stores constraint mass matrix and provides operations on it
 */
struct MassMatrix
{
    Jacobian operator*(Jacobian const& j) const
    {
        return {
            massA * j.nA,
            inertiaA * j.nwA,
            massB * j.nB,
            inertiaB * j.nwB,
        };
    }

    glm::mat3 massA;
    glm::mat3 inertiaA;
    glm::mat3 massB;
    glm::mat3 inertiaB;
};

/**
 * @brief Stores contact manifold with tangent vectors
 */
struct Manifold : arion::intersection::ContactManifold
{
    //!Friction tangent vectors
    glm::vec3 firstTangent;
    glm::vec3 secondTangent;
};

/**
 * @brief Stores contact information
 */
struct Contact
{
    //!Constructs contact instance
    Contact(scene::Handle aHandle, scene::Handle bHandle, Manifold manifold, float restitution, float friction)
        : aBodyHandle(aHandle)
        , bBodyHandle(bHandle)
        , manifold(manifold)
        , restitution(restitution)
        , friction(friction)
        , lagrangianMultiplier(0.0f)
        , tangentLagrangianMultiplier1(0.0f)
        , tangentLagrangianMultiplier2(0.0f)
    {
    }

    //!Handles
    scene::Handle aBodyHandle;
    scene::Handle bBodyHandle;

    //!Contact manifold data
    Manifold manifold;

    //!Factors responsible for calculating the amount of energy lost to the deformation
    float restitution;
    float friction;

    //!Contact constraint resolution data
    Jacobian deltaVelocity;

    //!Effective mass matrix inverse
    MassMatrix inverseEffectiveMass;

    //!Jacobian for effective mass matrix
    Jacobian jacobian;

    //!Total lagrangian multipliers
    float lagrangianMultiplier;
    float tangentLagrangianMultiplier1;
    float tangentLagrangianMultiplier2;
};

} // namespace collision
} // namespace pegasus
#endif // PEGASUS_CONTACT_HPP
