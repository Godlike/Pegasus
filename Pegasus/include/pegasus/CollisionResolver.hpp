/*
 * Copyright (C) 2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#pragma once

#include <pegasus/AssetManager.hpp>
#include <pegasus/Contact.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>

namespace pegasus
{
namespace collision
{

/**
 * @brief Resolves contact constraints and updates total lagrangian multiplier
 * @param[in,out] contact contact data
 * @param[in] duration duration of the frame
 * @param[in] V velocity vector of size 12
 * @param[in] rA contact point vector from the center of the body
 * @param[in] rB contact point vector from the center of the body
 * @param[in,out] totalLagrangianMultiplier total lagrangian multiplier for contact constraint
 */
inline void SolveContactConstraint(
    Contact& contact, float duration,
    Velocity const& V, glm::vec3 const& rA, glm::vec3 const& rB, float& totalLagrangianMultiplier
)
{
    contact.jacobian = Jacobian {
        -contact.manifold.normal,
        glm::cross(-rA, contact.manifold.normal),
        contact.manifold.normal,
        glm::cross(rB, contact.manifold.normal),
    };

    float const separationSpeed =
        -glm::dot(V.vB + glm::cross(V.wB, rB) - (V.vA + glm::cross(V.wA, rA)), contact.manifold.normal);
    float constexpr restitutionSlop = 0.5f;
    float const restitution = contact.restitution * glm::max(separationSpeed - restitutionSlop, 0.0f);
    float constexpr beta = 0.1f;
    float constexpr penetrationSlop = 0.0125f;
    float const baumgarteStabilizationTerm =
        -(beta / duration) * glm::max(contact.manifold.penetration + penetrationSlop, 0.0f) + restitution;

    float const lagrangianMultiplierDivisor = contact.jacobian * (contact.inverseEffectiveMass * contact.jacobian)
        + epona::fp::g_floatingPointThreshold;
    contact.lagrangianMultiplier = -(contact.jacobian * V + baumgarteStabilizationTerm)
        / lagrangianMultiplierDivisor;

    float const prevTotalLagrangianMultiplier = totalLagrangianMultiplier;
    totalLagrangianMultiplier += contact.lagrangianMultiplier;
    totalLagrangianMultiplier = glm::max(0.0f, totalLagrangianMultiplier);
    contact.lagrangianMultiplier = totalLagrangianMultiplier - prevTotalLagrangianMultiplier;

    contact.deltaVelocity = contact.inverseEffectiveMass * contact.jacobian * contact.lagrangianMultiplier;
}

/**
 * @brief Resolve friction constraints and updates eatch total lagrangian multiplier
 * @param[in, out] contact contact data
 * @param[in] V velocity vector of size 12
 * @param[in] rA contact point vector from the center of the body
 * @param[in] rB contact point vector from the center of the body
 * @param[in,out] totalLagrangianMultiplier accumulated value of Lagrangian Multiplier
 * @param[in,out] totalTangentLagrangianMultiplier1 accumulated value of first tangent Lagrangian Multiplier
 * @param[in,out] totalTangentLagrangianMultiplier2 accumulated value of second tangent Lagrangian Multiplier
 * @param[in] maxLagrangianMultiplier clamp value of Lagrangian Multiplier
 */
inline void SolveFrictionConstraint(
    Contact& contact,
    Velocity const& V, glm::vec3 const& rA, glm::vec3 const& rB,
    float& totalLagrangianMultiplier, float& totalTangentLagrangianMultiplier1, float& totalTangentLagrangianMultiplier2,
    float maxLagrangianMultiplier = 100.f
)
{
    Jacobian J {
        -contact.manifold.firstTangent,
        glm::cross(-rA, contact.manifold.firstTangent),
        contact.manifold.firstTangent,
        glm::cross(rB, contact.manifold.firstTangent),
    };

    {
        float lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));
        lagrangianMultiplier = glm::isnan(lagrangianMultiplier) ? 0 : lagrangianMultiplier;
        lagrangianMultiplier = lagrangianMultiplier > maxLagrangianMultiplier ? maxLagrangianMultiplier : lagrangianMultiplier;

        float const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier1;
        totalTangentLagrangianMultiplier1 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier1 = glm::max(
            glm::min(totalTangentLagrangianMultiplier1, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        contact.tangentLagrangianMultiplier1 = totalTangentLagrangianMultiplier1 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * contact.tangentLagrangianMultiplier1;

        assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
        assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
        assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
        assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));
    }

    {
        J = {
            -contact.manifold.secondTangent,
            glm::cross(-rA, contact.manifold.secondTangent),
            contact.manifold.secondTangent,
            glm::cross(rB, contact.manifold.secondTangent),
        };

        float lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));
        lagrangianMultiplier = glm::isnan(lagrangianMultiplier) ? 0 : lagrangianMultiplier;
        lagrangianMultiplier = lagrangianMultiplier > maxLagrangianMultiplier ? maxLagrangianMultiplier : lagrangianMultiplier;

        float const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier2;
        totalTangentLagrangianMultiplier2 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier2 = glm::max(
            glm::min(totalTangentLagrangianMultiplier2, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        contact.tangentLagrangianMultiplier2 = totalTangentLagrangianMultiplier2 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * contact.tangentLagrangianMultiplier2;

        assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
        assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
        assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
        assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));
    }
}

/**
 * @brief Calculates and solves contact and friction constraints and updates lambdas
 * @param[in,out] assetManager asset manager
 * @param[in,out] contact contact data
 * @param[in]     duration duration of the frame
 * @param[in,out] contactLambda lagrangian multiplier for contact constraint
 * @param[in,out] frictionLamda1 lagrangian multiplier for friction constraint
 * @param[in,out] frictionLamda2 lagrangian multiplier for friction constraint
 */
inline void SolveConstraints(
    scene::AssetManager& assetManager, Contact& contact, float duration,
    float& contactLambda, float& frictionLamda1, float& frictionLamda2
)
{
    mechanics::Body const& aBody = assetManager.GetAsset(assetManager.GetBodies(), contact.aBodyHandle);
    mechanics::Body const& bBody = assetManager.GetAsset(assetManager.GetBodies(), contact.bBodyHandle);

    assert(!std::isinf(aBody.angularMotion.velocity.x)
        && !std::isinf(aBody.angularMotion.velocity.y)
        && !std::isinf(aBody.angularMotion.velocity.z));
    assert(!std::isinf(bBody.angularMotion.velocity.x)
        && !std::isinf(bBody.angularMotion.velocity.y)
        && !std::isinf(bBody.angularMotion.velocity.z));

    Velocity const V{
        aBody.linearMotion.velocity,
        aBody.angularMotion.velocity,
        bBody.linearMotion.velocity,
        bBody.angularMotion.velocity,
    };

    contact.inverseEffectiveMass = MassMatrix{
        aBody.material.HasInfiniteMass() ? glm::mat3(0) : glm::inverse(glm::mat3(aBody.material.GetMass())),
        aBody.material.GetInverseMomentOfInertia(),
        bBody.material.HasInfiniteMass() ? glm::mat3(0) : glm::inverse(glm::mat3(bBody.material.GetMass())),
        bBody.material.GetInverseMomentOfInertia(),
    };

    glm::vec3 const rA = contact.manifold.points.aWorldSpace - aBody.linearMotion.position;
    glm::vec3 const rB = contact.manifold.points.bWorldSpace - bBody.linearMotion.position;

    if (!epona::fp::IsZero(glm::length2(aBody.angularMotion.velocity)))
    {
        glm::vec3 const velocityCrossNormal = glm::cross(aBody.angularMotion.velocity, contact.manifold.normal);
        if (!epona::fp::IsZero(glm::length2(velocityCrossNormal)))
        {
            contact.manifold.firstTangent = glm::normalize(velocityCrossNormal);
            assert(!glm::isnan(contact.manifold.firstTangent.x));
            assert(!glm::isnan(contact.manifold.firstTangent.y));
            assert(!glm::isnan(contact.manifold.firstTangent.z));
        }

        glm::vec3 const tangentCrossNormal = glm::cross(contact.manifold.firstTangent, contact.manifold.normal);
        if (!epona::fp::IsZero(glm::length2(tangentCrossNormal)))
        {
            contact.manifold.secondTangent = glm::normalize(tangentCrossNormal);
            assert(!glm::isnan(contact.manifold.secondTangent.x));
            assert(!glm::isnan(contact.manifold.secondTangent.y));
            assert(!glm::isnan(contact.manifold.secondTangent.z));
        }
    }

    SolveContactConstraint(contact, duration, V, rA, rB, contactLambda);
    assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
    assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
    assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
    assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));

    SolveFrictionConstraint(contact, V, rA, rB, contactLambda, frictionLamda1, frictionLamda2);
    assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
    assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
    assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
    assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));
}

/**
 * @brief Resolves collisions
 *
 * @note This method is inteded to be called once during the pipeline execution
 *
 * @param[in,out] assetManager        asset manager
 * @param[in,out] persistentContacts  persistent contacts
 * @param[in,out] contacts            contacts information
 * @param[in]     previousContacts    previous frame contacts
 * @param[in]     duration            delta time of the frame
 * @param[in]     persistentThreshold distance between corresponding contact points
 */
inline void ResolveContacts(
    scene::AssetManager& assetManager,
    std::vector<Contact>& persistentContacts,
    std::vector<Contact>& contacts,
    std::vector<Contact> const& previousContacts,
    float duration,
    float persistentThreshold = 1e-3f
)
{
    //Solve constraints
    float contactLambda = 0;
    float frictionLamda1 = 0;
    float frictionLamda2 = 0;
    for (auto& contact : contacts)
    {
        SolveConstraints(
            assetManager, contact, duration, contactLambda, frictionLamda1, frictionLamda2
        );
    }

    //Set current contacts buffer and find persistent contacts
    DetectPersistentContacts(contacts, previousContacts, persistentThreshold*persistentThreshold, persistentContacts);

    //Resolve constraints
    for (auto& contact : contacts)
    {
        auto& aBody = assetManager.GetAsset(assetManager.GetBodies(), contact.aBodyHandle);
        auto& bBody = assetManager.GetAsset(assetManager.GetBodies(), contact.bBodyHandle);

        aBody.linearMotion.velocity += contact.deltaVelocity.nA;
        aBody.angularMotion.velocity += contact.deltaVelocity.nwA;
        bBody.linearMotion.velocity += contact.deltaVelocity.nB;
        bBody.angularMotion.velocity += contact.deltaVelocity.nwB;

#ifndef NDEBUG
        auto a = assetManager.GetAsset(assetManager.GetBodies(), contact.aBodyHandle).angularMotion.velocity;
        auto b = assetManager.GetAsset(assetManager.GetBodies(), contact.bBodyHandle).angularMotion.velocity;
        assert(!std::isinf(a.x) && !std::isinf(a.y) && !std::isinf(a.z));
        assert(!std::isinf(b.x) && !std::isinf(b.y) && !std::isinf(b.z));
#endif
    }
}

/**
 * @brief Resolves cached contacts
 *
 * @note This method is inteded to be called once during the pipeline execution
 *
 * @param[in,out] assetManager       asset manager
 * @param[in,out] persistentContacts persistent contacts
 * @param[in]     duration           delta time of the frame
 * @param[in]     persistentFactor   factors amount of energy applied during persistent contact resolution
 */
inline void ResolvePersistantContacts(
    scene::AssetManager& assetManager,
    std::vector<Contact>& persistentContacts,
    float duration,
    float persistentFactor = 0.05f
)
{
    //Solve constraints
    for (auto& contact : persistentContacts)
    {
        float lagrangianMultiplier = contact.lagrangianMultiplier;
        float tangentLagrangianMultiplier1 = contact.tangentLagrangianMultiplier1;
        float tangentLagrangianMultiplier2 = contact.tangentLagrangianMultiplier2;

        float constexpr reduction = 0.01f;
        lagrangianMultiplier *= reduction;
        tangentLagrangianMultiplier1 *= reduction;
        tangentLagrangianMultiplier2 *= reduction;

        SolveConstraints(
            assetManager, contact, duration,
            lagrangianMultiplier, tangentLagrangianMultiplier1, tangentLagrangianMultiplier2
        );

        contact.lagrangianMultiplier = lagrangianMultiplier;
        contact.tangentLagrangianMultiplier1 = tangentLagrangianMultiplier1;
        contact.tangentLagrangianMultiplier2 = tangentLagrangianMultiplier2;
    }

    //Resolve constraints
    for (auto& contact : persistentContacts)
    {
        auto& aBody = assetManager.GetAsset(assetManager.GetBodies(), contact.aBodyHandle);
        auto& bBody = assetManager.GetAsset(assetManager.GetBodies(), contact.bBodyHandle);

        aBody.linearMotion.velocity += contact.deltaVelocity.nA  * persistentFactor;
        aBody.angularMotion.velocity += contact.deltaVelocity.nwA * persistentFactor;
        bBody.linearMotion.velocity += contact.deltaVelocity.nB  * persistentFactor;
        bBody.angularMotion.velocity += contact.deltaVelocity.nwB * persistentFactor;

#ifndef NDEBUG
        auto a = assetManager.GetAsset(assetManager.GetBodies(), contact.aBodyHandle).angularMotion.velocity;
        auto b = assetManager.GetAsset(assetManager.GetBodies(), contact.bBodyHandle).angularMotion.velocity;
        assert(!std::isinf(a.x) && !std::isinf(a.y) && !std::isinf(a.z));
        assert(!std::isinf(b.x) && !std::isinf(b.y) && !std::isinf(b.z));
#endif
    }
}

} // namespace collision
} // namespace pegasus
