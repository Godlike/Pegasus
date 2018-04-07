/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#include <pegasus/Integration.hpp>
#include <pegasus/Collision.hpp>

namespace
{
void ApplyDeltas(pegasus::collision::Contact& contact)
{
    contact.aBody->linearMotion.velocity += contact.deltaVelocity.nA;
    contact.aBody->angularMotion.velocity += contact.deltaVelocity.nwA;
    contact.bBody->linearMotion.velocity += contact.deltaVelocity.nB;
    contact.bBody->angularMotion.velocity += contact.deltaVelocity.nwB;
}
} // namespace ::

namespace pegasus
{
namespace collision
{

Contact::Contact(
    mechanics::Body& aBody,
    mechanics::Body& bBody,
    scene::Handle aHandle,
    scene::Handle bHandle,
    Manifold manifold,
    double restitution,
    double friction
)
    : aBody(&aBody)
    , bBody(&bBody)
    , aBodyHandle(aHandle)
    , bBodyHandle(bHandle)
    , manifold(manifold)
    , restitution(restitution)
    , friction(friction)
    , lagrangianMultiplier(0.0)
{
}

Detector::Detector(scene::AssetManager& assetManager)
    : m_pAssetManager(&assetManager)
{
}

std::vector<std::vector<Contact>> Detector::Detect()
{
    return {
        Detect<scene::DynamicBody, arion::Plane>(),
        Detect<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Sphere>(),
        Detect<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Box>(),
        Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Plane>(),
        Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Sphere>(),
        Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Box>(),

        Detect<scene::DynamicBody, arion::Sphere>(),
        Detect<scene::DynamicBody, arion::Sphere, scene::DynamicBody, arion::Box>(),
        Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Plane>(),
        Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Sphere>(),
        Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Box>(),

        Detect<scene::DynamicBody, arion::Box>(),
        Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Plane>(),
        Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Sphere>(),
        Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Box>(),
    };
}

double constexpr Detector::s_restitutionCoefficient;
double constexpr Detector::s_frictionCoefficient;

bool Detector::Intersect(arion::SimpleShape const* aShape, arion::SimpleShape const* bShape)
{
    return s_simpleShapeDetector.CalculateIntersection(aShape, bShape);
}

arion::intersection::SimpleShapeIntersectionDetector Detector::s_simpleShapeDetector;

Contact::Manifold Detector::CalculateContactManifold(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    )
{
    auto const manifold = s_simpleShapeDetector.CalculateContactManifold(aShape, bShape);

    Contact::Manifold result;
    result.contactPoints = manifold.contactPoints;
    result.contactNormal = manifold.contactNormal;
    result.penetration = manifold.penetration;
    result.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(manifold.contactNormal));
    result.secondTangent = glm::cross(result.firstTangent, result.contactNormal);

    return result;
}

Resolver::Resolver(scene::AssetManager& assetManager)
    : m_pAssetManager(&assetManager)
{
}

void Resolver::Resolve(std::vector<std::vector<Contact>>& contacts, double duration) const
{
    double contactLambda  = 0;
    double frictionLamda1 = 0;
    double frictionLamda2 = 0;

    for (std::vector<Contact>& contact : contacts)
    {
        for (auto& c : contact)
        {
            ResolveConstraint(
                c, duration, contactLambda, frictionLamda1, frictionLamda2
            );
        }
    }

    for (auto& contact : contacts)
    {
        for (auto& c : contact)
        {
            ::ApplyDeltas(c);
        }
    }
}

void Resolver::ResolvePersistantContacts() const
{
    // for (auto& contact : m_contactCache)
    // {
        //
    // }
}

void Resolver::ResolveConstraint(
    Contact& contact,
    double duration,
    double& contactLambda,
    double& frictionLamda1,
    double& frictionLamda2
) const
{
    mechanics::Body const& aBody = *contact.aBody;
    mechanics::Body const& bBody = *contact.bBody;

    Contact::Velocity const V {
        aBody.linearMotion.velocity,
        aBody.angularMotion.velocity,
        bBody.linearMotion.velocity,
        bBody.angularMotion.velocity,
    };

    contact.inverseEffectiveMass = Contact::MassMatrix {
        aBody.material.HasInfiniteMass() ? glm::dmat3(0) : glm::inverse(glm::dmat3(aBody.material.GetMass())),
        aBody.material.GetInverseMomentOfInertia(),
        bBody.material.HasInfiniteMass() ? glm::dmat3(0) : glm::inverse(glm::dmat3(bBody.material.GetMass())),
        bBody.material.GetInverseMomentOfInertia(),
    };

    glm::dvec3 const rA = contact.manifold.contactPoints.aWorldSpace - aBody.linearMotion.position;
    glm::dvec3 const rB = contact.manifold.contactPoints.bWorldSpace - bBody.linearMotion.position;
    ResolveContactConstraint(contact, duration, V, rA, rB, contactLambda);

    if (!epona::fp::IsZero(glm::length2(aBody.angularMotion.velocity)))
    {
        contact.manifold.firstTangent = glm::normalize(
            glm::cross(aBody.angularMotion.velocity, contact.manifold.contactNormal));
        contact.manifold.secondTangent = glm::normalize(glm::cross(
            contact.manifold.firstTangent, contact.manifold.contactNormal));
    }
    ResolveFrictionConstraint(contact, V, rA, rB, contactLambda, frictionLamda1, frictionLamda2);

    assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
    assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
    assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
    assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));
}

void Resolver::ResolveContactConstraint(
    Contact& contact,
    double duration,
    Contact::Velocity const& V,
    glm::dvec3 const& rA,
    glm::dvec3 const& rB,
    double& totalLagrangianMultiplier
)
{
    contact.jacobian = Contact::Jacobian {
        -contact.manifold.contactNormal,
        glm::cross(-rA, contact.manifold.contactNormal),
        contact.manifold.contactNormal,
        glm::cross( rB, contact.manifold.contactNormal),
    };

    double const separationSpeed =
        -glm::dot(V.vB + glm::cross(V.wB, rB) - (V.vA + glm::cross(V.wA, rA)), contact.manifold.contactNormal);
    double constexpr restitutionSlop = 0.5;
    double const restitution = contact.restitution * glm::max(separationSpeed - restitutionSlop, 0.0);
    double constexpr beta = 0.1;
    double constexpr penetrationSlop = 0.0125;
    double const baumgarteStabiliationTerm =
        - (beta / duration) * glm::max(contact.manifold.penetration + penetrationSlop, 0.0) + restitution;
    contact.lagrangianMultiplier = -(contact.jacobian * V + baumgarteStabiliationTerm)
        / (contact.jacobian * (contact.inverseEffectiveMass * contact.jacobian));

    double const prevTotalLagrangianMultiplier = totalLagrangianMultiplier;
    totalLagrangianMultiplier += contact.lagrangianMultiplier;
    totalLagrangianMultiplier = glm::max(0.0, totalLagrangianMultiplier);
    contact.lagrangianMultiplier = totalLagrangianMultiplier - prevTotalLagrangianMultiplier;

    contact.deltaVelocity = contact.inverseEffectiveMass * contact.jacobian * contact.lagrangianMultiplier;
}

void Resolver::ResolveFrictionConstraint(
    Contact& contact,
    Contact::Velocity const& V,
    glm::dvec3 const& rA,
    glm::dvec3 const& rB,
    double& totalLagrangianMultiplier,
    double& totalTangentLagrangianMultiplier1,
    double& totalTangentLagrangianMultiplier2
)
{
    Contact::Jacobian J {
        -contact.manifold.firstTangent,
        glm::cross(-rA, contact.manifold.firstTangent),
        contact.manifold.firstTangent,
        glm::cross( rB, contact.manifold.firstTangent),
    };

    {
        double lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));

        double const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier1;
        totalTangentLagrangianMultiplier1 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier1 = glm::max(
            glm::min(totalTangentLagrangianMultiplier1, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        lagrangianMultiplier = totalTangentLagrangianMultiplier1 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * lagrangianMultiplier;
    }

    {
        J = {
            -contact.manifold.secondTangent,
            glm::cross(-rA, contact.manifold.secondTangent),
            contact.manifold.secondTangent,
            glm::cross( rB, contact.manifold.secondTangent),
        };

        double lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));

        double const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier2;
        totalTangentLagrangianMultiplier2 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier2 = glm::max(
            glm::min(totalTangentLagrangianMultiplier2, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        lagrangianMultiplier = totalTangentLagrangianMultiplier2 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * lagrangianMultiplier;
    }
}

} // namespace collision
} // namespace pegasus
