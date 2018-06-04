/*
 * Copyright (C) 2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#include <pegasus/Integration.hpp>
#include <pegasus/Collision.hpp>


namespace pegasus
{
namespace collision
{

Contact::Contact(
    scene::Handle aHandle,
    scene::Handle bHandle,
    Manifold manifold,
    double restitution,
    double friction
)
    : aBodyHandle(aHandle)
    , bBodyHandle(bHandle)
    , manifold(manifold)
    , restitution(restitution)
    , friction(friction)
    , lagrangianMultiplier(0.0)
    , tangentLagrangianMultiplier1(0.0)
    , tangentLagrangianMultiplier2(0.0)
{
}

Detector::Detector(scene::AssetManager& assetManager)
    : m_pAssetManager(&assetManager)
{
}

std::vector<Contact> Detector::Detect()
{
    std::vector<Contact> contacts;

    Detect<scene::DynamicBody, arion::Plane>(contacts);
    Detect<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Sphere>(contacts);
    Detect<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Box>(contacts);
    Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Plane>(contacts);
    Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Sphere>(contacts);
    Detect<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Box>(contacts);

    Detect<scene::DynamicBody, arion::Sphere>(contacts);
    Detect<scene::DynamicBody, arion::Sphere, scene::DynamicBody, arion::Box>(contacts);
    Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Plane>(contacts);
    Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Sphere>(contacts);
    Detect<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Box>(contacts);

    Detect<scene::DynamicBody, arion::Box>(contacts);
    Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Plane>(contacts);
    Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Sphere>(contacts);
    Detect<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Box>(contacts);

    return contacts;
}

bool Detector::Intersect(arion::SimpleShape const* aShape, arion::SimpleShape const* bShape)
{
    return m_simpleShapeDetector.CalculateIntersection(aShape, bShape);
}

Contact::Manifold Detector::CalculateContactManifold(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    )
{
    auto const manifold = m_simpleShapeDetector.CalculateContactManifold(aShape, bShape);

    Contact::Manifold result;
    result.points = manifold.points;
    result.normal = manifold.normal;
    result.penetration = manifold.penetration;
    result.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(manifold.normal));
    result.secondTangent = glm::cross(result.firstTangent, result.normal);

    return result;
}

Resolver::Resolver(scene::AssetManager& assetManager)
    : m_pAssetManager(&assetManager)
{
}

void Resolver::Resolve(std::vector<Contact> contacts, double duration)
{
    //Solve constraints
    double contactLambda  = 0;
    double frictionLamda1 = 0;
    double frictionLamda2 = 0;
    for (auto& contact : contacts)
    {
        SolveConstraints(
            contact, duration, contactLambda, frictionLamda1, frictionLamda2
        );
    }

    //Set current contacts buffer and find persistent contacts
    DetectPersistentContacts(contacts);

    //Resolve constraints
    for (auto& contact : contacts)
    {
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.aBodyHandle).linearMotion.velocity += contact.deltaVelocity.nA;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.aBodyHandle).angularMotion.velocity += contact.deltaVelocity.nwA;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.bBodyHandle).linearMotion.velocity += contact.deltaVelocity.nB;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.bBodyHandle).angularMotion.velocity += contact.deltaVelocity.nwB;
    }

    //Save currenct contacts for use in the next frame
    m_prevContacts = std::move(contacts);
}

void Resolver::ResolvePersistantContacts(double duration)
{
    //Solve constraints
    for (auto& contact : m_persistentContacts)
    {
        double const lagrangianMultiplier = contact.lagrangianMultiplier;
        double const tangentLagrangianMultiplier1 = contact.tangentLagrangianMultiplier1;
        double const tangentLagrangianMultiplier2 = contact.tangentLagrangianMultiplier2;

        double constexpr reduction = 0.01f;
        contact.lagrangianMultiplier *= reduction;
        contact.tangentLagrangianMultiplier1 *= reduction;
        contact.tangentLagrangianMultiplier2 *= reduction;

        SolveConstraints(
            contact, duration, contact.lagrangianMultiplier,
            contact.tangentLagrangianMultiplier1, contact.tangentLagrangianMultiplier2
        );

        contact.lagrangianMultiplier = lagrangianMultiplier;
        contact.tangentLagrangianMultiplier1 = tangentLagrangianMultiplier1;
        contact.tangentLagrangianMultiplier2 = tangentLagrangianMultiplier2;
    }

    //Resolve constraints
    for (auto& contact : m_persistentContacts)
    {
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.aBodyHandle).linearMotion.velocity  += contact.deltaVelocity.nA  * m_persistentFactor;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.aBodyHandle).angularMotion.velocity += contact.deltaVelocity.nwA * m_persistentFactor;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.bBodyHandle).linearMotion.velocity  += contact.deltaVelocity.nB  * m_persistentFactor;
        m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.bBodyHandle).angularMotion.velocity += contact.deltaVelocity.nwB * m_persistentFactor;
    }
}

namespace
{
bool IsPersistent(Contact::Manifold::ContactPoints const& a, Contact::Manifold::ContactPoints const& b, double persistentThresholdSq)
{
    glm::dvec3 const curPoint = (a.aWorldSpace + b.bWorldSpace) * 0.5;
    glm::dvec3 const prevPoint = (a.aWorldSpace + b.bWorldSpace) * 0.5;

    return glm::distance2(curPoint, prevPoint) < persistentThresholdSq;
}
}

void Resolver::DetectPersistentContacts(std::vector<Contact> const& contacts)
{
    static std::vector<size_t> currentPersistentContactIndices;
    currentPersistentContactIndices.clear();

    //Find persistent contacts
    for (size_t i = 0; i < contacts.size(); ++i)
    {
        for (size_t j = 0; j < m_prevContacts.size(); ++j)
        {
            if (   contacts[i].aBodyHandle == m_prevContacts[j].aBodyHandle
                && contacts[i].bBodyHandle == m_prevContacts[j].bBodyHandle
                && IsPersistent(contacts[i].manifold.points,
                    m_prevContacts[j].manifold.points,
                    m_persistentThresholdSq))
            {
                currentPersistentContactIndices.push_back(i);
            }
        }
    }

    //Remove outdated persistent contacts
    for (size_t index = 0; index < m_persistentContacts.size();)
    {
        auto const indexIt = std::find_if(currentPersistentContactIndices.begin(), currentPersistentContactIndices.end(),
            [&contacts, this, index](size_t i) -> bool {
                return contacts[i].aBodyHandle == this->m_persistentContacts[index].aBodyHandle
                    && contacts[i].bBodyHandle == this->m_persistentContacts[index].bBodyHandle;
        });

        if (indexIt == currentPersistentContactIndices.end()
            || !IsPersistent(m_persistentContacts[index].manifold.points,
                contacts[*indexIt].manifold.points,
                m_persistentThresholdSq))
        {
            m_persistentContacts.erase(m_persistentContacts.begin() + index);
        }
        else
        {
            ++index;
        }
    }

    //Add new persistent contacts
    for (Contact const& contact : contacts)
    {
        auto const contactIterator = std::find_if(m_persistentContacts.begin(), m_persistentContacts.end(),
            [&contact](Contact& c) -> bool {
                return contact.aBodyHandle == c.aBodyHandle && contact.bBodyHandle == c.bBodyHandle;
        });

        if (contactIterator == m_persistentContacts.end())
        {
            m_persistentContacts.push_back(contact);
        }
    }
}

void Resolver::SolveConstraints(
    Contact& contact,
    double duration,
    double& contactLambda,
    double& frictionLamda1,
    double& frictionLamda2
) const
{
    mechanics::Body const& aBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.aBodyHandle);
    mechanics::Body const& bBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), contact.bBodyHandle);

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

    glm::dvec3 const rA = contact.manifold.points.aWorldSpace - aBody.linearMotion.position;
    glm::dvec3 const rB = contact.manifold.points.bWorldSpace - bBody.linearMotion.position;
    SolveContactConstraint(contact, duration, V, rA, rB, contactLambda);

    if (!epona::fp::IsZero(glm::length2(aBody.angularMotion.velocity)))
    {
        contact.manifold.firstTangent = glm::normalize(
            glm::cross(aBody.angularMotion.velocity, contact.manifold.normal));
        contact.manifold.secondTangent = glm::normalize(glm::cross(
            contact.manifold.firstTangent, contact.manifold.normal));
    }
    SolveFrictionConstraint(contact, V, rA, rB, contactLambda, frictionLamda1, frictionLamda2);

    assert(!glm::isnan(contact.deltaVelocity.nA.x + contact.deltaVelocity.nA.y + contact.deltaVelocity.nA.z));
    assert(!glm::isnan(contact.deltaVelocity.nwA.x + contact.deltaVelocity.nwA.y + contact.deltaVelocity.nwA.z));
    assert(!glm::isnan(contact.deltaVelocity.nB.x + contact.deltaVelocity.nB.y + contact.deltaVelocity.nB.z));
    assert(!glm::isnan(contact.deltaVelocity.nwB.x + contact.deltaVelocity.nwB.y + contact.deltaVelocity.nwB.z));
}

void Resolver::SolveContactConstraint(
    Contact& contact,
    double duration,
    Contact::Velocity const& V,
    glm::dvec3 const& rA,
    glm::dvec3 const& rB,
    double& totalLagrangianMultiplier
)
{
    contact.jacobian = Contact::Jacobian {
        -contact.manifold.normal,
        glm::cross(-rA, contact.manifold.normal),
        contact.manifold.normal,
        glm::cross( rB, contact.manifold.normal),
    };

    double const separationSpeed =
        -glm::dot(V.vB + glm::cross(V.wB, rB) - (V.vA + glm::cross(V.wA, rA)), contact.manifold.normal);
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

void Resolver::SolveFrictionConstraint(
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
        double const lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));

        double const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier1;
        totalTangentLagrangianMultiplier1 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier1 = glm::max(
            glm::min(totalTangentLagrangianMultiplier1, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        contact.tangentLagrangianMultiplier1 = totalTangentLagrangianMultiplier1 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * contact.tangentLagrangianMultiplier1;
    }

    {
        J = {
            -contact.manifold.secondTangent,
            glm::cross(-rA, contact.manifold.secondTangent),
            contact.manifold.secondTangent,
            glm::cross( rB, contact.manifold.secondTangent),
        };

        double const lagrangianMultiplier = -(J * V) / (J * (contact.inverseEffectiveMass * J));

        double const previousLagrangianMultiplierSum = totalTangentLagrangianMultiplier2;
        totalTangentLagrangianMultiplier2 += lagrangianMultiplier;
        totalTangentLagrangianMultiplier2 = glm::max(
            glm::min(totalTangentLagrangianMultiplier2, totalLagrangianMultiplier * contact.friction),
            -totalLagrangianMultiplier * contact.friction
        );
        contact.tangentLagrangianMultiplier2 = totalTangentLagrangianMultiplier2 - previousLagrangianMultiplierSum;

        contact.deltaVelocity += contact.inverseEffectiveMass * J * contact.tangentLagrangianMultiplier2;
    }
}

} // namespace collision
} // namespace pegasus
