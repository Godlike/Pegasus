/*
 * Copyright (C) 2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#pragma once

#include <pegasus/AssetManager.hpp>
#include <pegasus/Contact.hpp>
#include <Arion/SimpleShapeIntersection.hpp>

namespace pegasus
{
namespace collision
{

struct ObjectHasher
{
    template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
    size_t operator()(std::pair<ObjectTypeA const*, ObjectTypeB const*> data) const
    {
        return reinterpret_cast<size_t>(data.first)
            ^ reinterpret_cast<size_t>(data.second);
    }
};

inline bool IsPersistent(
    Manifold::ContactPoints const& a, Manifold::ContactPoints const& b, float persistentThresholdSq
)
{
    glm::vec3 const curPoint = (a.aWorldSpace + a.bWorldSpace) * 0.5f;
    glm::vec3 const prevPoint = (b.aWorldSpace + b.bWorldSpace) * 0.5f;

    return glm::distance2(curPoint, prevPoint) < persistentThresholdSq;
}

/**
 * @brief Detects collisions in the given set of rigid bodies
 * @tparam Object rigid body type
 * @tparam Shape shape type
 * @param[in,out] assetManager asset manager
 * @param[out] contacts contact data container
 */
template < typename Object, typename Shape >
void DetectContacts(scene::AssetManager& assetManager, std::vector<Contact>& contacts)
{
    std::vector<scene::Asset<scene::RigidBody>>& objects = assetManager.GetObjects<Object, Shape>();

    for (scene::Asset<scene::RigidBody> aObject : objects)
    {
        for (scene::Asset<scene::RigidBody> bObject : objects)
        {
            if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
            {
                continue;
            }

            mechanics::Body const& aBody = assetManager.GetAsset(assetManager.GetBodies(), aObject.data.body);
            mechanics::Body const& bBody = assetManager.GetAsset(assetManager.GetBodies(), bObject.data.body);
            if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
            {
                continue;
            }

            Shape const* aShape = &assetManager.GetAsset(assetManager.GetShapes<Shape>(), aObject.data.shape);
            Shape const* bShape = &assetManager.GetAsset(assetManager.GetShapes<Shape>(), bObject.data.shape);
            if (aShape == bShape)
            {
                continue;
            }

            if (epona::fp::IsEqual(aShape->centerOfMass.x, bShape->centerOfMass.x)
                && epona::fp::IsEqual(aShape->centerOfMass.y, bShape->centerOfMass.y)
                && epona::fp::IsEqual(aShape->centerOfMass.z, bShape->centerOfMass.z))
            {
                continue;
            }

            static arion::intersection::Cache<Shape, Shape> cache;

            if (arion::intersection::CalculateIntersection<Shape, Shape>(aShape, bShape, &cache))
            {
                auto const manifold = arion::intersection::CalculateContactManifold<Shape, Shape>(aShape, bShape, &cache);
                assert(!glm::isnan(manifold.points.aWorldSpace.x));
                assert(!glm::isnan(manifold.points.bWorldSpace.x));

                static Manifold contactManifold;
                contactManifold.points = manifold.points;
                contactManifold.normal = manifold.normal;
                contactManifold.penetration = manifold.penetration;
                contactManifold.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(manifold.normal));
                contactManifold.secondTangent = glm::cross(contactManifold.firstTangent, contactManifold.normal);

                contacts.emplace_back(
                    aObject.data.body, bObject.data.body,
                    contactManifold,
                    aBody.material.restitutionCoefficient,
                    aBody.material.frictionCoefficient
                );
            }
        }
    }
}

/**
 * @brief Calculates contacts between two sets of rigid bodies
 * @tparam ObjectA rigid body type
 * @tparam ShapeA shape type
 * @tparam ObjectB rigid body type
 * @tparam ShapeB shape type
 * @param[in,out] assetManager asset manager
 * @param[out] contacts contact data container
 */
template < typename ObjectA, typename ShapeA, typename ObjectB, typename ShapeB >
void DetectContacts(scene::AssetManager& assetManager, std::vector<Contact>& contacts)
{
    std::unordered_set<std::pair<void const*, void const*>, ObjectHasher> registeredContacts;
    std::vector<scene::Asset<scene::RigidBody>> const& aObjects = assetManager.GetObjects<ObjectA, ShapeA>();
    std::vector<scene::Asset<scene::RigidBody>> const& bObjects = assetManager.GetObjects<ObjectB, ShapeB>();

    for (scene::Asset<scene::RigidBody> aObject : aObjects)
    {
        for (scene::Asset<scene::RigidBody> bObject : bObjects)
        {
            if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
            {
                continue;
            }

            mechanics::Body const& aBody = assetManager.GetAsset(assetManager.GetBodies(), aObject.data.body);
            mechanics::Body const& bBody = assetManager.GetAsset(assetManager.GetBodies(), bObject.data.body);
            if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
            {
                continue;
            }

            ShapeA const* aShape = &assetManager.GetAsset(assetManager.GetShapes<ShapeA>(), aObject.data.shape);
            ShapeB const* bShape = &assetManager.GetAsset(assetManager.GetShapes<ShapeB>(), bObject.data.shape);

            if (epona::fp::IsEqual(aShape->centerOfMass.x, bShape->centerOfMass.x)
                && epona::fp::IsEqual(aShape->centerOfMass.y, bShape->centerOfMass.y)
                && epona::fp::IsEqual(aShape->centerOfMass.z, bShape->centerOfMass.z))
            {
                continue;
            }

            static arion::intersection::Cache<ShapeA, ShapeB> cache;
            std::pair<void const*, void const*> const key = { aShape, bShape };

            if (arion::intersection::CalculateIntersection<ShapeA, ShapeB>(aShape, bShape, &cache)
                && registeredContacts.find(key) == registeredContacts.end())
            {
                auto const manifold = arion::intersection::CalculateContactManifold<ShapeA, ShapeB>(aShape, bShape, &cache);

                static Manifold contactManifold;
                contactManifold.points = manifold.points;
                contactManifold.normal = manifold.normal;
                contactManifold.penetration = manifold.penetration;
                contactManifold.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(manifold.normal));
                contactManifold.secondTangent = glm::cross(contactManifold.firstTangent, contactManifold.normal);

                contacts.emplace_back(
                    aObject.data.body, bObject.data.body,
                    contactManifold,
                    aBody.material.restitutionCoefficient,
                    aBody.material.frictionCoefficient
                );
                registeredContacts.insert(key);
            }
        }
    }
}

/**
 * @brief Detects and returns contacts
 * @return contacts vector
 */
inline std::vector<Contact> DetectContacts(scene::AssetManager& assetManager)
{
    std::vector<Contact> contacts;

    DetectContacts<scene::DynamicBody, arion::Plane>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Sphere>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Plane, scene::DynamicBody, arion::Box>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Plane>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Sphere>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Plane, scene::StaticBody, arion::Box>(assetManager, contacts);

    DetectContacts<scene::DynamicBody, arion::Sphere>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Sphere, scene::DynamicBody, arion::Box>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Plane>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Sphere>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Sphere, scene::StaticBody, arion::Box>(assetManager, contacts);

    DetectContacts<scene::DynamicBody, arion::Box>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Plane>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Sphere>(assetManager, contacts);
    DetectContacts<scene::DynamicBody, arion::Box, scene::StaticBody, arion::Box>(assetManager, contacts);

    return contacts;
}

/**
 * @brief Detects contacts existing during multiple frames
 *
 * A contact considered persistent if it exists during more than one
 * frame and the distance between corresponding contact points is changed
 * within a fixed persistence threshold. This method compares contacts
 * from the previous frame with the ones found during
 * the current frame computation @param contacts and fills @param persistentContacts
 *
 * @param[in] contacts contact set for search
 * @param[in] previousContacts previous frame contacts
 * @param[in] persistentThresholdSq distance between corresponding contact points
 * @param[in,out] persistentContacts persistent contacts
 */
inline void DetectPersistentContacts(
    std::vector<Contact> const& contacts,
    std::vector<Contact> const& previousContacts,
    float persistentThresholdSq,
    std::vector<Contact>& persistentContacts
)
{
    static std::vector<size_t> currentPersistentContactIndices;
    currentPersistentContactIndices.clear();

    //Find persistent contacts
    for (size_t i = 0; i < contacts.size(); ++i)
    {
        for (size_t j = 0; j < previousContacts.size(); ++j)
        {
            if (contacts[i].aBodyHandle == previousContacts[j].aBodyHandle
                && contacts[i].bBodyHandle == previousContacts[j].bBodyHandle
                && IsPersistent(contacts[i].manifold.points, previousContacts[j].manifold.points, persistentThresholdSq))
            {
                currentPersistentContactIndices.push_back(i);
            }
        }
    }

    //Remove outdated persistent contacts
    for (size_t index = 0; index < persistentContacts.size();)
    {
        auto const indexIt = std::find_if(currentPersistentContactIndices.begin(), currentPersistentContactIndices.end(),
            [&contacts, &persistentContacts, index](size_t i) -> bool {
            return contacts[i].aBodyHandle == persistentContacts[index].aBodyHandle
                && contacts[i].bBodyHandle == persistentContacts[index].bBodyHandle;
        });

        if (indexIt == currentPersistentContactIndices.end()
            || !IsPersistent(persistentContacts[index].manifold.points, contacts[*indexIt].manifold.points, persistentThresholdSq))
        {
            persistentContacts.erase(persistentContacts.begin() + index);
        }
        else
        {
            ++index;
        }
    }

    //Add new persistent contacts
    for (Contact const& contact : contacts)
    {
        auto const contactIterator = std::find_if(persistentContacts.begin(), persistentContacts.end(),
            [&contact](Contact& c) -> bool {
            return contact.aBodyHandle == c.aBodyHandle && contact.bBodyHandle == c.bBodyHandle;
        });

        if (contactIterator == persistentContacts.end())
        {
            persistentContacts.push_back(contact);
        }
    }
}

} // namespace collision
} // namespace pegasus
