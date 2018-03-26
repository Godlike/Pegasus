/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_COLLISION_HPP
#define PEGASUS_COLLISION_HPP

#include <pegasus/Asset.hpp>
#include <pegasus/AssetManager.hpp>
#include <Arion/SimpleShapeIntersectionDetector.hpp>
#include <unordered_set>

namespace pegasus
{
namespace collision
{

/**
 * @brief Stores contact information
 */
struct Contact
{
    struct Velocity
    {
        glm::dvec3 vA;
        glm::dvec3 wA;
        glm::dvec3 vB;
        glm::dvec3 wB;
    };

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

        double operator*(Jacobian const& j) const
        {
            return glm::dot(nA, j.nA) + glm::dot(nwA, j.nwA) + glm::dot(nB, j.nB) + glm::dot(nwB, j.nwB);
        }

        double operator*(Velocity const& v) const
        {
            return glm::dot(nA, v.vA) + glm::dot(nwA, v.wA) + glm::dot(nB, v.vB) + glm::dot(nwB, v.wB);
        }

        Jacobian operator*(double s) const
        {
            return {
                nA  * s,
                nwA * s,
                nB  * s,
                nwB * s,
            };
        }

        glm::dvec3 nA;
        glm::dvec3 nwA;
        glm::dvec3 nB;
        glm::dvec3 nwB;
    };

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

        glm::dmat3 massA;
        glm::dmat3 inertiaA;
        glm::dmat3 massB;
        glm::dmat3 inertiaB;
    };

    struct Manifold : arion::intersection::ContactManifold
    {
        //!Friction tangent vectors
        glm::dvec3 firstTangent;
        glm::dvec3 secondTangent;
    };

    Contact(
        mechanics::Body& aBody,
        mechanics::Body& bBody,
        scene::Handle aHandle, 
        scene::Handle bHandle, 
        Manifold manifold, 
        double restitution, 
        double friction
    );

    //Rigid bodies
    mechanics::Body* aBody = nullptr;
    mechanics::Body* bBody = nullptr;

    //Handles
    scene::Handle aBodyHandle;
    scene::Handle bBodyHandle;

    //!Contact manifold data
    Manifold manifold;

    //!Factor that's responsible for calculating the amount of energy lost to the deformation
    double restitution;
    double friction;
    
    //Contact constraint resolution data
    Jacobian deltaVelocity;
    MassMatrix inverseEffectiveMass;
    Jacobian jacobian;
    double lagrangianMultiplier;
};

/**
 * @brief Detects collisions
 */
class Detector
{
public:
    Detector() = default;

    /**
     * @brief Construct detector initialized with a given asset manager
     * @param assetManager scene's asset manager
     */
    Detector(scene::AssetManager& assetManager);

    /**
     * @brief Detects and returns contacts
     * @return contacts
     */
    std::vector<std::vector<Contact>> Detect();

    //!Default restitution factor for collision manifests
    static double constexpr s_restitutionCoefficient = 0.35;  //Wood
    static double constexpr s_frictionCoefficient    = 0.6;   //Wood

private:
    struct ObjectHasher
    {
        template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
        size_t operator()(std::pair<ObjectTypeA*, ObjectTypeB*> data) const
        {
            return std::hash<ObjectTypeA*>()(data.first) ^ std::hash<ObjectTypeB*>()(data.second);
        }
    };

    scene::AssetManager* m_pAssetManager;
    static arion::intersection::SimpleShapeIntersectionDetector s_simpleShapeDetector;

    /**
     * @brief Checks if two shapes are intersecting
     * @param aShape first shape
     * @param bShape second shape
     * @return @c true if shapes are intersecting, @c false otherwise
     */
    static bool Intersect(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    );

    /**
     * @brief Calculates contact manifold for two intersecting shapes
     * @param aShape first shape
     * @param bShape second shapes
     * @return contact manifold
     */
    static Contact::Manifold CalculateContactManifold(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    );

    /**
     * @brief Detects collisions in the given set of rigid bodies
     * @tparam Object rigid body type
     * @tparam Shape shape type
     * @return contacts
     */
    template < typename Object, typename Shape >
    std::vector<Contact> Detect()
    {
        std::vector<Contact> contacts;
        std::unordered_set<std::pair<Shape*, Shape*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>>& objects = m_pAssetManager->GetObjects<Object, Shape>();

        for (scene::Asset<scene::RigidBody> aObject : objects)
        {
            for (scene::Asset<scene::RigidBody> bObject : objects)
            {
                if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
                {
                    continue;
                }

                mechanics::Body& aBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), aObject.data.body);
                mechanics::Body& bBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), bObject.data.body);
                if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
                {
                    continue;
                }

                Shape* aShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<Shape>(), aObject.data.shape);
                Shape* bShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<Shape>(), bObject.data.shape);
                if (aShape == bShape)
                {
                    continue;
                }

                std::pair<Shape*, Shape*> const key = std::make_pair(std::min(aShape, bShape), std::max(aShape, bShape));
                if (Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        std::ref(aBody), std::ref(bBody),
                        aObject.id, bObject.id,
                        CalculateContactManifold(aShape, bShape), 
                        s_restitutionCoefficient,
                        s_frictionCoefficient
                    );
                    registeredContacts.insert(key);
                }
            }
        }

        return contacts;
    }

    /**
     * @brief Calculates contacts between two sets of rigid bodies
     * @tparam ObjectA rigid body type
     * @tparam ShapeA shape type
     * @tparam ObjectB rigid body type
     * @tparam ShapeB shape type
     * @return contacts
     */
    template < typename ObjectA, typename ShapeA, typename ObjectB, typename ShapeB >
    std::vector<Contact> Detect()
    {
        std::vector<Contact> contacts;
        std::unordered_set<std::pair<void*, void*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>>& aObjects = m_pAssetManager->GetObjects<ObjectA, ShapeA>();
        std::vector<scene::Asset<scene::RigidBody>>& bObjects = m_pAssetManager->GetObjects<ObjectB, ShapeB>();

        for (scene::Asset<scene::RigidBody> aObject : aObjects)
        {
            for (scene::Asset<scene::RigidBody> bObject : bObjects)
            {
                if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
                {
                    continue;
                }

                mechanics::Body& aBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), aObject.data.body);
                mechanics::Body& bBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), bObject.data.body);
                if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
                {
                    continue;
                }

                ShapeA* aShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<ShapeA>(), aObject.data.shape);
                ShapeB* bShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<ShapeB>(), bObject.data.shape);
                std::pair<void*, void*> const key = std::make_pair(
                    std::min(static_cast<void*>(aShape), static_cast<void*>(bShape)),
                    std::max(static_cast<void*>(aShape), static_cast<void*>(bShape))
                );

                if (Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        std::ref(aBody), std::ref(bBody),
                        aObject.id, bObject.id,
                        CalculateContactManifold(aShape, bShape), 
                        s_restitutionCoefficient,
                        s_frictionCoefficient
                    );
                    registeredContacts.insert(key);
                }
            }
        }

        return contacts;
    }
};

class Resolver
{
public:
    Resolver() = default;

    /**
    * @brief Construct resolver initialized with a given asset manager
    * @param assetManager scene's asset manager
    */
    Resolver(scene::AssetManager& assetManager);

    /**
     * @brief Resolves collisions
     * @param contacts contacts information
     * @param duration delta time of the frame
     */
    void Resolve(std::vector<std::vector<Contact>>& contacts, double duration) const;

    /**
     * @brief Resolves cached contacts
     */
    void ResolvePersistantContacts() const; 

private:
    struct ContactHasher
    {
        size_t operator()(Contact const& c) const
        {
            return std::hash<scene::Handle>()(c.aBodyHandle)
                ^ std::hash<scene::Handle>()(c.bBodyHandle);
        }

        bool operator()(Contact const& a, Contact const& b) const
        {
            return (a.aBodyHandle == b.aBodyHandle)
                && (a.bBodyHandle == b.bBodyHandle);
        }
    };

    scene::AssetManager* m_pAssetManager;
    std::unordered_map<
            Contact, std::vector<Contact>, ContactHasher, ContactHasher
        > m_contactCache;

    void ResolveConstraint(
        Contact& contact,
        double duration,
        double& contactLambda,
        double& frictionLamda1,
        double& frictionLamda2
    ) const;

    static void ResolveContactConstraint(
        Contact& contact, 
        double duration, 
        Contact::Velocity const& V,
        glm::dvec3 const& rA,
        glm::dvec3 const& rB,
        double& totalLagrangianMultiplier
    );

    static void ResolveFrictionConstraint(
        Contact& contact,
        Contact::Velocity const& V,
        glm::dvec3 const& rA, 
        glm::dvec3 const& rB,
        double& totalLagrangianMultiplier,
        double& totalTangentLagrangianMultiplier1,
        double& totalTangentLagrangianMultiplier2
    );
};

} // namespace collision
} // namespace pegasus

#endif //PEGASUS_COLLISION_HPP
