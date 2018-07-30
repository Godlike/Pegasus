/*
* Copyright (C) 2018 by Godlike
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
    //!Stores constraint velocity vector of size 12
    struct Velocity
    {
        glm::vec3 vA;
        glm::vec3 wA;
        glm::vec3 vB;
        glm::vec3 wB;
    };

    //!Stores Jacobian vector of size 12 and defines operations on it
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

    //!Stores constraint mass matrix and provides operations on it
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

    //!Stores contact manifold with tangent vectors
    struct Manifold : arion::intersection::ContactManifold
    {
        //!Friction tangent vectors
        glm::vec3 firstTangent;
        glm::vec3 secondTangent;
    };

    //!Constructs contact instance
    Contact(
        scene::Handle aHandle,
        scene::Handle bHandle,
        Manifold manifold,
        float restitution,
        float friction
    );

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
     * @return contacts vector
     */
    std::vector<Contact> Detect();

    //!Default restitution factor for collision manifests
    float const restitutionCoefficient = 0.35f; //Wood
    float const frictionCoefficient    = 0.6f;  //Wood

private:
    scene::AssetManager* m_pAssetManager = nullptr;
    arion::intersection::SimpleShapeIntersectionDetector m_simpleShapeDetector;

    struct ObjectHasher
    {
        template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
        size_t operator()(std::pair<ObjectTypeA const*, ObjectTypeB const*> data) const
        {
            return std::hash<ObjectTypeA const*>()(data.first)
                ^ std::hash<ObjectTypeB const*>()(data.second);
        }
    };

    /**
     * @brief Checks if two shapes are intersecting
     * @param aShape first shape
     * @param bShape second shape
     * @return @c true if shapes are intersecting, @c false otherwise
     */
    bool Intersect(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    );

    /**
     * @brief Calculates contact manifold for two intersecting shapes
     * @param aShape first shape
     * @param bShape second shapes
     * @return contact manifold
     */
    Contact::Manifold CalculateContactManifold(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    );

    /**
     * @brief Detects collisions in the given set of rigid bodies
     * @tparam Object rigid body type
     * @tparam Shape shape type
     * @return contacts
     */
    template < typename Object, typename Shape >
    void Detect(std::vector<Contact>& contacts)
    {
        std::unordered_set<std::pair<Shape const*, Shape const*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>>& objects = m_pAssetManager->GetObjects<Object, Shape>();

        for (scene::Asset<scene::RigidBody> aObject : objects)
        {
            for (scene::Asset<scene::RigidBody> bObject : objects)
            {
                if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
                {
                    continue;
                }

                mechanics::Body const& aBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), aObject.data.body);
                mechanics::Body const& bBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), bObject.data.body);
                if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
                {
                    continue;
                }

                Shape const* aShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<Shape>(), aObject.data.shape);
                Shape const* bShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<Shape>(), bObject.data.shape);
                if (aShape == bShape)
                {
                    continue;
                }

                if (   epona::fp::IsEqual(aShape->centerOfMass.x, bShape->centerOfMass.x)
                    && epona::fp::IsEqual(aShape->centerOfMass.y, bShape->centerOfMass.y)
                    && epona::fp::IsEqual(aShape->centerOfMass.z, bShape->centerOfMass.z))
                {
                    continue;
                }

                std::pair<Shape const*, Shape const*> const key = std::make_pair(std::min(aShape, bShape), std::max(aShape, bShape));
                if (Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        aObject.data.body, bObject.data.body,
                        CalculateContactManifold(aShape, bShape),
                        restitutionCoefficient,
                        frictionCoefficient
                    );
                    registeredContacts.insert(key);
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
     * @return contacts
     */
    template < typename ObjectA, typename ShapeA, typename ObjectB, typename ShapeB >
    void Detect(std::vector<Contact>& contacts)
    {
        std::unordered_set<std::pair<void const*, void const*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>> const& aObjects = m_pAssetManager->GetObjects<ObjectA, ShapeA>();
        std::vector<scene::Asset<scene::RigidBody>> const& bObjects = m_pAssetManager->GetObjects<ObjectB, ShapeB>();

        for (scene::Asset<scene::RigidBody> aObject : aObjects)
        {
            for (scene::Asset<scene::RigidBody> bObject : bObjects)
            {
                if (aObject.id == scene::ZERO_HANDLE || bObject.id == scene::ZERO_HANDLE)
                {
                    continue;
                }

                mechanics::Body const& aBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), aObject.data.body);
                mechanics::Body const& bBody = m_pAssetManager->GetAsset(m_pAssetManager->GetBodies(), bObject.data.body);
                if (aBody.material.HasInfiniteMass() && bBody.material.HasInfiniteMass())
                {
                    continue;
                }

                ShapeA const* aShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<ShapeA>(), aObject.data.shape);
                ShapeB const* bShape = &m_pAssetManager->GetAsset(m_pAssetManager->GetShapes<ShapeB>(), bObject.data.shape);
                std::pair<void const*, void const*> const key = std::make_pair(
                    std::min(static_cast<void const*>(aShape), static_cast<void const*>(bShape)),
                    std::max(static_cast<void const*>(aShape), static_cast<void const*>(bShape))
                );

                if (   epona::fp::IsEqual(aShape->centerOfMass.x, bShape->centerOfMass.x)
                    && epona::fp::IsEqual(aShape->centerOfMass.y, bShape->centerOfMass.y)
                    && epona::fp::IsEqual(aShape->centerOfMass.z, bShape->centerOfMass.z))
                {
                    continue;
                }

                if (Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        aObject.data.body, bObject.data.body,
                        CalculateContactManifold(aShape, bShape),
                        restitutionCoefficient,
                        frictionCoefficient
                    );
                    registeredContacts.insert(key);
                }
            }
        }
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
     *
     * @note This method is inteded to be called once during the pipeline execution
     *
     * @param contacts contacts information
     * @param duration delta time of the frame
     */
    void Resolve(std::vector<Contact>& contacts, float duration);

    /**
     * @brief Resolves cached contacts
     *
     * @note This method is inteded to be called once during the pipeline execution
     */
    void ResolvePersistantContacts(float duration);

private:
    //! Factors amount of energy applied during persistent contact resolution
    float const m_persistentFactor = 0.05f;
    //! Distance between corresponding contact points for them to be from a persistent contact
    float const m_persistentThreshold = 1e-3f;
    float const m_persistentThresholdSq = m_persistentThreshold * m_persistentThreshold;
    scene::AssetManager* m_pAssetManager = nullptr;
    //! Stores contacts calculated during the previous frame
    std::vector<Contact> m_prevContacts;
    //! Stores persistent contacts
    std::vector<Contact> m_persistentContacts;

    //! Calculates hashes for contacts based on handles
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

    /**
     * @brief Detects contacts existing during multiple frames
     *
     * A contact considered persistent if it exists during more than one
     * frame and the distance between corresponding contact points is changed
     * within a fixed persistence threshold. This method compares contacts
     * from the previous frame #m_prevContacts with the ones found during
     * the current frame computation @param contacts and fills #m_persistentContacts
     *
     * @param contacts contact set for search
     */
    void DetectPersistentContacts(
        std::vector<Contact> const& contacts
    );

    /**
     * @brief Calculates and solves contact and friction constraints and updates lambdas
     * @param[in,out] contact contact data
     * @param[in]     duration duration of the frame
     * @param[in,out] contactLambda lagrangian multiplier for contact constraint
     * @param[in,out] frictionLamda1 lagrangian multiplier for friction constraint
     * @param[in,out] frictionLamda2 lagrangian multiplier for friction constraint
     */
    void SolveConstraints(
        Contact& contact, float duration,
        float& contactLambda, float& frictionLamda1, float& frictionLamda2
    ) const;

    /**
     * @brief Resolves contact constraints and updates total lagrangian multiplier
     * @param[in,out] contact contact data
     * @param[in] duration durtaion of the frame
     * @param[in] V velocity vector of size 12
     * @param[in] rA contact point vector from the center of the body
     * @param[in] rB contact point vector from the center of the body
     * @param[in] totalLagrangianMultiplier total lagrangian multiplier for contact constraint
     */
    static void SolveContactConstraint(
        Contact& contact, float duration,
        Contact::Velocity const& V, glm::vec3 const& rA, glm::vec3 const& rB, float& totalLagrangianMultiplier
    );

    /**
     * @brief Resolve friction constraints and updates eatch total lagrangian multiplier
     * @param[in, out] contact contact data
     * @param[in] V velocity vector of size 12
     * @param[in] rA contact point vector from the center of the body
     * @param[in] rB contact point vector from the center of the body
     * @param[in,out] totalLagrangianMultiplier
     * @param[in,out] totalTangentLagrangianMultiplier1
     * @param[in,out] totalTangentLagrangianMultiplier2
     */
    static void SolveFrictionConstraint(
        Contact& contact,
        Contact::Velocity const& V, glm::vec3 const& rA,  glm::vec3 const& rB,
        float& totalLagrangianMultiplier, float& totalTangentLagrangianMultiplier1, float& totalTangentLagrangianMultiplier2
    );
};

} // namespace collision
} // namespace pegasus

#endif //PEGASUS_COLLISION_HPP
