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
        glm::dvec3 vA;
        glm::dvec3 wA;
        glm::dvec3 vB;
        glm::dvec3 wB;
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

        glm::dmat3 massA;
        glm::dmat3 inertiaA;
        glm::dmat3 massB;
        glm::dmat3 inertiaB;
    };

    //!Stores contact manifold with tangent vectors
    struct Manifold : arion::intersection::ContactManifold
    {
        //!Friction tangent vectors
        glm::dvec3 firstTangent;
        glm::dvec3 secondTangent;
    };

    //!Constructs contact instance
    Contact(
        scene::Handle aHandle,
        scene::Handle bHandle,
        Manifold manifold,
        double restitution,
        double friction
    );

    //!Handles
    scene::Handle aBodyHandle;
    scene::Handle bBodyHandle;

    //!Contact manifold data
    Manifold manifold;

    //!Factors responsible for calculating the amount of energy lost to the deformation
    double restitution;
    double friction;

    //!Contact constraint resolution data
    Jacobian deltaVelocity;
    //!Effective mass matrix inverse
    MassMatrix inverseEffectiveMass;
    //!Jacobian for effective mass matrix
    Jacobian jacobian;
    //!Total lagrangian multipliers
    double lagrangianMultiplier;
    double tangentLagrangianMultiplier1;
    double tangentLagrangianMultiplier2;
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
    double const restitutionCoefficient = 0.35; //Wood
    double const frictionCoefficient    = 0.6;  //Wood

private:
    scene::AssetManager* m_pAssetManager = nullptr;
    arion::intersection::SimpleShapeIntersectionDetector m_simpleShapeDetector;

    struct ObjectHasher
    {
        template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
        size_t operator()(std::pair<ObjectTypeA const*, ObjectTypeB const*> data) const
        {
            return reinterpret_cast<uint64_t>(data.first)
                ^ reinterpret_cast<uint64_t>(data.second);
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
        static arion::intersection::Cache<Shape, Shape> cache = {};
        std::unordered_set<std::pair<Shape const*, Shape const*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>>& objects = m_pAssetManager->GetObjects<Object, Shape>();

        for (scene::Asset<scene::RigidBody> aObject : objects)
        {
            for (scene::Asset<scene::RigidBody> const& bObject : objects)
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
   
                std::pair<Shape const*, Shape const*> const key = std::make_pair(aShape, bShape);
                bool const intersection = arion::intersection::CalculateIntersection<Shape, Shape>(aShape, bShape, &cache);
                bool const repetition = registeredContacts.find(key) != registeredContacts.end();
                if (!intersection || repetition)
                {
                    continue;
                }

                {
                    auto const contactManifold = arion::intersection::CalculateContactManifold<Shape, Shape>(aShape, bShape, &cache);

                    Contact::Manifold manifold;
                    manifold.points = contactManifold.points;
                    manifold.normal = contactManifold.normal;
                    manifold.penetration = contactManifold.penetration;
                    manifold.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(contactManifold.normal));
                    manifold.secondTangent = glm::cross(manifold.firstTangent, manifold.normal);

                    contacts.emplace_back(
                        aObject.data.body, bObject.data.body,
                        manifold,
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
        static arion::intersection::Cache<ShapeA, ShapeB> cache = {};
        std::unordered_set<std::pair<void const*, void const*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>> const& aObjects = m_pAssetManager->GetObjects<ObjectA, ShapeA>();
        std::vector<scene::Asset<scene::RigidBody>> const& bObjects = m_pAssetManager->GetObjects<ObjectB, ShapeB>();

        for (scene::Asset<scene::RigidBody> aObject : aObjects)
        {
            for (scene::Asset<scene::RigidBody> const& bObject : bObjects)
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
                if (   epona::fp::IsEqual(aShape->centerOfMass.x, bShape->centerOfMass.x)
                    && epona::fp::IsEqual(aShape->centerOfMass.y, bShape->centerOfMass.y)
                    && epona::fp::IsEqual(aShape->centerOfMass.z, bShape->centerOfMass.z))
                {
                    continue;
                }

                auto const key = std::make_pair(static_cast<void const*>(aShape), static_cast<void const*>(bShape));
                bool const intersection = arion::intersection::CalculateIntersection<ShapeA, ShapeB>(aShape, bShape, &cache);
                bool const repetition = registeredContacts.find(key) != registeredContacts.end();
                if (!intersection || repetition)
                {
                    continue;
                }

                {
                    auto const contactManifold = arion::intersection::CalculateContactManifold<ShapeA, ShapeB>(aShape, bShape, &cache);

                    Contact::Manifold manifold;
                    manifold.points = contactManifold.points;
                    manifold.normal = contactManifold.normal;
                    manifold.penetration = contactManifold.penetration;
                    manifold.firstTangent = glm::normalize(epona::CalculateOrthogonalVector(contactManifold.normal));
                    manifold.secondTangent = glm::cross(manifold.firstTangent, manifold.normal);

                    contacts.emplace_back(
                        aObject.data.body, bObject.data.body,
                        manifold,
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
    void Resolve(std::vector<Contact>& contacts, double duration);

    /**
     * @brief Resolves cached contacts
     *
     * @note This method is inteded to be called once during the pipeline execution
     */
    void ResolvePersistantContacts(double duration);

private:
    //! Factors amount of energy applied during persistent contact resolution
    double const m_persistentFactor = 0.05f;
    //! Distance between corresponding contact points for them to be from a persistent contact
    double const m_persistentThreshold = 1e-3;
    double const m_persistentThresholdSq = m_persistentThreshold * m_persistentThreshold;
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
        Contact& contact, double duration,
        double& contactLambda, double& frictionLamda1, double& frictionLamda2
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
        Contact& contact, double duration,
        Contact::Velocity const& V, glm::dvec3 const& rA, glm::dvec3 const& rB, double& totalLagrangianMultiplier
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
        Contact::Velocity const& V, glm::dvec3 const& rA,  glm::dvec3 const& rB,
        double& totalLagrangianMultiplier, double& totalTangentLagrangianMultiplier1, double& totalTangentLagrangianMultiplier2
    );
};

} // namespace collision
} // namespace pegasus

#endif //PEGASUS_COLLISION_HPP
