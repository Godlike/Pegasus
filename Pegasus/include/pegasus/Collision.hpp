/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_COLLISION_HPP
#define PEGASUS_COLLISION_HPP

#include <pegasus/Scene.hpp>
#include <geometry/SimpleShapeIntersectionDetector.hpp>
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
    struct Manifold;

    Contact(mechanics::Body& aBody, mechanics::Body& bBody, Manifold manifold, double restitution);

    /**
     * @brief Stores contact manifold data
     */
    struct Manifold
    {
        glm::dvec3 normal;
        double penetration;
    };

    mechanics::Body* aBody;
    mechanics::Body* bBody;
    Manifold manifold;
    double restitution;
};

/**
 * @brief Detects collisions
 */
class Detector
{
public:
    /**
     * @brief Construct detector initialized with a given asset manager
     * @param assetManager
     */
    Detector(scene::AssetManager& assetManager);

    /**
     * @brief Detects and returns contacts
     * @return contacts
     */
    std::vector<std::vector<Contact>> Detect();

private:
    struct ObjectHasher
    {
        template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
        size_t operator()(std::pair<ObjectTypeA*, ObjectTypeB*> data) const
        {
            return std::hash<ObjectTypeA*>()(data.first) ^ std::hash<ObjectTypeB*>()(data.second);
        }
    };

    scene::AssetManager* m_assetManager = nullptr;
    static geometry::SimpleShapeIntersectionDetector s_simpleShapeDetector;

    /**
     * @brief Checks if two shapes are intersecting
     * @param aShape first shape
     * @param bShape second shape
     * @return @c true if shapes are intersecting, @c false otherwise
     */
    static bool Intersect(
        geometry::SimpleShape const* aShape, geometry::SimpleShape const* bShape
    );

    /**
     * @brief Calculates contact manifold for two intersecting shapes
     * @param aShape first shape
     * @param bShape second shapes
     * @return contact manifold
     */
    static Contact::Manifold CalculateContactManifold(
        geometry::SimpleShape const* aShape, geometry::SimpleShape const* bShape
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
        std::vector<scene::Asset<scene::RigidBody>>& objects = m_assetManager->GetObjects<Object, Shape>();

        for (scene::Asset<scene::RigidBody> aObject : objects)
        {
            for (scene::Asset<scene::RigidBody> bObject : objects)
            {
                if (aObject.id == 0 || bObject.id == 0)
                    continue;

                Shape* aShape = &m_assetManager->GetAsset(m_assetManager->GetShapes<Shape>(), aObject.data.shape);
                Shape* bShape = &m_assetManager->GetAsset(m_assetManager->GetShapes<Shape>(), bObject.data.shape);
                std::pair<Shape*, Shape*> const key = std::make_pair(aShape, bShape);

                if (aShape != bShape
                    && Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        std::ref(m_assetManager->GetAsset(m_assetManager->GetBodies(), aObject.data.body)),
                        std::ref(m_assetManager->GetAsset(m_assetManager->GetBodies(), bObject.data.body)),
                        CalculateContactManifold(aShape, bShape),
                        0.75
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
        std::unordered_set<std::pair<ShapeA*, ShapeB*>, ObjectHasher> registeredContacts;
        std::vector<scene::Asset<scene::RigidBody>>& aObjects = m_assetManager->GetObjects<ObjectA, ShapeA>();
        std::vector<scene::Asset<scene::RigidBody>>& bObjects = m_assetManager->GetObjects<ObjectB, ShapeB>();

        for (scene::Asset<scene::RigidBody> aObject : aObjects)
        {
            for (scene::Asset<scene::RigidBody> bObject : bObjects)
            {
                if (aObject.id == 0 || bObject.id == 0)
                    continue;

                ShapeA* aShape = &m_assetManager->GetAsset(m_assetManager->GetShapes<ShapeA>(), aObject.data.shape);
                ShapeB* bShape = &m_assetManager->GetAsset(m_assetManager->GetShapes<ShapeB>(), bObject.data.shape);
                std::pair<ShapeA*, ShapeB*> const key = std::make_pair(aShape, bShape);

                if (Intersect(aShape, bShape)
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.emplace_back(
                        std::ref(m_assetManager->GetAsset(m_assetManager->GetBodies(), aObject.data.body)),
                        std::ref(m_assetManager->GetAsset(m_assetManager->GetBodies(), bObject.data.body)),
                        CalculateContactManifold(aShape, bShape),
                        0.75
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
    uint32_t iterationsUsed;
    uint32_t iterations = 10000;

    /**
     * @brief Resolves collisions
     * @param contacts contacts information
     * @param duration delta time of the frame
     */
    void Resolve(std::vector<std::vector<Contact>>& contacts, double duration);

private:
    /**
     * @brief Calculates total separation speed of the contact
     * @param contact contact information
     * @return speed meters per second
     */
    static double CalculateTotalSeparationSpeed(Contact contact);

    /**
     * @brief Removes acceleration caused component from the separation speed
     * @param contact contact information
     * @param separationSpeed total separation speed
     * @param duration delta time of the frame
     * @return pure separation speed
     */
    static double CalculatePureSeparationSpeed(Contact contact, double totalSeparationSpeed, double duration);

    /**
     * @brief Calculates separation speed of the contact
     * @param contact contact information
     * @param duration delta time of the frame
     * @return separation speed
     */
    static double CalculateSeparationSpeed(Contact contact, double duration);

    /**
     * @brief Calculates total impulse of the contact
     * @param contact contact information
     * @param duration delta time of the frame
     * @return total impulse
     */
    static glm::dvec3 CalculateTotalImpulse(Contact contact, double duration);

    /**
     * @brief Updates velocities of the bodies in the contact
     * @param contact contact information
     * @param duration delta time of the frame
     */
    static void ResolveVelocity(Contact contact, double duration);

    /**
     * @brief Updated positions of the bodies in the contact
     * @param contact contact information
     */
    static void ResolveInterpenetration(Contact contact);

    /**
     * @brief Resolves contact
     * @param contact contact information
     * @param duration delta time of the frame
     */
    static void Resolve(Contact contact, double duration);
};

} // namespace collision
} // namespace pegasus

#endif //PEGASUS_COLLISION_HPP
