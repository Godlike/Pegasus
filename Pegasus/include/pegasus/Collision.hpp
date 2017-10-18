/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_COLLISION_HPP
#define PEGASUS_COLLISION_HPP

#include <pegasus/Geometry.hpp>

namespace pegasus
{
namespace collision
{

struct Contact
{
    struct Manifold
    {
        glm::dvec3 normal;
        double penetration;
    };

    void* aObject;
    void* bObject;
    Manifold manifold;
    double restitution;
};

class Detector
{
public:
    template < typename ObjectType >
    std::vector<Contact> Detect(std::vector<ObjectType>& objects)
    {
        std::unordered_set<std::pair<ObjectType*, ObjectType*>, ObjectHasher> registeredContacts;
        std::vector<Contact> contacts;

        for (ObjectType& aObject : objects)
        {
            for (ObjectType& bObject : objects)
            {
                auto const key = std::make_pair(&aObject, &bObject);

                if (&aObject != &bObject
                    && Intersect(aObject.shape.get(), bObject.shape.get())
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.push_back({
                        reinterpret_cast<void*>(&aObject),
                        reinterpret_cast<void*>(&bObject),
                        CalculateContactManifold(aObject.shape.get(), bObject.shape.get())
                    });
                    registeredContacts.insert(key);
                }
            }
        }

        return contacts;
    }

    template < typename ObjectTypeA, typename ObjectTypeB >
    std::vector<Contact> Detect(std::vector<ObjectTypeA>& aObjects, std::vector<ObjectTypeB>& bObjects)
    {
        std::unordered_set<std::pair<ObjectTypeA*, ObjectTypeB*>, ObjectHasher> registeredContacts;
        std::vector<Contact> contacts;

        for (ObjectTypeA& aObject : aObjects)
        {
            for (ObjectTypeB & bObject : bObjects)
            {
                auto const key = std::make_pair(&aObject, &bObject);

                if (Intersect(aObject.shape.get(), bObject.shape.get())
                    && registeredContacts.find(key) == registeredContacts.end())
                {
                    contacts.push_back({
                        reinterpret_cast<void*>(&aObject),
                        reinterpret_cast<void*>(&bObject),
                        CalculateContactManifold(aObject.shape.get(), bObject.shape.get()),
                        0.75
                    });
                    registeredContacts.insert(key);
                }
            }
        }

        return contacts;
    }

private:
    struct ObjectHasher
    {
        template < typename ObjectTypeA, typename ObjectTypeB = ObjectTypeA >
        size_t operator()(std::pair<ObjectTypeA*, ObjectTypeB*> data) const
        {
            return std::hash<ObjectTypeA*>()(data.first) ^ std::hash<ObjectTypeB*>()(data.second);
        }
    };

    static geometry::SimpleShapeIntersectionDetector s_simpleShapeDetector;

    static bool Intersect(
        geometry::SimpleShape const* aShape, geometry::SimpleShape const* bShape
    );

    static Contact::Manifold CalculateContactManifold(
        geometry::SimpleShape const* aShape, geometry::SimpleShape const* bShape
    );
};

class Resolver
{
public:
    uint32_t iterationsUsed;
    uint32_t iterations = 100;

    void Resolve(std::vector<Contact>& contacts, double duration);

private:
    static double CalculateTotalSeparationSpeed(Contact contact);

    static double CalculatePureSeparationSpeed(Contact contact, double separationSpeed, double duration);

    static double CalculateSeparationSpeed(Contact contact, double duration);

    static glm::dvec3 CalculateTotalImpulse(Contact contact, double duration);

    static void ResolveVelocity(Contact contact, double duration);

    static void ResolveInterpenetration(Contact contact);

    static void Resolve(Contact contact, double duration);
};

} // namespace collision
} // namespace pegasus

#endif //PEGASUS_COLLISION_HPP
