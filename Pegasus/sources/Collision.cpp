/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <pegasus/Integration.hpp>
#include <pegasus/Collision.hpp>

using namespace pegasus;
using namespace collision;

Contact::Contact(mechanics::Body& aBody, mechanics::Body& bBody, Manifold manifold, double restitution)
    : aBody(&aBody)
    , bBody(&bBody)
    , manifold(manifold)
    , restitution(restitution)
{
}

Detector::Detector(scene::AssetManager& assetManager)
    : m_assetManager(&assetManager)
{
}

std::vector<std::vector<Contact>> Detector::Detect()
{
    return {
        Detect<scene::DynamicBody, geometry::Plane>(),
        Detect<scene::DynamicBody, geometry::Plane, scene::DynamicBody, geometry::Sphere>(),
        Detect<scene::DynamicBody, geometry::Plane, scene::DynamicBody, geometry::Box>(),
        Detect<scene::DynamicBody, geometry::Plane, scene::StaticBody, geometry::Plane>(),
        Detect<scene::DynamicBody, geometry::Plane, scene::StaticBody, geometry::Sphere>(),
        Detect<scene::DynamicBody, geometry::Plane, scene::StaticBody, geometry::Box>(),

        Detect<scene::DynamicBody, geometry::Sphere>(),
        Detect<scene::DynamicBody, geometry::Sphere, scene::DynamicBody, geometry::Box>(),
        Detect<scene::DynamicBody, geometry::Sphere, scene::StaticBody, geometry::Plane>(),
        Detect<scene::DynamicBody, geometry::Sphere, scene::StaticBody, geometry::Sphere>(),
        Detect<scene::DynamicBody, geometry::Sphere, scene::StaticBody, geometry::Box>(),

        Detect<scene::DynamicBody, geometry::Box>(),
        Detect<scene::DynamicBody, geometry::Box, scene::StaticBody, geometry::Plane>(),
        Detect<scene::DynamicBody, geometry::Box, scene::StaticBody, geometry::Sphere>(),
        Detect<scene::DynamicBody, geometry::Box, scene::StaticBody, geometry::Box>(),
    };
}

bool Detector::Intersect(geometry::SimpleShape const* aShape, geometry::SimpleShape const* bShape)
{
    return s_simpleShapeDetector.CalculateIntersection(aShape, bShape);
}

geometry::SimpleShapeIntersectionDetector Detector::s_simpleShapeDetector;

Contact::Manifold Detector::CalculateContactManifold(geometry::SimpleShape const* aShape,
    geometry::SimpleShape const* bShape)
{
    Contact::Manifold manifold;
    manifold.normal = s_simpleShapeDetector.CalculateContactNormal(aShape, bShape);
    manifold.penetration = s_simpleShapeDetector.CalculatePenetration(aShape, bShape);

    return manifold;
}

void Resolver::Resolve(std::vector<std::vector<Contact>>& contacts, double duration)
{
    for (std::vector<Contact>& contact : contacts)
    {
        std::sort(contact.begin(), contact.end(),
            [](Contact const& a, Contact const& b) -> bool
        {
            return CalculateTotalSeparationSpeed(a) < CalculateTotalSeparationSpeed(b);
        });

        iterationsUsed = 0;
        while (iterationsUsed++ < iterations && !contact.empty())
        {
            Resolve(contact.back(), duration);
            contact.pop_back();
        }
    }
}

double Resolver::CalculateTotalSeparationSpeed(Contact contact)
{
    //Calculate initial separation velocity
    glm::dvec3 const relativeVelocity = contact.aBody->linearMotion.velocity - contact.bBody->linearMotion.velocity;
    double const separationSpeed = glm::dot(relativeVelocity, contact.manifold.normal);

    return separationSpeed;
}

double Resolver::CalculatePureSeparationSpeed(Contact contact, double separationSpeed, double duration)
{
    //Decompose acceleration caused separation velocity component
    double newSeparationSpeed = -separationSpeed * contact.restitution;
    glm::dvec3 const accelerationCausedVelocity = contact.aBody->linearMotion.acceleration - contact.bBody->linearMotion.acceleration;
    double const accelerationCausedSeparationSpeed = glm::dot(accelerationCausedVelocity,
        contact.manifold.normal * duration);
    if (accelerationCausedSeparationSpeed < 0.0)
    {
        newSeparationSpeed += contact.restitution * accelerationCausedSeparationSpeed;

        if (newSeparationSpeed < 0.0)
        {
            newSeparationSpeed = 0;
        }
    }
    double const deltaSpeed = newSeparationSpeed - separationSpeed;

    return deltaSpeed;
}

double Resolver::CalculateSeparationSpeed(Contact contact, double duration)
{
    double const totalSeparationSpeed = CalculateTotalSeparationSpeed(contact);
    double const pureSeparationSpeed = CalculatePureSeparationSpeed(contact, totalSeparationSpeed, duration);

    return pureSeparationSpeed;
}

glm::dvec3 Resolver::CalculateTotalImpulse(Contact contact, double duration)
{
    double const separationSpeed = CalculateSeparationSpeed(contact, duration);
    double const totalInverseMass = contact.aBody->material.GetInverseMass() + contact.bBody->material.GetInverseMass();
    double const impulse = separationSpeed / totalInverseMass;
    glm::dvec3 const totalImpulse = contact.manifold.normal * impulse;

    return totalImpulse;
}

void Resolver::ResolveVelocity(Contact contact, double duration)
{
    glm::dvec3 const totalImpulse = CalculateTotalImpulse(contact, duration);

    contact.aBody->linearMotion.velocity = contact.aBody->linearMotion.velocity + totalImpulse * contact.aBody->material.GetInverseMass();
    contact.bBody->linearMotion.velocity = contact.bBody->linearMotion.velocity + totalImpulse * -contact.bBody->material.GetInverseMass();
}

void Resolver::ResolveInterpenetration(Contact contact)
{
    double const totalInverseMass = contact.aBody->material.GetInverseMass() + contact.bBody->material.GetInverseMass();
    glm::dvec3 const movePerIMass = contact.manifold.normal * (contact.manifold.penetration / totalInverseMass);

    contact.aBody->linearMotion.position += movePerIMass * contact.aBody->material.GetInverseMass();
    contact.aBody->linearMotion.force = integration::IntegrateForce(contact.aBody->linearMotion.force, movePerIMass * -1.0);

    contact.bBody->linearMotion.position -= movePerIMass * contact.bBody->material.GetInverseMass();
    contact.bBody->linearMotion.force = integration::IntegrateForce(contact.bBody->linearMotion.force, movePerIMass * -1.0);
}

void Resolver::Resolve(Contact contact, double duration)
{
    ResolveVelocity(contact, duration);
    ResolveInterpenetration(contact);
}
