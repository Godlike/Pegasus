/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <pegasus/Integration.hpp>
#include <pegasus/Collision.hpp>

using namespace pegasus;
using namespace collision;

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

void Resolver::Resolve(std::vector<Contact>& contacts, double duration)
{
    std::sort(contacts.begin(), contacts.end(),
        [](Contact const& a, Contact const& b) -> bool
    {
        return CalculateTotalSeparationSpeed(a) < CalculateTotalSeparationSpeed(b);
    });

    iterationsUsed = 0;
    while (iterationsUsed++ < iterations && !contacts.empty())
    {
        Resolve(contacts.back(), duration);
        contacts.pop_back();
    }
}

double Resolver::CalculateTotalSeparationSpeed(Contact contact)
{
    mechanics::Body& aBody = *reinterpret_cast<mechanics::Object*>(contact.aObject)->body;
    mechanics::Body& bBody = *reinterpret_cast<mechanics::Object*>(contact.bObject)->body;

    //Calculate initial separation velocity
    glm::dvec3 const relativeVelocity = aBody.linearMotion.velocity - bBody.linearMotion.velocity;
    double const separationSpeed = glm::dot(relativeVelocity, contact.manifold.normal);

    return separationSpeed;
}

double Resolver::CalculatePureSeparationSpeed(Contact contact, double separationSpeed, double duration)
{
    mechanics::Body& aBody = *reinterpret_cast<mechanics::Object*>(contact.aObject)->body;
    mechanics::Body& bBody = *reinterpret_cast<mechanics::Object*>(contact.bObject)->body;

    //Decompose acceleration caused separation velocity component
    double newSeparationSpeed = -separationSpeed * contact.restitution;
    glm::dvec3 const accelerationCausedVelocity = aBody.linearMotion.acceleration - bBody.linearMotion.acceleration;
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
    mechanics::Body& aBody = *reinterpret_cast<mechanics::Object*>(contact.aObject)->body;
    mechanics::Body& bBody = *reinterpret_cast<mechanics::Object*>(contact.bObject)->body;

    double const separationSpeed = CalculateSeparationSpeed(contact, duration);
    double const totalInverseMass = aBody.material.GetInverseMass() + bBody.material.GetInverseMass();
    double const impulse = separationSpeed / totalInverseMass;
    glm::dvec3 const totalImpulse = contact.manifold.normal * impulse;

    return totalImpulse;
}

void Resolver::ResolveVelocity(Contact contact, double duration)
{
    mechanics::Body& aBody = *reinterpret_cast<mechanics::Object*>(contact.aObject)->body;
    mechanics::Body& bBody = *reinterpret_cast<mechanics::Object*>(contact.bObject)->body;

    glm::dvec3 const totalImpulse = CalculateTotalImpulse(contact, duration);

    aBody.linearMotion.velocity = aBody.linearMotion.velocity + totalImpulse * aBody.material.GetInverseMass();
    bBody.linearMotion.velocity = bBody.linearMotion.velocity + totalImpulse * -bBody.material.GetInverseMass();
}

void Resolver::ResolveInterpenetration(Contact contact)
{
    mechanics::Body& aBody = *reinterpret_cast<mechanics::Object*>(contact.aObject)->body;
    mechanics::Body& bBody = *reinterpret_cast<mechanics::Object*>(contact.bObject)->body;

    double const totalInverseMass = aBody.material.GetInverseMass() + bBody.material.GetInverseMass();
    glm::dvec3 const movePerIMass = contact.manifold.normal * (contact.manifold.penetration / totalInverseMass);

    aBody.linearMotion.position += movePerIMass * aBody.material.GetInverseMass();
    aBody.linearMotion.force = IntegrateForce(aBody.linearMotion.force, movePerIMass * -1.0);

    bBody.linearMotion.position -= movePerIMass * bBody.material.GetInverseMass();
    bBody.linearMotion.force = IntegrateForce(bBody.linearMotion.force, movePerIMass * -1.0);
}

void Resolver::Resolve(Contact contact, double duration)
{
    ResolveVelocity(contact, duration);
    ResolveInterpenetration(contact);
}
