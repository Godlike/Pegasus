/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Integration.hpp>
#include <pegasus/Collision.hpp>

namespace pegasus
{
namespace collision
{

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

double constexpr Detector::restitutionCoefficient;

bool Detector::Intersect(arion::SimpleShape const* aShape, arion::SimpleShape const* bShape)
{
    return s_simpleShapeDetector.CalculateIntersection(aShape, bShape);
}

arion::SimpleShapeIntersectionDetector Detector::s_simpleShapeDetector;

Contact::Manifold Detector::CalculateContactManifold(
        arion::SimpleShape const* aShape, arion::SimpleShape const* bShape
    )
{
    Contact::Manifold manifold;

    manifold.normal = s_simpleShapeDetector.CalculateContactNormal(aShape, bShape);
    std::pair<glm::dvec3, glm::dvec3> const contactPoints =
        s_simpleShapeDetector.CalculateContactPoints(aShape, bShape);
    manifold.aContactPoint = contactPoints.first;
    manifold.bContactPoint = contactPoints.second;
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

double Resolver::CalculatePureSeparationSpeed(Contact contact, double totalSeparationSpeed, double duration)
{
    //Decompose acceleration caused separation velocity component
    glm::dvec3 const accelerationCausedVelocity =
        contact.aBody->linearMotion.acceleration - contact.bBody->linearMotion.acceleration;
    double const accelerationCausedSeparationSpeed =
        glm::dot(accelerationCausedVelocity, contact.manifold.normal * duration);

    double newSeparationSpeed = -totalSeparationSpeed * contact.restitution;
    if (accelerationCausedSeparationSpeed < 0.0)
    {
        newSeparationSpeed += contact.restitution * accelerationCausedSeparationSpeed;

        if (newSeparationSpeed < 0.0)
        {
            newSeparationSpeed = 0;
        }
    }
    double const deltaSpeed = newSeparationSpeed - totalSeparationSpeed;

    return deltaSpeed;
}

double Resolver::CalculateSeparationSpeed(Contact contact, double duration)
{
    double const totalSeparationSpeed = CalculateTotalSeparationSpeed(contact);
    double const pureSeparationSpeed = CalculatePureSeparationSpeed(contact, totalSeparationSpeed, duration);

    return pureSeparationSpeed;
}

void Resolver::ResolveVelocity(Contact contact, double duration)
{
    //Convert contact points to the local space
    contact.manifold.aContactPoint -= contact.aBody->linearMotion.position;
    contact.manifold.bContactPoint -= contact.bBody->linearMotion.position;
    contact.restitution = 0.5;

    //Calculate total contact impulse magnitude
    glm::dvec3 const aContactPointVelocity = contact.aBody->linearMotion.velocity
        + glm::cross(contact.aBody->angularMotion.velocity, contact.manifold.aContactPoint);
    glm::dvec3 const bContactPointVelocity = contact.bBody->linearMotion.velocity
        + glm::cross(contact.bBody->angularMotion.velocity, contact.manifold.bContactPoint);

    glm::dvec3 const aBodyAngularImpulse = contact.aBody->material.GetInverseMomentOfInertia()
        * glm::cross(glm::cross(contact.manifold.aContactPoint, contact.manifold.normal), contact.manifold.aContactPoint);
    glm::dvec3 const bBodyAngularImpulse = contact.bBody->material.GetInverseMomentOfInertia()
        * glm::cross(glm::cross(contact.manifold.bContactPoint, contact.manifold.normal), contact.manifold.bContactPoint);

    double const linearImpulsePerUnitMass = contact.aBody->material.GetInverseMass() + contact.bBody->material.GetInverseMass();
    double const angularImpulsePerUnitMass = glm::dot(aBodyAngularImpulse + bBodyAngularImpulse, contact.manifold.normal);
    double const totalImpulsePerUnitMass = linearImpulsePerUnitMass + angularImpulsePerUnitMass;
    glm::dvec3 const relativeContactPointVelocity = aContactPointVelocity - bContactPointVelocity;

    double const impulseMagnitude =
        glm::dot(-(1 + contact.restitution) * relativeContactPointVelocity, contact.manifold.normal) / totalImpulsePerUnitMass;
    glm::dvec3 const impulse = impulseMagnitude * contact.manifold.normal;

    //Calculate post contact linear velocity
    contact.aBody->linearMotion.velocity =
        contact.aBody->linearMotion.velocity + impulse * contact.aBody->material.GetInverseMass();
    contact.bBody->linearMotion.velocity =
        contact.bBody->linearMotion.velocity - impulse * contact.bBody->material.GetInverseMass();

    //Calculate post contact angular velocity
    contact.aBody->angularMotion.velocity = contact.aBody->angularMotion.velocity
        + (contact.aBody->material.GetInverseMomentOfInertia() * impulseMagnitude)
        * glm::cross(contact.manifold.aContactPoint, contact.manifold.normal);
    contact.bBody->angularMotion.velocity = contact.bBody->angularMotion.velocity
        - (contact.bBody->material.GetInverseMomentOfInertia() * impulseMagnitude)
        * glm::cross(contact.manifold.bContactPoint, contact.manifold.normal);
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
} // namespace collision
} // namespace pegasus
