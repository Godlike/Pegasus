/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_INTEGRATION_HPP
#define PEGASUS_INTEGRATION_HPP

#include <pegasus/SharedMacros.hpp>

#include <glm/glm.hpp>

namespace pegasus
{
namespace integration
{
struct Material
{
    PEGASUS_EXPORT Material();

    double mass;
    double damping;
};

struct LinearMotion
{
    PEGASUS_EXPORT LinearMotion();

    glm::dvec3 position;
    glm::dvec3 velocity;
    glm::dvec3 acceleration;
    glm::dvec3 force;
};

struct StaticBody
{
    PEGASUS_EXPORT StaticBody();

    Material material;
};

struct DynamicBody : StaticBody
{
    PEGASUS_EXPORT DynamicBody();

    LinearMotion linearMotion;
};

struct KinematicBody : DynamicBody
{
};

PEGASUS_EXPORT
glm::dvec3 IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce);

PEGASUS_EXPORT
void Integrate(DynamicBody& body, double duration);

PEGASUS_EXPORT
void Integrate (KinematicBody& body, double duration);

} // namespace integration
} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
