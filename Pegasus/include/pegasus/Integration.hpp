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

    static double const s_infiniteMass;

    double mass;
    double inverseMass;
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

struct Body
{
    PEGASUS_EXPORT Body();

    Material material;
    LinearMotion linearMotion;
};

PEGASUS_EXPORT
bool FiniteMass(double inverseMass);

PEGASUS_EXPORT
glm::dvec3 IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce);

PEGASUS_EXPORT
void Integrate(Material& material, LinearMotion& linearMotion, double duration);
} // namespace integration
} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
