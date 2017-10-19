/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_INTEGRATION_HPP
#define PEGASUS_INTEGRATION_HPP

#include <pegasus/Object.hpp>
#include <pegasus/SharedMacros.hpp>

namespace pegasus
{
namespace integration
{
PEGASUS_EXPORT
glm::dvec3 IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce);

PEGASUS_EXPORT
void Integrate(mechanics::Body& body, double duration);
} // namespace integration
} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
