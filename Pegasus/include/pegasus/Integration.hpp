/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_INTEGRATION_HPP
#define PEGASUS_INTEGRATION_HPP

#include <pegasus/Object.hpp>
#include <pegasus/SharedMacros.hpp>

#include <glm/glm.hpp>

namespace pegasus
{

PEGASUS_EXPORT
glm::dvec3 IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce);

PEGASUS_EXPORT
void Integrate(mechanics::Body& body, double duration);

} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
