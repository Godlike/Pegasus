/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_INTEGRATION_HPP
#define PEGASUS_INTEGRATION_HPP

#include <pegasus/Body.hpp>

namespace pegasus
{
namespace integration
{
/**
 * @brief Calculates total force acting on the body
 * @param accumulatedForce current total force
 * @param appliedForce applied force
 * @return total force
 */
inline glm::dvec3 IntegrateForce(glm::dvec3 accumulatedForce, glm::dvec3 appliedForce)
{
    return accumulatedForce + appliedForce;
}

inline glm::dvec3 IntegrateTorque(glm::dvec3 accumulatedTorque, glm::dvec3 appliedTorque)
{
    return IntegrateForce(accumulatedTorque, appliedTorque);
}

/**
 * @brief Calculates integrated body position
 * @param[in,out] body point mass data
 * @param[in] duration delta time
 */
void Integrate(mechanics::Body& body, double duration);
} // namespace integration
} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
