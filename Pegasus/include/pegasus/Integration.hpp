/*
* Copyright (C) 2018 by Godlike
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
inline glm::vec3 IntegrateForce(glm::vec3 accumulatedForce, glm::vec3 appliedForce)
{
    return accumulatedForce + appliedForce;
}

inline glm::vec3 IntegrateTorque(glm::vec3 accumulatedTorque, glm::vec3 appliedTorque)
{
    return IntegrateForce(accumulatedTorque, appliedTorque);
}

/**
 * @brief Calculates integrated body position
 * @param[in,out] body point mass data
 * @param[in] duration delta time
 */
void Integrate(mechanics::Body& body, float duration);
} // namespace integration
} // namespace pegasus

#endif // PEGASUS_INTEGRATION_HPP
