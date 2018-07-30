/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Body.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace pegasus
{
namespace mechanics
{

Body::LinearMotion::LinearMotion()
    : position(0, 0, 0)
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , force(0, 0, 0)
{
}

Body::AngularMotion::AngularMotion()
    : orientation(glm::angleAxis(0.0f, glm::vec3{ 0, 0, 0 }))
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , torque(0, 0, 0)
{
}

Body::Body()
    : material()
    , linearMotion()
    , angularMotion()
{
}

} // namespace mechanics
} // namespace pegasus
