/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/debug/DebugImplementation.hpp>

namespace pegasus
{

std::function<void(std::vector<collision::Contact>&)>
    DebugImplementation::s_collisionDetectionCall = [](std::vector<collision::Contact>&) -> void {};

} // namespace arion
