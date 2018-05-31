/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEBUG_IMPLEMENTATION_HPP
#define PEGASUS_DEBUG_IMPLEMENTATION_HPP

#include <pegasus/Collision.hpp>
#include <functional>

namespace pegasus
{

/* Provides callback interface for debugging */
class DebugDummy
{
public:

    /**
     * @brief Collision detection debug call
     *
     * @note This is a dummy method that should be optimized away
     *
     * @param contacts contacts data
     */
    static void CollisionDetectionCall(std::vector<collision::Contact>& contacts) {}

    //! Stores Collision Detection callback function
    static std::function<void(std::vector<collision::Contact>&)> s_collisionDetectionCall;
};

} // pegasus

#endif // PEGASUS_DEBUG_IMPLEMENTATION_HPP
