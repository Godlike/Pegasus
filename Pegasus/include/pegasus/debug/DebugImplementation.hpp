/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEBUG_IMPLEMENTATION_HPP
#define PEGASUS_DEBUG_IMPLEMENTATION_HPP

#include <pegasus/Contact.hpp>
#include <vector>
#include <functional>

namespace pegasus
{

/* Provides callback interface for debugging */
class DebugImplementation
{
public:
    /**
     * @brief Collision detection debug call
     *
     * This method is called from within the Collidion Detector
     * during the calculation. The method is called on every iteration of the EPA.
     * It then proxies all the arguments to the currently
     * set callback functor.
     *
     * @param contacts contacts data
     */
    static void CollisionDetectionCall(std::vector<collision::Contact>& contacts)
    {
        if (s_collisionDetectionCall)
    	{
            s_collisionDetectionCall(contacts);
    	}
    }

    //! Stores Collision Detection callback function
    static std::function<void(std::vector<collision::Contact>&)> s_collisionDetectionCall;
};

} // pegasus

#endif // PEGASUS_DEBUG_IMPLEMENTATION_HPP
