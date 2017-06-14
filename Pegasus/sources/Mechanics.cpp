/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/Mechanics.hpp"

using namespace pegasus;

pegasus::RigidBody::RigidBody(Particle & p, std::unique_ptr<geometry::SimpleShape> && s)
    : p(p)
    , s(std::move(s))
{
}
