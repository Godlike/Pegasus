/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MECHANICS_HPP
#define PEGASUS_MECHANICS_HPP

#include "pegasus/Geometry.hpp"
#include "pegasus/Particle.hpp"

#include <memory>

namespace pegasus
{

class RigidBody
{
public:
    Particle & p;
    std::unique_ptr<geometry::SimpleShape> const s;

    RigidBody(Particle & p, std::unique_ptr<geometry::SimpleShape> && s);
};

} // namespace pegasus
#endif // PEGASUS_MECHANICS_HPP
