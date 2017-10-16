/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MECHANICS_HPP
#define PEGASUS_MECHANICS_HPP

#include <pegasus/Geometry.hpp>
#include <pegasus/Integration.hpp>

#include <pegasus/SharedMacros.hpp>

#include <memory>

namespace pegasus
{

class RigidBody
{
public:
    integration::Body& pointMass;
    std::unique_ptr<geometry::SimpleShape> const shape;

    PEGASUS_EXPORT RigidBody(integration::Body& body, std::unique_ptr<geometry::SimpleShape>&& shape);
};
using RigidBodies = std::list<RigidBody>;

} // namespace pegasus
#endif // PEGASUS_MECHANICS_HPP
