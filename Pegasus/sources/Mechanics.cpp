/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Mechanics.hpp>

using namespace pegasus;

RigidBody::RigidBody(integration::Body& pointMass, std::unique_ptr<geometry::SimpleShape>&& shape)
    : pointMass(pointMass)
    , shape(std::move(shape))
{
}
