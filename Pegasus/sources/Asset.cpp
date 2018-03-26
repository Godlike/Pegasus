/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Asset.hpp>
#include <pegasus/Scene.hpp>

namespace pegasus
{
namespace scene
{

RigidBody::RigidBody(Scene& scene, Handle body, Handle shape)
    : body(body)
    , shape(shape)
    , pScene(&scene)
{
}

StaticBody::StaticBody(Scene& scene, Handle body, Handle shape)
    : RigidBody(scene, body, shape)
{
    scene.GetBody(body).material.SetInfiniteMass();
}

DynamicBody::DynamicBody(Scene& scene, Handle body, Handle shape)
    : RigidBody(scene, body, shape)
{
}

} // namespace scene
} // namespace pegasus
