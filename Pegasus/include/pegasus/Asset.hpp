/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_ASSET_HPP
#define PEGASUS_SCENE_ASSET_HPP

#include <cstdint>

namespace pegasus
{
namespace scene
{

using Handle = uint32_t;
Handle const ZERO_HANDLE = 0;
class Scene;

/**
 * @brief Stores handles to the physical body and collision geometry
 */
struct RigidBody
{
    RigidBody() = default;

    RigidBody(Scene& scene, Handle body, Handle shape);

    Handle body = ZERO_HANDLE;
    Handle shape = ZERO_HANDLE;
    Scene* pScene = nullptr;
};

/**
 * @brief Represents a body with an infinite mass
 */
struct StaticBody : RigidBody
{
    StaticBody(Scene& scene, Handle body, Handle shape);
};

/**
 * @brief Represents a body with a finite mass
 */
struct DynamicBody : RigidBody
{
    DynamicBody(Scene& scene, Handle body, Handle shape);
};

/**
 * @brief Stores handles to a physical body and a binded force
 */
struct ForceBind
{
    Handle body = ZERO_HANDLE;
    Handle force = ZERO_HANDLE;
};

/**
* @brief Represents a unit of storage with id
* @tparam T storage data type
*/
template < typename T >
struct Asset
{
    Handle id;
    T data;
};

} // namespace scene
} // namespace pegasus
#endif // PEGASUS_SCENE_ASSET_HPP
