/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_ASSET_MANAGER_HPP
#define PEGASUS_SCENE_ASSET_MANAGER_HPP

#include <pegasus/Asset.hpp>
#include <pegasus/Force.hpp>
#include <Arion/Shape.hpp>
#include <vector>
#include <deque>

namespace pegasus
{
namespace scene
{

/**
 * @brief Stores scene's assets
 */
class AssetManager
{
public:
    struct Assets;

    /**
    * @brief Returns a handle for @p T data
    *
    * Searches for an empty handle in @p data. If there are no empty handles, creates a new one.
    *
    * @attention Calling this method invalidates all data pointers and iterators, handles are
    * guaranteed to not be invalidated
    *
    * @tparam T storage data type
    * @param[in,out] data reference to the vector of assets
    * @return handle to the free element
    */
    template < typename T >
    Handle MakeAsset(std::vector<Asset<T>>& data)
    {
        for (size_t i = 0; i < data.size(); ++i)
        {
            if (data[i].id == ZERO_HANDLE)
            {
                data[i].id = static_cast<Handle>(i + 1);
                return data[i].id;
            }
        }

        data.resize(data.size() + 1);
        data.back().id = static_cast<Handle>(data.size());
        return data.back().id;
    }

    /**
    * @brief Return reference to the data assigned to the given vector
    *
    * @attention Reference might be invalidated after the push to the given vector
    *
    * @tparam T storage data type
    * @param[in] data assets vector
    * @param[in] id handle of the data
    * @return reference to the data
    */
    template < typename T >
    T& GetAsset(std::vector<Asset<T>>& data, Handle id)
    {
        return data[id - 1].data;
    }

    /**
    * @brief Marks element as free
    * @tparam T storage data type
    * @param[in,out] data vector of assets
    * @param[in] id handle to the element
    */
    template < typename T >
    void RemoveAsset(std::vector<Asset<T>>& data, Handle id)
    {
        data[id - 1].id = 0;
    }

    /**
     * @brief Returns body buffer
     * @return physical body buffer
     */
    std::vector<Asset<mechanics::Body>>& GetBodies();

    /**
     * @brief Returns @t Shape buffer
     * @tparam Shape collision geometry shape type
     * @return shape buffer
     */
    template < typename Shape >
    std::vector<Asset<Shape>>& GetShapes();

    /**
     * @brief Returns rigid body buffer
     * @tparam Object body type
     * @tparam Shape collision geometry shape type
     * @return rigid body buffer
     */
    template < typename Object, typename Shape >
    std::vector<Asset<RigidBody>>& GetObjects();

    /**
     * @brief Return @t Force buffer
     * @tparam Force force type
     * @return force buffer
     */
    template < typename Force >
    std::vector<Asset<Force>>& GetForces();

    /**
     * @brief Returns body force bind buffer
     * @tparam Force force type
     * @return body force bind buffer
     */
    template < typename Force >
    std::vector<Asset<ForceBind>>& GetForceBinds();

    /**
     * @brief Saves a copy of current scene on asset stack
     */
    void PushFrame()
    {
        m_assetStack.push_back(m_asset);
    }

    /**
    * @brief Removes top stack element from the asset stack
    */
    void PopFrame()
    {
        if (!m_assetStack.empty())
        {
            m_assetStack.pop_back();
        }
    }

    /**
     * @brief Copies top stack element to the current assets
     */
    void Top()
    {
        if (!m_assetStack.empty())
        {
            m_asset = m_assetStack.back();
        }
    }

    /**
     * @brief Stores asset data
     */
    struct Assets
    {
        //!Bodies
        std::vector<Asset<mechanics::Body>> m_bodies;

        //!Shapes
        std::vector<Asset<arion::Plane>> m_planes;
        std::vector<Asset<arion::Sphere>> m_spheres;
        std::vector<Asset<arion::Box>> m_boxes;

        //!Objects
        std::vector<Asset<RigidBody>> m_staticPlanes;
        std::vector<Asset<RigidBody>> m_dynamicPlanes;
        std::vector<Asset<RigidBody>> m_staticSpheres;
        std::vector<Asset<RigidBody>> m_dynamicSpheres;
        std::vector<Asset<RigidBody>> m_staticBoxes;
        std::vector<Asset<RigidBody>> m_dynamicBoxes;

        //!Forces
        std::vector<Asset<force::StaticField>> m_staticFieldForces;
        std::vector<Asset<force::SquareDistanceSource>> m_squareDistanceSourceForces;
        std::vector<Asset<force::Drag>> m_dragForces;
        std::vector<Asset<force::Spring>> m_springForces;
        std::vector<Asset<force::Bungee>> m_bungeeForces;
        std::vector<Asset<force::Buoyancy>> m_buoyancyForces;

        //!ForceBind bindings
        std::vector<Asset<ForceBind>> m_staticFieldForceBindings;
        std::vector<Asset<ForceBind>> m_squareDistanceSourceForceBindings;
        std::vector<Asset<ForceBind>> m_dragForceBindings;
        std::vector<Asset<ForceBind>> m_springForceBindings;
        std::vector<Asset<ForceBind>> m_bungeeForceBindings;
        std::vector<Asset<ForceBind>> m_buoyancyForceBindings;
    };

private:
    Assets m_asset;
    std::deque<Assets> m_assetStack;
};

template <>
inline std::vector<Asset<arion::Plane>>& AssetManager::GetShapes<arion::Plane>()
{
    return m_asset.m_planes;
}

template <>
inline std::vector<Asset<arion::Sphere>>& AssetManager::GetShapes<arion::Sphere>()
{
    return m_asset.m_spheres;
}

template <>
inline std::vector<Asset<arion::Box>>& AssetManager::GetShapes<arion::Box>()
{
    return m_asset.m_boxes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, arion::Plane>()
{
    return m_asset.m_staticPlanes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, arion::Plane>()
{
    return m_asset.m_dynamicPlanes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, arion::Sphere>()
{
    return m_asset.m_staticSpheres;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, arion::Sphere>()
{
    return m_asset.m_dynamicSpheres;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, arion::Box>()
{
    return m_asset.m_staticBoxes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, arion::Box>()
{
    return m_asset.m_dynamicBoxes;
}

template <>
inline std::vector<Asset<force::StaticField>>& AssetManager::GetForces<force::StaticField>()
{
    return m_asset.m_staticFieldForces;
}

template <>
inline std::vector<Asset<force::SquareDistanceSource>>& AssetManager::GetForces<force::SquareDistanceSource>()
{
    return m_asset.m_squareDistanceSourceForces;
}

template <>
inline std::vector<Asset<force::Drag>>& AssetManager::GetForces<force::Drag>()
{
    return m_asset.m_dragForces;
}

template <>
inline std::vector<Asset<force::Spring>>& AssetManager::GetForces<force::Spring>()
{
    return m_asset.m_springForces;
}

template <>
inline std::vector<Asset<force::Bungee>>& AssetManager::GetForces<force::Bungee>()
{
    return m_asset.m_bungeeForces;
}

template <>
inline std::vector<Asset<force::Buoyancy>>& AssetManager::GetForces<force::Buoyancy>()
{
    return m_asset.m_buoyancyForces;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::StaticField>()
{
    return m_asset.m_staticFieldForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::SquareDistanceSource>()
{
    return m_asset.m_squareDistanceSourceForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Drag>()
{
    return m_asset.m_dragForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Spring>()
{
    return m_asset.m_springForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Bungee>()
{
    return m_asset.m_bungeeForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Buoyancy>()
{
    return m_asset.m_buoyancyForceBindings;
}

} // namespace scene
} // namespace pegasus
#endif // PEGASUS_SCENE_ASSET_MANAGER_HPP
