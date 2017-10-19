/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_HPP
#define PEGASUS_SCENE_HPP

#include <pegasus/Object.hpp>
#include <pegasus/Force.hpp>
#include <pegasus/SharedMacros.hpp>

#include <vector>

namespace pegasus
{
using Handle = uint32_t;

class Scene
{
public:
    PEGASUS_EXPORT
    void ComputeFrame(double duration);


    PEGASUS_EXPORT
    Handle MakeBody();

    PEGASUS_EXPORT
    mechanics::Body& GetBody(Handle handle) ;

    PEGASUS_EXPORT
    void RemoveBody(Handle handle);


    PEGASUS_EXPORT
    template < typename Shape >
    Handle MakeShape();

    PEGASUS_EXPORT
    template < typename Shape >
    Shape& GetShape(Handle handle);

    PEGASUS_EXPORT
    template < typename Shape >
    void RemoveShape(Handle handle);


    PEGASUS_EXPORT
    template < typename Object >
    Handle MakeObject(Handle body, Handle shape);

    PEGASUS_EXPORT
    template < typename Object >
    void RemoveObject(Handle handle);


    PEGASUS_EXPORT
    template < typename Force >
    Handle MakeForce();

    PEGASUS_EXPORT
    template < typename Force >
    Force& GetForce(Handle handle);

    PEGASUS_EXPORT
    template < typename Force >
    void RemoveForce(Handle handle);


    PEGASUS_EXPORT
    template < typename Force >
    Handle BindForce(Handle body, Handle force);

    PEGASUS_EXPORT
    template < typename Force >
    void UnbindForce(Handle handle);

private:
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

    struct RigidBody
    {
        Handle body;
        Handle shape;
    };

    struct ForceBind
    {
        Handle force;
        Handle body;
    };

    //!Bodies
    std::vector<Asset<mechanics::Body>> m_bodies;

    //!Shapes
    std::vector<Asset<geometry::Plane>> m_planes;
    std::vector<Asset<geometry::Sphere>> m_spheres;
    std::vector<Asset<geometry::Box>> m_boxes;

    //!Objects
    std::vector<Asset<RigidBody>> m_staticObjects;
	std::vector<Asset<RigidBody>> m_dynamicObjects;

    //!Forces
    std::vector<Asset<force::StaticField>> m_staticFieldForces;
    std::vector<Asset<force::Drag>> m_dragForces;
    std::vector<Asset<force::Spring>> m_springForces;
    std::vector<Asset<force::AnchoredSpring>> m_anchoredSpringForces;
    std::vector<Asset<force::Bungee>> m_bungeeForces;
    std::vector<Asset<force::Buoyancy>> m_buoyancyForces;

    //!ForceBind bindings
    std::vector<Asset<ForceBind>> m_staticFieldForceBindings;
    std::vector<Asset<ForceBind>> m_dragForceBindings;
    std::vector<Asset<ForceBind>> m_springForceBindings;
    std::vector<Asset<ForceBind>> m_anchoredSpringForceBindings;
    std::vector<Asset<ForceBind>> m_bungeeForceBindings;
    std::vector<Asset<ForceBind>> m_buoyancyForceBindings;

    /**
     * @brief Returns a handle for @p T data
     *
     * Searches for an empty handle in @p data. If there are no empty handles, creates a new one.
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
            if (data[i].id == 0)
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

    template < typename Force >
    void ApplyForce(std::vector<Asset<Force>>& forces)
    {
        for (Asset<ForceBind>& asset : forces)
        {
            if (asset.id != 0)
            {
                auto& force = GetForce<Force>(asset.data.force);
                auto& body = GetBody(asset.data.body);
                body.linearMotion.force += force.CalculateForce(body);
            }
        }
    }

    void ResolveCollisions(double duration);
    
    void ApplyForces();

    void Integrate(double duration);
};
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
