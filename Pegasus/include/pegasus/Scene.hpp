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
namespace scene
{
using Handle = uint32_t;

class Scene
{
public:
    PEGASUS_EXPORT
    inline static Scene& GetInstance();

    PEGASUS_EXPORT
    inline void ComputeFrame(double duration);


    PEGASUS_EXPORT
    inline Handle MakeBody();

    PEGASUS_EXPORT
    inline mechanics::Body& GetBody(Handle handle) ;

    PEGASUS_EXPORT
    inline void RemoveBody(Handle handle);


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
    template < typename Object, typename Shape >
    Handle MakeObject(Handle body, Handle shape);

    PEGASUS_EXPORT
    template < typename Object, typename Shape >
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

    //!Bodies
    std::vector<Asset<mechanics::Body>> m_bodies;

    //!Shapes
    std::vector<Asset<geometry::Plane>> m_planes;
    std::vector<Asset<geometry::Sphere>> m_spheres;
    std::vector<Asset<geometry::Box>> m_boxes;

    //!Objects
    std::vector<Asset<RigidBody>> m_staticPlanes;
	std::vector<Asset<RigidBody>> m_dynamicPlanes;
    std::vector<Asset<RigidBody>> m_staticSpheres;
    std::vector<Asset<RigidBody>> m_dynamicSpheres;
    std::vector<Asset<RigidBody>> m_staticBoxes;
    std::vector<Asset<RigidBody>> m_dynamicBoxes;

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

    Scene() = default;

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

    inline void ResolveCollisions(double duration);

    inline void ApplyForces();

    inline void Integrate(double duration);
};

template <>
inline Handle Scene::MakeShape<geometry::Plane>()
{
    return MakeAsset(m_planes);
}

template <>
inline geometry::Plane& Scene::GetShape<geometry::Plane>(Handle handle)
{
    return GetAsset(m_planes, handle);
}

template <>
inline void Scene::RemoveShape<geometry::Plane>(Handle handle)
{
    RemoveAsset(m_planes, handle);
}

template <>
inline Handle Scene::MakeShape<geometry::Sphere>()
{
    return MakeAsset(m_spheres);
}

template <>
inline geometry::Sphere& Scene::GetShape<geometry::Sphere>(Handle handle)
{
    return GetAsset(m_spheres, handle);
}

template <>
inline void Scene::RemoveShape<geometry::Sphere>(Handle handle)
{
    RemoveAsset(m_spheres, handle);
}

template <>
inline Handle Scene::MakeShape<geometry::Box>()
{
    return MakeAsset(m_boxes);
}

template <>
inline geometry::Box& Scene::GetShape<geometry::Box>(Handle handle)
{
    return GetAsset(m_boxes, handle);
}

template <>
inline void Scene::RemoveShape<geometry::Box>(Handle handle)
{
    RemoveAsset(m_boxes, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::StaticObject, geometry::Plane>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_staticPlanes);
    GetAsset(m_staticPlanes, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::StaticObject, geometry::Plane>(Handle handle)
{
    RemoveAsset(m_staticPlanes, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::StaticObject, geometry::Sphere>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_staticSpheres);
    GetAsset(m_staticSpheres, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::StaticObject, geometry::Sphere>(Handle handle)
{
    RemoveAsset(m_staticSpheres, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::StaticObject, geometry::Box>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_staticBoxes);
    GetAsset(m_staticBoxes, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::StaticObject, geometry::Box>(Handle handle)
{
    RemoveAsset(m_staticBoxes, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::DynamicObject, geometry::Plane>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_dynamicPlanes);
    GetAsset(m_dynamicPlanes, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::DynamicObject, geometry::Plane>(Handle handle)
{
    RemoveAsset(m_dynamicPlanes, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::DynamicObject, geometry::Sphere>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_dynamicSpheres);
    GetAsset(m_dynamicSpheres, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::DynamicObject, geometry::Sphere>(Handle handle)
{
    RemoveAsset(m_dynamicSpheres, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::DynamicObject, geometry::Box>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_dynamicBoxes);
    GetAsset(m_dynamicBoxes, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::DynamicObject, geometry::Box>(Handle handle)
{
    RemoveAsset(m_dynamicBoxes, handle);
}

template <>
inline Handle Scene::MakeForce<force::StaticField>()
{
    return MakeAsset(m_staticFieldForces);
}

template <>
inline force::StaticField& Scene::GetForce<force::StaticField>(Handle handle)
{
    return GetAsset(m_staticFieldForces, handle);
}

template <>
inline void Scene::RemoveForce<force::StaticField>(Handle handle)
{
    RemoveAsset(m_staticFieldForces, handle);
}

template <>
inline Handle Scene::BindForce<force::StaticField>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_staticFieldForceBindings);
    GetAsset(m_staticFieldForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::StaticField>(Handle handle)
{
    RemoveAsset(m_staticFieldForceBindings, handle);
}


template <>
inline Handle Scene::MakeForce<force::Drag>()
{
    return MakeAsset(m_dragForces);
}

template <>
inline force::Drag& Scene::GetForce<force::Drag>(Handle handle)
{
    return GetAsset(m_dragForces, handle);
}

template <>
inline void Scene::RemoveForce<force::Drag>(Handle handle)
{
    RemoveAsset(m_dragForces, handle);
}

template <>
inline Handle Scene::BindForce<force::Drag>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_dragForceBindings);
    GetAsset(m_dragForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::Drag>(Handle handle)
{
    RemoveAsset(m_dragForceBindings, handle);
}

template <>
inline Handle Scene::MakeForce<force::Spring>()
{
    return MakeAsset(m_springForces);
}

template <>
inline force::Spring& Scene::GetForce<force::Spring>(Handle handle)
{
    return GetAsset(m_springForces, handle);
}

template <>
inline void Scene::RemoveForce<force::Spring>(Handle handle)
{
    RemoveAsset(m_springForces, handle);
}

template <>
inline Handle Scene::BindForce<force::Spring>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_springForceBindings);
    GetAsset(m_springForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::Spring>(Handle handle)
{
    RemoveAsset(m_springForceBindings, handle);
}

template <>
inline Handle Scene::MakeForce<force::AnchoredSpring>()
{
    return MakeAsset(m_anchoredSpringForces);
}

template <>
inline force::AnchoredSpring& Scene::GetForce<force::AnchoredSpring>(Handle handle)
{
    return GetAsset(m_anchoredSpringForces, handle);
}

template <>
inline void Scene::RemoveForce<force::AnchoredSpring>(Handle handle)
{
    RemoveAsset(m_anchoredSpringForces, handle);
}

template <>
inline Handle Scene::BindForce<force::AnchoredSpring>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_anchoredSpringForceBindings);
    GetAsset(m_anchoredSpringForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::AnchoredSpring>(Handle handle)
{
    RemoveAsset(m_anchoredSpringForceBindings, handle);
}

template <>
inline Handle Scene::MakeForce<force::Bungee>()
{
    return MakeAsset(m_bungeeForces);
}

template <>
inline force::Bungee& Scene::GetForce<force::Bungee>(Handle handle)
{
    return GetAsset(m_bungeeForces, handle);
}

template <>
inline void Scene::RemoveForce<force::Bungee>(Handle handle)
{
    RemoveAsset(m_bungeeForces, handle);
}

template <>
inline Handle Scene::BindForce<force::Bungee>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_bungeeForceBindings);
    GetAsset(m_bungeeForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::Bungee>(Handle handle)
{
    RemoveAsset(m_bungeeForceBindings, handle);
}

template <>
inline Handle Scene::MakeForce<force::Buoyancy>()
{
    return MakeAsset(m_buoyancyForces);
}

template <>
inline force::Buoyancy& Scene::GetForce<force::Buoyancy>(Handle handle)
{
    return GetAsset(m_buoyancyForces, handle);
}

template <>
inline void Scene::RemoveForce<force::Buoyancy>(Handle handle)
{
    RemoveAsset(m_buoyancyForces, handle);
}

template <>
inline Handle Scene::BindForce<force::Buoyancy>(Handle body, Handle force)
{
    Handle const id = MakeAsset(m_buoyancyForceBindings);
    GetAsset(m_buoyancyForceBindings, id) = { body, force };
    return id;
}

template <>
inline void Scene::UnbindForce<force::Buoyancy>(Handle handle)
{
    RemoveAsset(m_buoyancyForceBindings, handle);
}

class Primitive
{
public:
    enum class Type : uint8_t
    {
        STATIC,
        DYNAMIC
    };

    virtual ~Primitive()
    {
        m_scene->RemoveBody(m_body);
    }

    mechanics::Body& GetBody() const
    {
        return m_scene->GetBody(m_body);
    }

protected:
    Scene* m_scene = nullptr;
    Type m_type = Type::DYNAMIC;
    Handle m_body = 0;
    Handle m_shape = 0;
    Handle m_object = 0;

    Primitive(Type type, mechanics::Body body)
        : m_scene(&Scene::GetInstance())
        , m_type(type)
    {
        m_body = m_scene->MakeBody();
        m_scene->GetBody(m_body) = body;
    }

    template < typename Shape >
    void MakeObject()
    {
        switch (m_type)
        {
            case Type::DYNAMIC:
                m_object = m_scene->MakeObject<mechanics::DynamicObject, Shape>(m_body, m_shape);
                break;
            case Type::STATIC:
                m_object = m_scene->MakeObject<mechanics::StaticObject, Shape>(m_body, m_shape);
                break;
            default:
                break;
        }
    }

    template < typename Shape >
    void RemoveObject() const
    {
        switch (m_type)
        {
        case Type::DYNAMIC:
            m_scene->RemoveObject<mechanics::DynamicObject, Shape>(m_object);
            break;
        case Type::STATIC:
            m_scene->RemoveObject<mechanics::StaticObject, Shape>(m_object);
            break;
        default:
            break;
        }
    }
};

class Plane : public Primitive
{
public:
    Plane(Type type, mechanics::Body body, geometry::Plane plane)
        : Primitive(type, body)
    {
        m_shape = m_scene->MakeShape<geometry::Plane>();
        m_scene->GetShape<geometry::Plane>(m_shape) = plane;
        MakeObject<geometry::Plane>();
    }

    ~Plane()
    {
        m_scene->RemoveShape<geometry::Plane>(m_shape);
        RemoveObject<geometry::Plane>();
    }

    geometry::Plane& GetShape() const
    {
        return m_scene->GetShape<geometry::Plane>(m_shape);
    }
};

class Sphere : public Primitive
{
public:
    Sphere(Type type, mechanics::Body body, geometry::Sphere sphere)
        : Primitive(type, body)
    {
        m_shape = m_scene->MakeShape<geometry::Sphere>();
        m_scene->GetShape<geometry::Sphere>(m_shape) = sphere;
        MakeObject<geometry::Sphere>();
    }

    ~Sphere()
    {
        m_scene->RemoveShape<geometry::Sphere>(m_shape);
        RemoveObject<geometry::Sphere>();
    }

    geometry::Sphere& GetShape() const
    {
        return m_scene->GetShape<geometry::Sphere>(m_shape);
    }
};

class Box : public Primitive
{
public:
    Box(Type type, mechanics::Body body, geometry::Box box)
        : Primitive(type, body)
    {
        m_shape = m_scene->MakeShape<geometry::Box>();
        m_scene->GetShape<geometry::Box>(m_shape) = box;
        MakeObject<geometry::Box>();
    }

    ~Box()
    {
        m_scene->RemoveShape<geometry::Box>(m_shape);
        RemoveObject<geometry::Box>();
    }

    geometry::Box& GetShape() const
    {
        return m_scene->GetShape<geometry::Box>(m_shape);
    }
};
} // namespace scene
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
