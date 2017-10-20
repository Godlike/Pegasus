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
    RigidBody() = default;

    RigidBody(Handle body, Handle shape);

    Handle body;
    Handle shape;
};

struct ForceBind
{
    Handle force;
    Handle body;
};

class AssetManager
{
public:
    static AssetManager& GetInstance();

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

    std::vector<Asset<mechanics::Body>>& GetBodies();

    template < typename Shape >
    std::vector<Asset<Shape>>& GetShapes();

    template < typename Object, typename Shape >
    std::vector<Asset<RigidBody>>& GetObjects();

    template < typename Force >
    std::vector<Asset<Force>>& GetForces();

    template < typename Force >
    std::vector<Asset<ForceBind>>& GetForceBinds();

private:
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

    AssetManager() = default;
};


struct StaticBody : RigidBody
{
    StaticBody(Handle body, Handle shape);
};

struct DynamicBody : RigidBody
{
    DynamicBody(Handle body, Handle shape);
};


template <>
inline std::vector<Asset<geometry::Plane>>& AssetManager::GetShapes<geometry::Plane>()
{
    return m_planes;
}

template <>
inline std::vector<Asset<geometry::Sphere>>& AssetManager::GetShapes<geometry::Sphere>()
{
    return m_spheres;
}

template <>
inline std::vector<Asset<geometry::Box>>& AssetManager::GetShapes<geometry::Box>()
{
    return m_boxes;
}


template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, geometry::Plane>()
{
    return m_staticPlanes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, geometry::Plane>()
{
    return m_dynamicPlanes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, geometry::Sphere>()
{
    return m_staticSpheres;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, geometry::Sphere>()
{
    return m_dynamicSpheres;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<StaticBody, geometry::Box>()
{
    return m_staticBoxes;
}

template <>
inline std::vector<Asset<RigidBody>>& AssetManager::GetObjects<DynamicBody, geometry::Box>()
{
    return m_dynamicBoxes;
}


template <>
inline std::vector<Asset<force::StaticField>>& AssetManager::GetForces<force::StaticField>()
{
    return m_staticFieldForces;
}

template <>
inline std::vector<Asset<force::Drag>>& AssetManager::GetForces<force::Drag>()
{
    return m_dragForces;
}

template <>
inline std::vector<Asset<force::Spring>>& AssetManager::GetForces<force::Spring>()
{
    return m_springForces;
}

template <>
inline std::vector<Asset<force::AnchoredSpring>>& AssetManager::GetForces<force::AnchoredSpring>()
{
    return m_anchoredSpringForces;
}

template <>
inline std::vector<Asset<force::Bungee>>& AssetManager::GetForces<force::Bungee>()
{
    return m_bungeeForces;
}

template <>
inline std::vector<Asset<force::Buoyancy>>& AssetManager::GetForces<force::Buoyancy>()
{
    return m_buoyancyForces;
}


template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::StaticField>()
{
    return m_staticFieldForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Drag>()
{
    return m_dragForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Spring>()
{
    return m_springForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::AnchoredSpring>()
{
    return m_anchoredSpringForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Bungee>()
{
    return m_bungeeForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Buoyancy>()
{
    return m_buoyancyForceBindings;
}

class Scene
{
public:
    PEGASUS_EXPORT
    static Scene& GetInstance();

    PEGASUS_EXPORT
    void ComputeFrame(double duration);


    PEGASUS_EXPORT

    inline Handle MakeBody() const;

    PEGASUS_EXPORT

    inline mechanics::Body& GetBody(Handle handle) const;

    PEGASUS_EXPORT

    inline void RemoveBody(Handle handle) const;

    PEGASUS_EXPORT
    template < typename Shape >
    Handle MakeShape() const
    {
        return m_assets.MakeAsset(m_assets.GetShapes<Shape>());
    }

    PEGASUS_EXPORT
    template < typename Shape >
    Shape& GetShape(Handle handle)
    {
        return m_assets.GetAsset(m_assets.GetShapes<Shape>(), handle);
    }

    PEGASUS_EXPORT
    template < typename Shape >
    void RemoveShape(Handle handle) const
    {
        m_assets.RemoveAsset(m_assets.GetShapes<geometry::Box>(), handle);
    }


    PEGASUS_EXPORT
    template < typename Object, typename Shape >
    Handle MakeObject(Handle body, Handle shape) const
    {
        Handle const id = m_assets.MakeAsset<RigidBody>(m_assets.GetObjects<Object, Shape>());
        m_assets.GetAsset<RigidBody>(m_assets.GetObjects<Object, Shape>(), id) = static_cast<RigidBody>(Object(body, shape));
        return id;
    }

    PEGASUS_EXPORT
    template < typename Object, typename Shape >
    void RemoveObject(Handle handle) const
    {
        m_assets.RemoveAsset(m_assets.GetObjects<Object, Shape>(), handle);
    }


    PEGASUS_EXPORT
    template < typename Force >
    Handle MakeForce() const
    {
        return m_assets.MakeAsset(m_assets.GetForces<Force>());
    }

    PEGASUS_EXPORT
    template < typename Force >
    Force& GetForce(Handle handle)
    {
        return m_assets.GetAsset(m_assets.GetForces<Force>(), handle);
    }

    PEGASUS_EXPORT
    template < typename Force >
    void RemoveForce(Handle handle) const
    {
        m_assets.RemoveAsset(m_assets.GetForces<Force>(), handle);
    }


    PEGASUS_EXPORT
    template < typename Force >
    Handle BindForce(Handle body, Handle force) const
    {
        Handle const id = m_assets.MakeAsset(m_assets.GetForceBinds<Force>());
        m_assets.GetAsset(m_assets.GetForceBinds<Force>(), id) = { body, force };
        return id;
    }

    PEGASUS_EXPORT
    template < typename Force >
    void UnbindForce(Handle handle) const
    {
        m_assets.RemoveAsset(m_assets.GetForceBinds<Force>(), handle);
    }

private:
    AssetManager& m_assets;

    Scene();

    template < typename Force >
    void ApplyForce()
    {
        for (Asset<ForceBind>& asset : m_assets.GetForceBinds<Force>())
        {
            if (asset.id != 0)
            {
                auto& force = GetForce<Force>(asset.data.force);
                auto& body = GetBody(asset.data.body);
                body.linearMotion.force += force.CalculateForce(body);
            }
        }
    }

    template < typename Object, typename Shape >
    void UpdateShapes()
    {
        for (Asset<RigidBody>& asset : m_assets.GetObjects<Object, Shape>())
        {
            if (asset.id != 0)
            {
                mechanics::Body& body = GetBody(asset.data.body);
                Shape& box = GetShape<Shape>(asset.data.shape);
                box.centerOfMass = body.linearMotion.position;
            }
        }
    }

    static inline void ResolveCollisions(double duration);

    inline void ApplyForces();

    inline void Integrate(double duration);
};


class Primitive
{
public:
    enum class Type : uint8_t
    {
        STATIC,
        DYNAMIC
    };

    virtual ~Primitive();

    mechanics::Body& GetBody() const;

protected:
    Scene* m_scene = nullptr;
    Type m_type = Type::DYNAMIC;
    Handle m_body = 0;
    Handle m_shape = 0;
    Handle m_object = 0;

    Primitive(Type type, mechanics::Body body);

    template < typename Shape >
    void MakeObject()
    {
        switch (m_type)
        {
            case Type::DYNAMIC:
                m_object = m_scene->MakeObject<DynamicBody, Shape>(m_body, m_shape);
                break;
            case Type::STATIC:
                m_object = m_scene->MakeObject<StaticBody, Shape>(m_body, m_shape);
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
            m_scene->RemoveObject<DynamicBody, Shape>(m_object);
            break;
        case Type::STATIC:
            m_scene->RemoveObject<StaticBody, Shape>(m_object);
            break;
        default:
            break;
        }
    }
};

class Plane : public Primitive
{
public:
    Plane(Type type, mechanics::Body body, geometry::Plane plane);

    virtual ~Plane();

    geometry::Plane& GetShape() const;
};

class Sphere : public Primitive
{
public:
    Sphere(Type type, mechanics::Body body, geometry::Sphere sphere);

    virtual ~Sphere();

    geometry::Sphere& GetShape() const;
};

class Box : public Primitive
{
public:
    Box(Type type, mechanics::Body body, geometry::Box box);

    virtual ~Box();

    geometry::Box& GetShape() const;
};

} // namespace scene
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
