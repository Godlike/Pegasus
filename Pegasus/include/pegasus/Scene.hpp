/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_HPP
#define PEGASUS_SCENE_HPP

#include <pegasus/SharedMacros.hpp>
#include <pegasus/Body.hpp>
#include <pegasus/Force.hpp>
#include <geometry/Shape.hpp>

#include <vector>
#include <unordered_map>
#include <algorithm>

namespace pegasus
{
namespace scene
{
using Handle = uint32_t;

/**
 * @brief Stores handles to the physical body and collision geometry
 */
struct RigidBody
{
    RigidBody() = default;

    RigidBody(Handle body, Handle shape);

    Handle body = 0;
    Handle shape = 0;
};

/**
 * @brief Represents a body with an infinite mass
 */
struct StaticBody : RigidBody
{
    StaticBody(Handle body, Handle shape);
};

/**
 * @brief Represents a body with a finite mass
 */
struct DynamicBody : RigidBody
{
    DynamicBody(Handle body, Handle shape);
};

/**
 * @brief Stores handles to a physical body and a binded force
 */
struct ForceBind
{
    Handle body = 0;
    Handle force = 0;
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

/**
 * @brief Singleton class stores scene's assets
 */
class AssetManager
{
public:
    /**
     * @brief Returns the instance of the asset manager
     * @return asset manager
     */
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
    std::vector<Asset<force::Bungee>> m_bungeeForces;
    std::vector<Asset<force::Buoyancy>> m_buoyancyForces;

    //!ForceBind bindings
    std::vector<Asset<ForceBind>> m_staticFieldForceBindings;
    std::vector<Asset<ForceBind>> m_dragForceBindings;
    std::vector<Asset<ForceBind>> m_springForceBindings;
    std::vector<Asset<ForceBind>> m_bungeeForceBindings;
    std::vector<Asset<ForceBind>> m_buoyancyForceBindings;

    AssetManager() = default;
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
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Bungee>()
{
    return m_bungeeForceBindings;
}

template <>
inline std::vector<Asset<ForceBind>>& AssetManager::GetForceBinds<force::Buoyancy>()
{
    return m_buoyancyForceBindings;
}

/**
 * @brief Singleton physical world simulation class
 */
class Scene
{
public:
    /**
    * @brief Returns singleton instance of the scene
    * @return scene instance
    */
    PEGASUS_EXPORT static Scene& GetInstance();

    /**
     * @brief Performs scene initialization
     * @param assetManager scene asset holder
     */
    PEGASUS_EXPORT void Initialize(AssetManager& assetManager);

    /**
     * @brief Runs physical simulation with the given duration
     * @param duration delta time of the integration in seconds
     */
    PEGASUS_EXPORT void ComputeFrame(double duration);

    /**
     * @brief Makes new body instance and returns its handle
     * @return new body handle
     */
    PEGASUS_EXPORT Handle MakeBody() const;

    /**
     * @brief Returns instance of the body assigned to the given handle
     * @param handle body handle
     * @return body instance
     */
    PEGASUS_EXPORT mechanics::Body& GetBody(Handle handle) const;

    /**
     * @brief Removes instance of the body assigned to the given handle
     * @param handle body handle
     */
    PEGASUS_EXPORT void RemoveBody(Handle handle) const;

    /**
     * @brief Makes new shape instance and returns its handle
     * @tparam Shape collision geometry shape type
     * @return shape handle
     */
    PEGASUS_EXPORT template < typename Shape >
    Handle MakeShape() const
    {
        return m_assetManager->MakeAsset(m_assetManager->GetShapes<Shape>());
    }

    /**
     * @brief Returns instance of the shape assigned to the given handle
     * @tparam Shape collision geometry shape type
     * @param handle shape handle
     * @return shape instance
     */
    PEGASUS_EXPORT template < typename Shape >
    Shape& GetShape(Handle handle)
    {
        return m_assetManager->GetAsset(m_assetManager->GetShapes<Shape>(), handle);
    }

    /**
     * @brief Removes instance of the shape assigned to the given handle
     * @tparam Shape collision geometry shape type
     * @param handle shape handle
     */
    PEGASUS_EXPORT template < typename Shape >
    void RemoveShape(Handle handle) const
    {
        m_assetManager->RemoveAsset(m_assetManager->GetShapes<Shape>(), handle);
    }

    /**
     * @brief Makes rigid body from a shape and a point mass
     * @tparam Object body type
     * @tparam Shape collision geometry shape type
     * @param body point mass handle
     * @param shape handle of the shape
     * @return handle of the rigid body
     */
    PEGASUS_EXPORT template < typename Object, typename Shape >
    Handle MakeObject(Handle body, Handle shape) const
    {
        Handle const id = m_assetManager->MakeAsset<RigidBody>(m_assetManager->GetObjects<Object, Shape>());
        m_assetManager->GetAsset<RigidBody>(m_assetManager->GetObjects<Object, Shape>(), id) = static_cast<RigidBody>(Object(body, shape));
        return id;
    }

    /**
     * @brief Removes instance of the rigid body assigned to the given handle
     * @tparam Object body type
     * @tparam Shape collision geometry shape type
     * @param handle rigid body handle
     */
    PEGASUS_EXPORT template < typename Object, typename Shape >
    void RemoveObject(Handle handle) const
    {
        m_assetManager->RemoveAsset(m_assetManager->GetObjects<Object, Shape>(), handle);
    }

    /**
     * @brief Makes new instance of the force
     * @tparam Force type of the force
     * @return force handle
     */
    PEGASUS_EXPORT template < typename Force >
    Handle MakeForce() const
    {
        return m_assetManager->MakeAsset(m_assetManager->GetForces<Force>());
    }

    /**
     * @brief Returns an instance of the force assigned to the given handle
     * @tparam Force type of the force
     * @param handle force handle
     * @return force instance
     */
    PEGASUS_EXPORT template < typename Force >
    Force& GetForce(Handle handle)
    {
        return m_assetManager->GetAsset(m_assetManager->GetForces<Force>(), handle);
    }

    /**
     * @brief Remove instance of the force assigned to the given handle
     * @tparam Force type of the force
     * @param handle force handle
     */
    PEGASUS_EXPORT template < typename Force >
    void RemoveForce(Handle handle) const
    {
        m_assetManager->RemoveAsset(m_assetManager->GetForces<Force>(), handle);
    }

    /**
     * @brief Makes force body bind and returns its handle
     * @tparam Force type of the force
     * @param body handle of the body
     * @param force handle of the force
     * @return bind handle
     */
    PEGASUS_EXPORT template < typename Force >
    Handle BindForce(Handle body, Handle force) const
    {
        Handle const id = m_assetManager->MakeAsset(m_assetManager->GetForceBinds<Force>());
        m_assetManager->GetAsset(m_assetManager->GetForceBinds<Force>(), id) = { body, force };
        return id;
    }

    /**
     * @brief Removes force body bind assigned to the given handle
     * @tparam Force type of the force
     * @param handle bind handle
     */
    PEGASUS_EXPORT template < typename Force >
    void UnbindForce(Handle handle) const
    {
        m_assetManager->RemoveAsset(m_assetManager->GetForceBinds<Force>(), handle);
    }

private:
    AssetManager* m_assetManager;

    Scene() = default;

    /**
     * @brief Calculates force applied to the binded bodies
     * @tparam Force type of the force
     */
    template < typename Force >
    void ApplyForce()
    {
        for (Asset<ForceBind>& asset : m_assetManager->GetForceBinds<Force>())
        {
            if (asset.id != 0)
            {
                Force& force = GetForce<Force>(asset.data.force);
                mechanics::Body& body = GetBody(asset.data.body);
                body.linearMotion.force += force.CalculateForce(body);
            }
        }
    }

    /**
     * @brief Synchronizes collision geometry and point mass positions
     * @tparam Object body type
     * @tparam Shape collision geometry shape type
     */
    template < typename Object, typename Shape >
    void UpdateShapes()
    {
        for (Asset<RigidBody>& asset : m_assetManager->GetObjects<Object, Shape>())
        {
            if (asset.id != 0)
            {
                mechanics::Body& body = GetBody(asset.data.body);
                Shape& box = GetShape<Shape>(asset.data.shape);
                box.centerOfMass = body.linearMotion.position;
            }
        }
    }

    /**
     * @brief Resolves registered collisions
     * @param duration delta time of the frame
     */
    void ResolveCollisions(double duration) const;

    /**
     * @brief Applies all registered forces to the bodies
     */
    void ApplyForces();

    /**
     * @brief Updates physical properties of the bodies within given duration
     * @param duration delta time of the frame
     */
    void Integrate(double duration);
};


/**
 * @brief Generic scene object that stores reference to the scene singleton
 */
class SceneObject
{
public:
    SceneObject()
        : m_pScene(&Scene::GetInstance())
    {
    }

    virtual ~SceneObject() = default;

protected:
    Scene* m_pScene = nullptr;
};

/**
 * @brief Generic primitive type, stores geometric and physical handles to the body
 */
class Primitive : public SceneObject
{
public:
    enum class Type : uint8_t
    {
        STATIC,
        DYNAMIC
    };

    /**
     * @brief Releases body handle
     */
    virtual ~Primitive();

    /**
     * @brief Sets body data
     * @param body physical data
     */
    void SetBody(mechanics::Body body) const;

    /**
     * @brief Returns body data
     * @return body data
     */
    mechanics::Body GetBody() const;

    /**
     * @brief Returns scene handle to the body
     * @return body handle
     */
    Handle GetBodyHandle() const;

    /**
     * @brief Returns scene handle to the shape
     * @return shape handle
     */
    Handle GetShapeHandle() const;

    /**
     * @brief Return scene rigid body handle
     * @return rigid body handle
     */
    Handle GetObjectHandle() const;

protected:
    Type m_type = Type::DYNAMIC;
    Handle m_bodyHandle = 0;
    Handle m_shapeHandle = 0;
    Handle m_objectHandle = 0;

    /**
     * @brief Allocates body in the scene and initializes
     * @param type body type
     * @param body data
     */
    Primitive(Type type, mechanics::Body body);

    /**
     * @brief Makes new scene primitive
     * @tparam Shape collision geometry shape type
     */
    template < typename Shape >
    void MakeObject()
    {
        switch (m_type)
        {
            case Type::DYNAMIC:
                m_objectHandle = m_pScene->MakeObject<DynamicBody, Shape>(m_bodyHandle, m_shapeHandle);
                break;
            case Type::STATIC:
                m_objectHandle = m_pScene->MakeObject<StaticBody, Shape>(m_bodyHandle, m_shapeHandle);
                break;
            default:
                break;
        }
    }

    /**
     * @brief Removes scene primitive
     * @tparam Shape collision geometry shape type
     */
    template < typename Shape >
    void RemoveObject() const
    {
        switch (m_type)
        {
        case Type::DYNAMIC:
            m_pScene->RemoveObject<DynamicBody, Shape>(m_objectHandle);
            break;
        case Type::STATIC:
            m_pScene->RemoveObject<StaticBody, Shape>(m_objectHandle);
            break;
        default:
            break;
        }
    }
};

/**
 * @brief Stores plane geometry shape physical data
 */
class Plane : public Primitive
{
public:
    /**
     * @brief Makes new scene plane object
     * @param type primitive body type
     * @param body physical body data
     * @param plane shape data
     */
    Plane(Type type, mechanics::Body body, geometry::Plane plane);

    /**
     * @brief Releases shape handle and object handle
     */
    virtual ~Plane();

    /**
     * @brief Returns reference to the plane instance
     * @return shape instance
     */
    geometry::Plane& GetShape() const;
};

/**
* @brief Stores sphere geometry shape physical data
*/
class Sphere : public Primitive
{
public:
    /**
     * @brief Makes new scene sphere body
     * @param type primitive body type
     * @param body physical body data
     * @param sphere shape data
     */
    Sphere(Type type, mechanics::Body body, geometry::Sphere sphere);

    /**
     * @brief Releases shape handle and body handle
     */
    virtual ~Sphere();

    /**
     * @brief Return reference to the sphere instance
     * @return sphere instance
     */
    geometry::Sphere& GetShape() const;
};

/**
* @brief Stores box geometry shape physical data
*/
class Box : public Primitive
{
public:
    /**
     * @brief Makes new scene box primitive
     * @param type body type
     * @param body physical body data
     * @param box shape data
     */
    Box(Type type, mechanics::Body body, geometry::Box box);

    /**
     * @brief Releases shape handle and object handle
     */
    virtual ~Box();

    /**
     * @brief Returns reference to the box instance
     * @return box shape
     */
    geometry::Box& GetShape() const;
};

/**
 * @brief Defines force bind interface
 */
class ForceBase : public SceneObject
{
public:
    /**
     * @brief Binds primitive's body to the force
     * @param[in] primitive body to be bound
     */
    virtual void Bind(Primitive const& primitive) = 0;

    /**
     * @brief Unbinds primitive's body from the force
     * @param[in] primitive body to be unbound
     */
    virtual void Unbind(Primitive const& primitive) = 0;
};

/**
 * @brief Stores force data
 * @tparam ForceType force type
 */
template < typename ForceType >
class Force : public ForceBase
{
public:
    /**
     * @brief Allocates new force in the scene and initializes it
     * @param force initialized force data
     */
    Force(ForceType force)
        : m_force(force)
    {
        m_forceHandle = m_pScene->MakeForce<ForceType>();
        SetForce(force);
    }

    /**
     * @brief Unbinds all bodies and releases force handle
     */
    virtual ~Force()
    {
        for (std::pair<Handle, Handle> const bind : m_bodyForceBinds)
        {
            m_pScene->UnbindForce<ForceType>(bind.second);
        }
        m_pScene->RemoveForce<ForceType>(m_forceHandle);
    }

    /**
     * @brief Sets force data
     * @param force initialized force data
     */
    void SetForce(ForceType force)
    {
        m_pScene->GetForce<ForceType>(m_forceHandle) = force;
    }

    /**
     * @brief Returns force data
     * @return force data
     */
    ForceType GetForce() const
    {
        return m_pScene->GetForce<ForceType>(m_forceHandle);
    }

    /**
     * @brief Bind force to the given body
     * @param primitive body to be bound
     */
    void Bind(Primitive const& primitive) override
    {
        Handle const body = primitive.GetBodyHandle();

        if (m_bodyForceBinds.find(body) == m_bodyForceBinds.end())
        {
            m_bodyForceBinds[body] = m_pScene->BindForce<ForceType>(body, m_forceHandle);
        }
    }

    /**
     * @brief Unbinds force from the body and releases bind handle
     * @param primitive body to be unbound
     */
    void Unbind(Primitive const& primitive) override
    {
        Handle const body = primitive.GetBodyHandle();

        if (m_bodyForceBinds.find(body) != m_bodyForceBinds.end())
        {
            m_pScene->UnbindForce<ForceType>(m_bodyForceBinds[body]);
            m_bodyForceBinds.erase(body);
        }
    }

private:
    std::unordered_map<Handle, Handle> m_bodyForceBinds;
    Handle m_forceHandle;
    ForceType m_force;
};

/**
 * @brief Allows multiple similar force bindings
 */
class PrimitiveGroup
{
public:
    /**
     * @brief Adds force pointer to the group and binds the force to all group's bodies
     * @param force reference to the force
     */
    void BindForce(ForceBase& force)
    {
        m_forces.push_back(&force);

        for (Primitive* primitive : m_primitives)
        {
            force.Bind(*primitive);
        }
    }

    /**
     * @brief Removes force from the group and unbinds all bodies
     */
    void UnbindForce(ForceBase& force)
    {
        for (Primitive* primitive : m_primitives)
        {
            force.Unbind(*primitive);
        }

        m_forces.erase(
            std::remove_if(m_forces.begin(), m_forces.end(),
                [&force](ForceBase* f) { return &force == f; }),
            m_forces.end()
        );
    }

    /**
     * @brief Adds primitive pointer to the group and binds all group's forces to it
     * @param primitive reference to the primitive to bind
     */
    void BindPrimitive(Primitive& primitive)
    {
        m_primitives.push_back(&primitive);

        for (ForceBase* force : m_forces)
        {
            force->Bind(primitive);
        }
    }

    /**
     * @brief Removes body from the group and unbinds all group's forces from it
     * @param primitive reference to the primitive to unbind
     */
    void UnbindPrimitive(Primitive& primitive)
    {
        for (ForceBase* force : m_forces)
        {
            force->Unbind(primitive);
        }

        m_primitives.erase(
            std::remove_if(m_primitives.begin(), m_primitives.end(),
                [&primitive](Primitive* p) { return p == &primitive; }),
            m_primitives.end()
        );
    }

private:
    std::vector<ForceBase*> m_forces;
    std::vector<Primitive*> m_primitives;
};

} // namespace scene
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
