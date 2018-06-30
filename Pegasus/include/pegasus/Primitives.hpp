/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_PRIMITIVES_HPP
#define PEGASUS_SCENE_PRIMITIVES_HPP

#include <pegasus/Scene.hpp>

namespace pegasus
{
namespace scene
{

/**
 * @brief Generic scene object that stores reference to the scene
 */
class SceneObject
{
public:
    SceneObject(Scene& scene)
        : m_pScene(&scene)
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
     * @param scene scene instance
     * @param type body type
     * @param body data
     */
    Primitive(Scene& scene, Type type, mechanics::Body body);

    /**
     * @brief Initializes object with primitive data based on body type
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
     * @brief Removes scene primitive and invalidates its handle
     * @tparam Shape collision geometry shape type
     */
    template < typename Shape >
    void RemoveObject()
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

        m_objectHandle = ZERO_HANDLE;
    }

    /**
     * @brief Initializes shape data with corresponding body data
     * @param[in, out]  shape shape data
     * @param[in]       body body data
     */
    static void InitializeShape(arion::SimpleShape& shape, mechanics::Body const& body)
    {
        shape.centerOfMass = body.linearMotion.position;
        shape.orientation = body.angularMotion.orientation;
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
     * @param scene scene instance
     * @param type primitive body type
     * @param body physical body data
     * @param plane shape data
     */
    Plane(Scene& scene, Type type, mechanics::Body body, arion::Plane plane);

    /**
     * @brief Releases shape handle and object handle
     */
    virtual ~Plane();

    /**
     * @brief Returns reference to the plane instance
     * @return shape instance
     */
    arion::Plane& GetShape() const;
};

/**
* @brief Stores sphere geometry shape physical data
*/
class Sphere : public Primitive
{
public:
    /**
     * @brief Makes new scene sphere body
     * @param scene scene instance
     * @param type primitive body type
     * @param body physical body data
     * @param sphere shape data
     */
    Sphere(Scene& scene, Type type, mechanics::Body body, arion::Sphere sphere);

    /**
     * @brief Releases shape handle and body handle
     */
    virtual ~Sphere();

    /**
     * @brief Return reference to the sphere instance
     * @return shape instance
     */
    arion::Sphere& GetShape() const;
};

/**
* @brief Stores box geometry shape physical data
*/
class Box : public Primitive
{
public:
    /**
     * @brief Makes new scene box primitive
     * @param scene scene instance
     * @param type body type
     * @param body physical body data
     * @param box shape data
     */
    Box(Scene& scene, Type type, mechanics::Body body, arion::Box box);

    /**
     * @brief Releases shape handle and object handle
     */
    virtual ~Box();

    /**
     * @brief Returns reference to the box instance
     * @return shape instance
     */
    arion::Box& GetShape() const;
};

/**
 * @brief Defines force bind interface
 */
class ForceBase : public SceneObject
{
public:
    ForceBase(Scene& scene)
        : SceneObject(scene)
    {
    }

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
 *
 * @attention This class works as a high level API over the Scene instance.
 * Only the handles are guaranteed to be valid. All pointers are invalidated
 * after a new instance of a Force class is created or deleted.
 *
 * @tparam ForceType force type
 */
template < typename ForceType >
class Force final : public ForceBase
{
public:
    /**
     * @brief Allocates new force in the scene and initializes it
     * @param scene scene instance
     * @param force initialized force data
     */
    Force(Scene& scene, ForceType force)
        : ForceBase(scene)
        , m_force(force)
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
     *
     * @attention This class works as a high level API over the Scene instance.
     * Only the handles are guaranteed to be valid. Any reference/pointer is invalidated
     * after a new instance of a Force class is created or deleted.
     *
     * @return force data reference
     */
    ForceType& GetForce()
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
    void BindForce(ForceBase& force);

    /**
     * @brief Removes force from the group and unbinds all bodies
     */
    void UnbindForce(ForceBase& force);

    /**
     * @brief Adds primitive pointer to the group and binds all group's forces to it
     * @param primitive reference to the primitive to bind
     */
    void BindPrimitive(Primitive& primitive);

    /**
     * @brief Removes primitive from the group and unbinds all group's forces from it
     * @param primitive reference to the primitive to unbind
     */
    void UnbindPrimitive(Primitive& primitive);

private:
    std::vector<ForceBase*> m_forces;
    std::vector<Primitive*> m_primitives;
};

}
}

#endif // PEGASUS_SCENE_PRIMITIVES_HPP
