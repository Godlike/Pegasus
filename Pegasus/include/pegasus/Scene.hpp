/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_HPP
#define PEGASUS_SCENE_HPP

#include <pegasus/SharedMacros.hpp>
#include <pegasus/Body.hpp>
#include <pegasus/Asset.hpp>
#include <pegasus/AssetManager.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Force.hpp>

namespace pegasus
{
namespace scene
{

/**
 * @brief Singleton physical world simulation class
 */
class Scene
{
public:
    /**
     * @brief Constructs default scene object
     */
    Scene();

    /**
     * @brief Runs physical simulation with the given duration
     * @param duration delta time of the integration in seconds
     */
    PEGASUS_EXPORT void ComputeFrame(double duration);

    /**
     * @brief Makes new body instance and returns its handle
     * @return new body handle
     */
    PEGASUS_EXPORT Handle MakeBody();

    /**
     * @brief Returns instance of the body assigned to the given handle
     * @param handle body handle
     * @return body instance
     */
    PEGASUS_EXPORT mechanics::Body& GetBody(Handle handle);

    /**
     * @brief Removes instance of the body assigned to the given handle
     * @param handle body handle
     */
    PEGASUS_EXPORT void RemoveBody(Handle handle);

    /**
     * @brief Makes new shape instance and returns its handle
     * @tparam Shape collision geometry shape type
     * @return shape handle
     */
    PEGASUS_EXPORT template < typename Shape >
    Handle MakeShape()
    {
        return m_assetManager.MakeAsset(m_assetManager.GetShapes<Shape>());
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
        return m_assetManager.GetAsset(m_assetManager.GetShapes<Shape>(), handle);
    }

    /**
     * @brief Removes instance of the shape assigned to the given handle
     * @tparam Shape collision geometry shape type
     * @param handle shape handle
     */
    PEGASUS_EXPORT template < typename Shape >
    void RemoveShape(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetShapes<Shape>(), handle);
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
    Handle MakeObject(Handle body, Handle shape)
    {
        Handle const id = m_assetManager.MakeAsset<RigidBody>(m_assetManager.GetObjects<Object, Shape>());
        m_assetManager.GetAsset<RigidBody>(m_assetManager.GetObjects<Object, Shape>(), id) = static_cast<RigidBody>(Object(*this, body, shape));
        return id;
    }

    /**
     * @brief Removes instance of the rigid body assigned to the given handle
     * @tparam Object body type
     * @tparam Shape collision geometry shape type
     * @param handle rigid body handle
     */
    PEGASUS_EXPORT template < typename Object, typename Shape >
    void RemoveObject(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetObjects<Object, Shape>(), handle);
    }

    /**
     * @brief Makes new instance of the force
     * @tparam Force type of the force
     * @return force handle
     */
    PEGASUS_EXPORT template < typename Force >
    Handle MakeForce()
    {
        return m_assetManager.MakeAsset(m_assetManager.GetForces<Force>());
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
        return m_assetManager.GetAsset(m_assetManager.GetForces<Force>(), handle);
    }

    /**
     * @brief Remove instance of the force assigned to the given handle
     * @tparam Force type of the force
     * @param handle force handle
     */
    PEGASUS_EXPORT template < typename Force >
    void RemoveForce(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetForces<Force>(), handle);
    }

    /**
     * @brief Makes force body bind and returns its handle
     * @tparam Force type of the force
     * @param body handle of the body
     * @param force handle of the force
     * @return bind handle
     */
    PEGASUS_EXPORT template < typename Force >
    Handle BindForce(Handle body, Handle force)
    {
        Handle const id = m_assetManager.MakeAsset(m_assetManager.GetForceBinds<Force>());
        m_assetManager.GetAsset(m_assetManager.GetForceBinds<Force>(), id) = { body, force };
        return id;
    }

    /**
     * @brief Removes force body bind assigned to the given handle
     * @tparam Force type of the force
     * @param handle bind handle
     */
    PEGASUS_EXPORT template < typename Force >
    void UnbindForce(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetForceBinds<Force>(), handle);
    }

    AssetManager& GetAssets();

private:
    AssetManager m_assetManager;
    collision::Detector m_detector;
    collision::Resolver m_resolver;

    /**
     * @brief Calculates force applied to the binded bodies
     * @tparam Force type of the force
     */
    template < typename Force >
    void ApplyForce()
    {
        for (Asset<ForceBind>& asset : m_assetManager.GetForceBinds<Force>())
        {
            if (asset.id != ZERO_HANDLE)
            {
                auto& force = GetForce<Force>(asset.data.force);
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
        for (Asset<RigidBody>& asset : m_assetManager.GetObjects<Object, Shape>())
        {
            if (asset.id != ZERO_HANDLE)
            {
                mechanics::Body& body = GetBody(asset.data.body);
                auto& shape = GetShape<Shape>(asset.data.shape);
                shape.centerOfMass = body.linearMotion.position;
                shape.orientation = body.angularMotion.orientation;
            }
        }
    }

    /**
     * @brief Resolves registered collisions
     * @param duration delta time of the frame
     */
    void ResolveCollisions(double duration);

    /**
     * @brief Applies initial impulses to resolve cached persistent contacts
     */
    void ApplyCollisionCache();

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

template <>
inline void Scene::ApplyForce<force::Drag>()
{
    for (Asset<ForceBind>& asset : m_assetManager.GetForceBinds<force::Drag>())
    {
        if (asset.id != ZERO_HANDLE)
        {
            auto& force = GetForce<force::Drag>(asset.data.force);
            mechanics::Body& body = GetBody(asset.data.body);
            body.linearMotion.force += force.CalculateForce(body);

            glm::dvec3 const velocity = body.linearMotion.velocity;
            body.linearMotion.velocity = body.angularMotion.velocity;
            body.angularMotion.torque += force.CalculateForce(body);
            body.linearMotion.velocity = velocity;
        }
    }
}

} // namespace scene
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
