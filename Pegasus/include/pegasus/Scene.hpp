/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_HPP
#define PEGASUS_SCENE_HPP

#include <pegasus/Body.hpp>
#include <pegasus/Asset.hpp>
#include <pegasus/AssetManager.hpp>
#include <pegasus/Force.hpp>
#include <pegasus/CollisionDetector.hpp>
#include <pegasus/CollisionResolver.hpp>

namespace pegasus
{
namespace scene
{

/**
 * @brief Physical world simulation class
 */
class Scene
{
public:
    /**
     * @brief Runs physical simulation with the given duration
     * @param duration delta time of the integration in seconds
     */
    void ComputeFrame(float duration);

    /**
     * @brief Makes new body instance and returns its handle
     * @return new body handle
     */
    Handle MakeBody();

    /**
     * @brief Returns instance of the body assigned to the given handle
     * @param handle body handle
     * @return body instance
     */
    mechanics::Body& GetBody(Handle handle);

    /**
     * @brief Removes instance of the body assigned to the given handle
     * @param handle body handle
     */
    void RemoveBody(Handle handle);

    /**
     * @brief Makes new shape instance and returns its handle
     * @tparam Shape collision geometry shape type
     * @return shape handle
     */
    template < typename Shape >
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
    template < typename Shape >
    Shape& GetShape(Handle handle)
    {
        return m_assetManager.GetAsset(m_assetManager.GetShapes<Shape>(), handle);
    }

    /**
     * @brief Removes instance of the shape assigned to the given handle
     * @tparam Shape collision geometry shape type
     * @param handle shape handle
     */
    template < typename Shape >
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
    template < typename Object, typename Shape >
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
    template < typename Object, typename Shape >
    void RemoveObject(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetObjects<Object, Shape>(), handle);
    }

    /**
     * @brief Makes new instance of the force
     * @tparam Force type of the force
     * @return force handle
     */
    template < typename Force >
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
    template < typename Force >
    Force& GetForce(Handle handle)
    {
        return m_assetManager.GetAsset(m_assetManager.GetForces<Force>(), handle);
    }

    /**
     * @brief Remove instance of the force assigned to the given handle
     * @tparam Force type of the force
     * @param handle force handle
     */
    template < typename Force >
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
    template < typename Force >
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
    template < typename Force >
    void UnbindForce(Handle handle)
    {
        m_assetManager.RemoveAsset(m_assetManager.GetForceBinds<Force>(), handle);
    }

    /**
     * @brief Returns reference to the current asset manager
     */
    AssetManager& GetAssets();

    float forceDuration = 1.0f;

private:
    AssetManager m_assetManager;
    std::vector<collision::Contact> m_previousContacts;
    std::vector<collision::Contact> m_persistentContacts;
    std::vector<collision::Contact> m_currentContacts;

    /**
     * @brief Calculates force applied to the bound bodies
     * @tparam Force type of the force
     */
    template < typename Force >
    void ApplyForce(float duration)
    {
        for (Asset<ForceBind>& asset : m_assetManager.GetForceBinds<Force>())
        {
            if (asset.id != ZERO_HANDLE)
            {
                auto& force = GetForce<Force>(asset.data.force);
                mechanics::Body& body = GetBody(asset.data.body);
                body.linearMotion.force += force.CalculateForce(body) * duration;
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
     * @brief Applies all registered forces to the bodies
     */
    void ApplyForces(float duration);

    /**
     * @brief Updates physical properties of the bodies within given duration
     * @param duration delta time of the frame
     */
    void Integrate(float duration);
};

template <>
inline void Scene::ApplyForce<force::Drag>(float duration)
{
    for (Asset<ForceBind>& asset : m_assetManager.GetForceBinds<force::Drag>())
    {
        if (asset.id != ZERO_HANDLE)
        {
            auto& force = GetForce<force::Drag>(asset.data.force);
            mechanics::Body& body = GetBody(asset.data.body);
            body.linearMotion.force += force.CalculateForce(body) * duration;

            glm::vec3 const velocity = body.linearMotion.velocity;
            body.linearMotion.velocity = body.angularMotion.velocity;
            body.angularMotion.torque += force.CalculateForce(body) * duration;
            body.linearMotion.velocity = velocity;

            {
                if (epona::fp::IsZero(body.angularMotion.torque.x))
                    body.angularMotion.torque.x = 0;
                if (epona::fp::IsZero(body.angularMotion.torque.y))
                    body.angularMotion.torque.y = 0;
                if (epona::fp::IsZero(body.angularMotion.torque.z))
                    body.angularMotion.torque.z = 0;
            }
        }
    }
}

} // namespace scene
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
