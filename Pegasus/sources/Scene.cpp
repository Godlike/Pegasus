/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Scene.hpp>
#include <pegasus/Debug.hpp>
#include <pegasus/Force.hpp>
#include <pegasus/Integration.hpp>

namespace pegasus
{
namespace scene
{

Scene::Scene()
    : m_detector(m_assetManager)
    , m_resolver(m_assetManager)
{
}

void Scene::ComputeFrame(float duration)
{
    ApplyCollisionCache(duration);

    ApplyForces(forceDuration);

    Integrate(duration);

    ResolveCollisions(duration);
}

Handle Scene::MakeBody()
{
    return m_assetManager.MakeAsset(m_assetManager.GetBodies());
}

mechanics::Body& Scene::GetBody(Handle handle)
{
    return m_assetManager.GetAsset(m_assetManager.GetBodies(), handle);
}

void Scene::RemoveBody(Handle handle)
{
    m_assetManager.RemoveAsset(m_assetManager.GetBodies(), handle);
}

AssetManager& Scene::GetAssets()
{
    return m_assetManager;
}

void Scene::ResolveCollisions(float duration)
{
    std::vector<collision::Contact> contacts = m_detector.Detect();
    Debug::CollisionDetectionCall(contacts);
    m_resolver.Resolve(contacts, duration);
}

void Scene::ApplyCollisionCache(float duration)
{
    m_resolver.ResolvePersistantContacts(duration);
}

void Scene::ApplyForces(float duration)
{
    //Clear previously applied forces
    for (Asset<mechanics::Body>& asset : m_assetManager.GetBodies())
    {
        asset.data.linearMotion.force = glm::vec3(0);
        asset.data.angularMotion.torque = glm::vec3(0);
    }

    //Reapply forces
    ApplyForce<force::StaticField>(duration);
    ApplyForce<force::SquareDistanceSource>(duration);
    ApplyForce<force::Drag>(duration);
    ApplyForce<force::Spring>(duration);
    ApplyForce<force::Bungee>(duration);
    ApplyForce<force::Buoyancy>(duration);
}

void Scene::Integrate(float duration)
{
    for (Asset<mechanics::Body>& asset : m_assetManager.GetBodies())
    {
        if (asset.id != ZERO_HANDLE)
        {
            integration::Integrate(asset.data, duration);
        }
    }

    UpdateShapes<DynamicBody, arion::Plane>();
    UpdateShapes<DynamicBody, arion::Sphere>();
    UpdateShapes<DynamicBody, arion::Box>();
}

} // namespace scene
} // namespace pegasus
