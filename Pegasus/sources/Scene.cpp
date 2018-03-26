/*
* Copyright (C) 2017 by Godlike
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

void Scene::ComputeFrame(double duration)
{
    ApplyCollisionCache();

    ApplyForces();

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

void Scene::ResolveCollisions(double duration)
{
    std::vector<std::vector<collision::Contact>> contacts = m_detector.Detect();
    debug::Debug::CollisionDetectionCall(contacts);
    m_resolver.Resolve(contacts, duration);
}

void Scene::ApplyCollisionCache()
{

}

void Scene::ApplyForces()
{
    //Clear previously applied forces
    for (Asset<mechanics::Body>& asset : m_assetManager.GetBodies())
    {
        asset.data.linearMotion.force = glm::dvec3(0);
    }

    //Reapply forces
    ApplyForce<force::StaticField>();
    ApplyForce<force::SquareDistanceSource>();
    ApplyForce<force::Drag>();
    ApplyForce<force::Spring>();
    ApplyForce<force::Bungee>();
    ApplyForce<force::Buoyancy>();
}

void Scene::Integrate(double duration)
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
