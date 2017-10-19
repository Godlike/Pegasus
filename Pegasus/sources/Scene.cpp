/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Scene.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Integration.hpp>

using namespace pegasus;

void Scene::ComputeFrame(double duration)
{
    ResolveCollisions(duration);

    ApplyForces();

    Integrate(duration);
}

Handle Scene::MakeBody()
{
    return MakeAsset(m_bodies);
}

mechanics::Body& Scene::GetBody(Handle handle)
{
    return GetAsset(m_bodies, handle);
}

void Scene::RemoveBody(Handle handle)
{
    RemoveAsset(m_bodies, handle);
}

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
inline Handle Scene::MakeObject<mechanics::StaticObject>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_staticObjects);
    GetAsset(m_staticObjects, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::StaticObject>(Handle handle)
{
    RemoveAsset(m_staticObjects, handle);
}

template <>
inline Handle Scene::MakeObject<mechanics::DynamicObject>(Handle body, Handle shape)
{
    Handle const id = MakeAsset(m_staticObjects);
    GetAsset(m_staticObjects, id) = { body, shape };
    return id;
}

template <>
inline void Scene::RemoveObject<mechanics::DynamicObject>(Handle handle)
{
    RemoveAsset(m_dynamicObjects, handle);
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

inline void Scene::ResolveCollisions(double duration)
{
    static collision::Detector detector;
    std::vector<collision::Contact> dContacts = detector.Detect(m_dynamicObjects);
    std::vector<collision::Contact> dsContacts = detector.Detect(m_dynamicObjects, m_staticObjects);

    static collision::Resolver resolver;
    resolver.Resolve(dContacts, duration);
    resolver.Resolve(dsContacts, duration);
}

inline void Scene::ApplyForces()
{
    //Clear previously applied forces
    for (Asset<mechanics::Body>& asset : m_bodies)
    {
        asset.data.linearMotion.force = glm::dvec3(0);
    }

    //Reapply forces
    ApplyForce(m_staticFieldForceBindings);
    ApplyForce(m_dragForceBindings);
    ApplyForce(m_springForceBindings);
    ApplyForce(m_anchoredSpringForceBindings);
    ApplyForce(m_bungeeForceBindings);
    ApplyForce(m_buoyancyForceBindings);
}

inline void Scene::Integrate(double duration)
{
    for (Asset<mechanics::Body>& asset : m_bodies)
    {
        if (asset.id != 0)
        {
            integration::Integrate(asset.data, duration);
        }
    }
}
