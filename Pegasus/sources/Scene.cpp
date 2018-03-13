/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Debug.hpp>
#include <pegasus/Scene.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Integration.hpp>

namespace pegasus
{
namespace scene
{

RigidBody::RigidBody(Scene& scene, Handle body, Handle shape)
    : body(body)
    , shape(shape)
    , pScene(&scene)
{
}

std::vector<Asset<mechanics::Body>>& AssetManager::GetBodies()
{
    return m_asset.m_bodies;
}

StaticBody::StaticBody(Scene& scene, Handle body, Handle shape)
    : RigidBody(scene, body, shape)
{
    scene.GetBody(body).material.SetInfiniteMass();
}

DynamicBody::DynamicBody(Scene& scene, Handle body, Handle shape)
    : RigidBody(scene, body, shape)
{
}

void Scene::ComputeFrame(double duration)
{
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
    static collision::Detector detector(m_assetManager);
    std::vector<std::vector<collision::Contact>> contacts = detector.Detect();
    debug::Debug::CollisionDetectionCall(contacts);

    static collision::Resolver resolver;
    resolver.Resolve(contacts, duration);
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
        if (asset.id != 0)
        {
            integration::Integrate(asset.data, duration);
        }
    }

    UpdateShapes<DynamicBody, arion::Plane>();
    UpdateShapes<DynamicBody, arion::Sphere>();
    UpdateShapes<DynamicBody, arion::Box>();
}

Primitive::~Primitive()
{
    m_pScene->RemoveBody(m_bodyHandle);
}

void Primitive::SetBody(mechanics::Body body) const
{
    m_pScene->GetBody(m_bodyHandle) = body;
}

mechanics::Body Primitive::GetBody() const
{
    return m_pScene->GetBody(m_bodyHandle);
}

Handle Primitive::GetBodyHandle() const
{
    return m_bodyHandle;
}

Handle Primitive::GetShapeHandle() const
{
    return m_shapeHandle;
}

Handle Primitive::GetObjectHandle() const
{
    return m_objectHandle;
}

Primitive::Primitive(Scene& scene, Type type, mechanics::Body body)
    : SceneObject(scene)
    , m_type(type)
{
    m_bodyHandle = m_pScene->MakeBody();
    m_pScene->GetBody(m_bodyHandle) = body;
}

Plane::Plane(Scene& scene, Type type, mechanics::Body body, arion::Plane plane)
    : Primitive(scene, type, body)
{
    m_shapeHandle = m_pScene->MakeShape<arion::Plane>();
    m_pScene->GetShape<arion::Plane>(m_shapeHandle) = plane;
    MakeObject<arion::Plane>();
}

Plane::~Plane()
{
    m_pScene->RemoveShape<arion::Plane>(m_shapeHandle);
    RemoveObject<arion::Plane>();
}

arion::Plane& Plane::GetShape() const
{
    return m_pScene->GetShape<arion::Plane>(m_shapeHandle);
}

Sphere::Sphere(Scene& scene, Type type, mechanics::Body body, arion::Sphere sphere)
    : Primitive(scene, type, body)
{
    m_shapeHandle = m_pScene->MakeShape<arion::Sphere>();
    m_pScene->GetShape<arion::Sphere>(m_shapeHandle) = sphere;
    MakeObject<arion::Sphere>();
}

Sphere::~Sphere()
{
    m_pScene->RemoveShape<arion::Sphere>(m_shapeHandle);
    RemoveObject<arion::Sphere>();
}

arion::Sphere& Sphere::GetShape() const
{
    return m_pScene->GetShape<arion::Sphere>(m_shapeHandle);
}

Box::Box(Scene& scene, Type type, mechanics::Body body, arion::Box box)
    : Primitive(scene, type, body)
{
    m_shapeHandle = m_pScene->MakeShape<arion::Box>();
    m_pScene->GetShape<arion::Box>(m_shapeHandle) = box;
    MakeObject<arion::Box>();
}

Box::~Box()
{
    m_pScene->RemoveShape<arion::Box>(m_shapeHandle);
    RemoveObject<arion::Box>();
}

arion::Box& Box::GetShape() const
{
    return m_pScene->GetShape<arion::Box>(m_shapeHandle);
}

void PrimitiveGroup::BindForce(ForceBase& force)
{
    if (std::find_if(m_forces.begin(), m_forces.end(),
                        [&force](ForceBase* f) { return &force == f; }) != m_forces.end())
    {
        return;
    }

    m_forces.push_back(&force);

    for (Primitive* primitive : m_primitives)
    {
        force.Bind(*primitive);
    }
}

void PrimitiveGroup::UnbindForce(ForceBase& force)
{
    if (std::find_if(m_forces.begin(), m_forces.end(),
                        [&force](ForceBase* f) { return &force == f; }) == m_forces.end())
    {
        return;
    }

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

void PrimitiveGroup::BindPrimitive(Primitive& primitive)
{
    if (std::find_if(m_primitives.begin(), m_primitives.end(),
                        [&primitive](Primitive* p) { return &primitive == p; }) != m_primitives.end())
    {
        return;
    }

    m_primitives.push_back(&primitive);

    for (ForceBase* force : m_forces)
    {
        force->Bind(primitive);
    }
}

void PrimitiveGroup::UnbindPrimitive(Primitive& primitive)
{
    if (std::find_if(m_primitives.begin(), m_primitives.end(),
                        [&primitive](Primitive* p) { return &primitive == p; }) == m_primitives.end())
    {
        return;
    }

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

} // namespace scene
} // namespace pegasus
