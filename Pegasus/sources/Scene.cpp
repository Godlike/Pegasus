/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Scene.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Integration.hpp>

namespace pegasus
{
namespace scene
{

RigidBody::RigidBody(Handle body, Handle shape)
    : body(body)
    , shape(shape)
{
}

AssetManager& AssetManager::GetInstance()
{
    static AssetManager am;
    return am;
}

std::vector<Asset<mechanics::Body>>& AssetManager::GetBodies()
{
    return m_bodies;
}

StaticBody::StaticBody(Handle body, Handle shape)
    : RigidBody(body, shape)
{
    static AssetManager& am = AssetManager::GetInstance();
    am.GetAsset(am.GetBodies(), body).material.SetInfiniteMass();
}

DynamicBody::DynamicBody(Handle body, Handle shape)
    : RigidBody(body, shape)
{
}

Scene& Scene::GetInstance()
{
    static Scene scene;
    return scene;
}

void Scene::Initialize(AssetManager& assetManager)
{
    m_assetManager = &assetManager;
}

void Scene::ComputeFrame(double duration)
{
    ResolveCollisions(duration);

    ApplyForces();

    Integrate(duration);
}

Handle Scene::MakeBody() const
{
    return m_assetManager->MakeAsset(m_assetManager->GetBodies());
}

mechanics::Body& Scene::GetBody(Handle handle) const
{
    return m_assetManager->GetAsset(m_assetManager->GetBodies(), handle);
}

void Scene::RemoveBody(Handle handle) const
{
    m_assetManager->RemoveAsset(m_assetManager->GetBodies(), handle);
}

void Scene::ResolveCollisions(double duration) const
{
    static collision::Detector detector(*m_assetManager);
    std::vector<std::vector<collision::Contact>> contacts = detector.Detect();

    static collision::Resolver resolver;
    resolver.Resolve(contacts, duration);
}

void Scene::ApplyForces()
{
    //Clear previously applied forces
    for (Asset<mechanics::Body>& asset : m_assetManager->GetBodies())
    {
        asset.data.linearMotion.force = glm::dvec3(0);
    }

    //Reapply forces
    ApplyForce<force::StaticField>();
    ApplyForce<force::Drag>();
    ApplyForce<force::Spring>();
    ApplyForce<force::Bungee>();
    ApplyForce<force::Buoyancy>();
}

void Scene::Integrate(double duration)
{
    for (Asset<mechanics::Body>& asset : m_assetManager->GetBodies())
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

Primitive::Primitive(Type type, mechanics::Body body)
    : m_type(type)
{
    m_bodyHandle = m_pScene->MakeBody();
    m_pScene->GetBody(m_bodyHandle) = body;
}

Plane::Plane(Type type, mechanics::Body body, arion::Plane plane)
    : Primitive(type, body)
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

Sphere::Sphere(Type type, mechanics::Body body, arion::Sphere sphere)
    : Primitive(type, body)
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

Box::Box(Type type, mechanics::Body body, arion::Box box)
    : Primitive(type, body)
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
} // namespace scene
} // namespace pegasus
