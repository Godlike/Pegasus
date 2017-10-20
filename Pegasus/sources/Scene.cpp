/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Scene.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Integration.hpp>

using namespace pegasus;
using namespace scene;

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
    am.GetAsset(am.GetBodies(), body).material.SetInverseMass(0);
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

void Scene::ComputeFrame(double duration)
{
    ResolveCollisions(duration);

    ApplyForces();

    Integrate(duration);
}

Handle Scene::MakeBody() const
{
    return m_assets.MakeAsset(m_assets.GetBodies());
}

Scene::Scene()
    : m_assets(AssetManager::GetInstance())
{
}

mechanics::Body& Scene::GetBody(Handle handle) const
{
    return m_assets.GetAsset(m_assets.GetBodies(), handle);
}

void Scene::RemoveBody(Handle handle) const
{
    m_assets.RemoveAsset(m_assets.GetBodies(), handle);
}

void Scene::ResolveCollisions(double duration)
{
    static collision::Detector detector;
    std::vector<std::vector<collision::Contact>> contacts = detector.Detect();

    static collision::Resolver resolver;
    resolver.Resolve(contacts, duration);
}

void Scene::ApplyForces()
{
    //Clear previously applied forces
    for (Asset<mechanics::Body>& asset : m_assets.GetBodies())
    {
        asset.data.linearMotion.force = glm::dvec3(0);
    }

    //Reapply forces
    ApplyForce<force::StaticField>();
    ApplyForce<force::Drag>();
    ApplyForce<force::Spring>();
    ApplyForce<force::AnchoredSpring>();
    ApplyForce<force::Bungee>();
    ApplyForce<force::Buoyancy>();
}

void Scene::Integrate(double duration)
{
    for (Asset<mechanics::Body>& asset : m_assets.GetBodies())
    {
        if (asset.id != 0)
        {
            integration::Integrate(asset.data, duration);
        }
    }

    UpdateShapes<DynamicBody, geometry::Plane>();
    UpdateShapes<DynamicBody, geometry::Sphere>();
    UpdateShapes<DynamicBody, geometry::Box>();
}

Primitive::~Primitive()
{
    m_scene->RemoveBody(m_body);
}

mechanics::Body& Primitive::GetBody() const
{
    return m_scene->GetBody(m_body);
}

Primitive::Primitive(Type type, mechanics::Body body)
    : m_scene(&Scene::GetInstance())
    , m_type(type)
{
    m_body = m_scene->MakeBody();
    m_scene->GetBody(m_body) = body;
}

Plane::Plane(Type type, mechanics::Body body, geometry::Plane plane)
    : Primitive(type, body)
{
    m_shape = m_scene->MakeShape<geometry::Plane>();
    m_scene->GetShape<geometry::Plane>(m_shape) = plane;
    MakeObject<geometry::Plane>();
}

Plane::~Plane()
{
    m_scene->RemoveShape<geometry::Plane>(m_shape);
    RemoveObject<geometry::Plane>();
}

geometry::Plane& Plane::GetShape() const
{
    return m_scene->GetShape<geometry::Plane>(m_shape);
}

Sphere::Sphere(Type type, mechanics::Body body, geometry::Sphere sphere)
    : Primitive(type, body)
{
    m_shape = m_scene->MakeShape<geometry::Sphere>();
    m_scene->GetShape<geometry::Sphere>(m_shape) = sphere;
    MakeObject<geometry::Sphere>();
}

Sphere::~Sphere()
{
    m_scene->RemoveShape<geometry::Sphere>(m_shape);
    RemoveObject<geometry::Sphere>();
}

geometry::Sphere& Sphere::GetShape() const
{
    return m_scene->GetShape<geometry::Sphere>(m_shape);
}

Box::Box(Type type, mechanics::Body body, geometry::Box box)
    : Primitive(type, body)
{
    m_shape = m_scene->MakeShape<geometry::Box>();
    m_scene->GetShape<geometry::Box>(m_shape) = box;
    MakeObject<geometry::Box>();
}

Box::~Box()
{
    m_scene->RemoveShape<geometry::Box>(m_shape);
    RemoveObject<geometry::Box>();
}

geometry::Box& Box::GetShape() const
{
    return m_scene->GetShape<geometry::Box>(m_shape);
}
