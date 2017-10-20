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

inline void Scene::ResolveCollisions(double duration)
{
    static collision::Detector detector;
    std::vector<std::vector<collision::Contact>> contacts({
    	detector.Detect(m_dynamicPlanes),
    	detector.Detect(m_dynamicPlanes, m_dynamicSpheres),
    	detector.Detect(m_dynamicPlanes, m_dynamicBoxes),
    	detector.Detect(m_dynamicPlanes, m_staticPlanes),
    	detector.Detect(m_dynamicPlanes, m_staticSpheres),
    	detector.Detect(m_dynamicPlanes, m_staticBoxes),
    	detector.Detect(m_dynamicSpheres),
    	detector.Detect(m_dynamicSpheres, m_dynamicPlanes),
    	detector.Detect(m_dynamicSpheres, m_dynamicBoxes),
    	detector.Detect(m_dynamicSpheres, m_staticPlanes),
    	detector.Detect(m_dynamicSpheres, m_staticSpheres),
    	detector.Detect(m_dynamicSpheres, m_staticBoxes),
    	detector.Detect(m_dynamicBoxes),
    	detector.Detect(m_dynamicBoxes, m_dynamicPlanes),
    	detector.Detect(m_dynamicBoxes, m_dynamicSpheres),
    	detector.Detect(m_dynamicBoxes, m_staticPlanes),
		detector.Detect(m_dynamicBoxes, m_staticSpheres),
		detector.Detect(m_dynamicBoxes, m_staticBoxes),
    });

    static collision::Resolver resolver;
    for (auto& c : contacts)
    {
    	resolver.Resolve(c, duration);
    }
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

    for (Asset<RigidBody>& asset : m_dynamicPlanes)
    {
        if (asset.id != 0)
        {
            mechanics::Body& body = GetBody(asset.data.body);
            geometry::Plane& plane = GetShape<geometry::Plane>(asset.data.shape);
            plane.centerOfMass = body.linearMotion.position;
        }
    }

    for (Asset<RigidBody>& asset : m_dynamicSpheres)
    {
        if (asset.id != 0)
        {
            mechanics::Body& body = GetBody(asset.data.body);
            geometry::Sphere& sphere = GetShape<geometry::Sphere>(asset.data.shape);
            sphere.centerOfMass = body.linearMotion.position;
        }
    }

    for (Asset<RigidBody>& asset : m_dynamicBoxes)
    {
        if (asset.id != 0)
        {
            mechanics::Body& body = GetBody(asset.data.body);
            geometry::Box& box = GetShape<geometry::Box>(asset.data.shape);
            box.centerOfMass = body.linearMotion.position;
        }
    }
}
