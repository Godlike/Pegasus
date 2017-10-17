/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"

#include <chrono>
#include <thread>

using namespace pegasus;

Demo& Demo::GetInstance()
{
    static Demo demo;
    return demo;
}

bool Demo::IsValid() const
{
    return m_pRenderer.IsValid();
}

void Demo::RunFrame()
{
    static std::chrono::steady_clock::time_point nextFrameTime = std::chrono::steady_clock::now();
    static std::chrono::milliseconds const deltaTime(16);
    nextFrameTime += deltaTime;

    RenderFrame();
    ComputeFrame(static_cast<double>(deltaTime.count()) / 1e3);

    std::this_thread::sleep_until(nextFrameTime);
}

Demo::Object& Demo::MakeLine(integration::DynamicBody body, glm::vec3 start, glm::vec3 end)
{
    m_objects.emplace_back(
        nullptr,
        new render::primitive::LineSegment(
            glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), start, end
        )
    );

    return m_objects.back();
}

Demo::Object& Demo::MakePlane(integration::DynamicBody body, glm::dvec3 normal)
{
    m_objects.emplace_back(
        &MakeRigidBody(body, std::make_unique<geometry::Plane>(body.linearMotion.position, normal)),
        new render::primitive::Plane(
            glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), normal
        )
    );
    m_particleContactGenerators.emplace_back(
        std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
    );

    return m_objects.back();
}

Demo::Object& Demo::MakeSphere(integration::DynamicBody body, double radius)
{
    m_objects.emplace_back(
        &MakeRigidBody(body, std::make_unique<geometry::Sphere>(body.linearMotion.position, radius)),
        new render::primitive::Sphere(
            glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), radius
        )
    );
    m_particleContactGenerators.emplace_back(
        std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
    );

    return m_objects.back();
}

Demo::Object& Demo::MakeBox(integration::DynamicBody body, glm::vec3 i, glm::vec3 j, glm::vec3 k)
{
    m_objects.emplace_back(
        &MakeRigidBody(body, std::make_unique<geometry::Box>(body.linearMotion.position, i, j, k)),
        new render::primitive::Box(
            glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)),
            glm::vec3(0.439, 0.502, 0.565),
            render::primitive::Box::Axes{i, j, k}
        )
    );
    m_particleContactGenerators.emplace_back(
        std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
    );

    return m_objects.back();
}

void Demo::Remove(Object& object)
{
    if (object.body != nullptr)
    {
        integration::DynamicBody& body = object.body->pointMass;
        m_particleForceRegistry.Remove(body);

        //Remove contact generator
        m_particleContactGenerators.erase(
            std::find_if(m_particleContactGenerators.begin(), m_particleContactGenerators.end(),
                [&object](auto& generator) -> bool
                {
                    return (object.body == &dynamic_cast<ShapeContactGenerator<RigidBodies>*>(generator.get())->rigidBody);
                })
        );

        //Remove RB
        for (auto it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it)
        {
            if (&it->pointMass == &body)
            {
                m_rigidBodies.erase(it);
                break;
            }
        }

        //Remove integration::DynamicBody
        for (auto it = m_particles.begin(); it != m_particles.end(); ++it)
        {
            if (&*it == &body)
            {
                m_particles.erase(it);
                break;
            }
        }
    }

    //Remove object
    for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
    {
        if (&*it == &object)
        {
            m_objects.erase(it);
            break;
        }
    }
}

Demo::Object::Object(RigidBody* body, render::primitive::Primitive* shape)
    : body(body)
    , shape(shape)
{
}

Demo::Demo()
    : m_pRenderer(render::Renderer::GetInstance())
    , m_particleWorld(m_particles,
        m_particleForceRegistry,
        m_particleContactGenerators,
        glm::pow2(maxParticles),
        maxParticles)
    , m_gravityForce(glm::dvec3{0, -9.8, 0})
{
}

void Demo::ComputeFrame(double duration)
{
    m_particleWorld.StartFrame();
    m_particleWorld.RunPhysics(duration);

    //Update positions
    for (Object& object : m_objects)
    {
        if (object.body != nullptr)
        {
            object.body->shape->centerOfMass = object.body->pointMass.linearMotion.position;
            glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(object.body->shape->centerOfMass));
            object.shape->SetModel(model);
        }
    }
}

void Demo::RenderFrame() const
{
    m_pRenderer.RenderFrame();
}

RigidBody& Demo::MakeRigidBody(integration::DynamicBody body, std::unique_ptr<geometry::SimpleShape>&& shape)
{
    m_particles.push_back(body);
    m_particleForceRegistry.Add(m_particles.back(), m_gravityForce);
    m_rigidBodies.emplace_back(m_particles.back(), std::move(shape));

    return m_rigidBodies.back();
}
