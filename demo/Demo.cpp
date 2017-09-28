/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <demo/Demo.hpp>

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

Demo::Object& Demo::MakeLine(Particle particle, glm::vec3 start, glm::vec3 end)
{
    m_objects.emplace_back(
        nullptr,
        std::make_unique<render::primitive::LineSegment>(
            glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), start, end)
    );

    return m_objects.back();
}

Demo::Object& Demo::MakePlane(Particle particle, glm::dvec3 normal)
{
    m_objects.emplace_back(
        &MakeRigidBody(particle, std::make_unique<geometry::Plane>(particle.GetPosition(), normal)),
        std::make_unique<render::primitive::Plane>(
            glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), normal)
    );
    m_particleContactGenerators.emplace_back(
        std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
    );

    return m_objects.back();
}

Demo::Object& Demo::MakeSphere(Particle particle, double radius)
{
    m_objects.emplace_back(
        &MakeRigidBody(particle, std::make_unique<geometry::Sphere>(particle.GetPosition(), radius)),
        std::make_unique<render::primitive::Sphere>(
            glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), radius)
    );
    m_particleContactGenerators.emplace_back(
        std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
    );

    return m_objects.back();
}

Demo::Object& Demo::MakeBox(Particle particle, glm::vec3 i, glm::vec3 j, glm::vec3 k)
{
    m_objects.emplace_back(
        &MakeRigidBody(particle, std::make_unique<geometry::Box>(particle.GetPosition(), i, j, k)),
        std::make_unique<render::primitive::Box>(
            glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())),
            glm::vec3(0.439, 0.502, 0.565),
            render::primitive::Box::Axes{
                i, j, k
            }
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
        Particle& particle = object.body->p;
        m_particleForceRegistry.Remove(particle);

        for (auto it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it)
        {
            if (&it->p == &particle)
            {
                m_rigidBodies.erase(it);
                break;
            }
        }
        for (auto it = m_particles.begin(); it != m_particles.end(); ++it)
        {
            if (&*it == &particle)
            {
                m_particles.erase(it);
                break;
            }
        }
    }

    for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
    {
        if (&*it == &object)
        {
            m_objects.erase(it);
            break;
        }
    }

    m_particleContactGenerators.clear();
    for (Object& obj : m_objects)
    {
        if (obj.body != nullptr)
        {
            m_particleContactGenerators.emplace_back(
                std::make_unique<ShapeContactGenerator<RigidBodies>>(*obj.body, m_rigidBodies, 0.7)
            );
        }
    }
}

Demo::Object::Object(RigidBody* body, std::unique_ptr<render::primitive::Primitive>&& renderShape)
    : body(body)
    , shape(std::move(renderShape))
{
}

Demo::Demo()
    : m_pRenderer(render::Renderer::GetInstance())
    , m_particleWorld(m_particles,
        m_particleForceRegistry,
        m_particleContactGenerators,
        glm::pow2(maxParticles),
        maxParticles)
    , m_gravityForce(glm::dvec3{
        0, -9.8, 0
    })
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
            object.body->s->centerOfMass = object.body->p.GetPosition();
            glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(object.body->s->centerOfMass));
            object.shape->SetModel(model);
        }
    }
}

void Demo::RenderFrame() const
{
    m_pRenderer.RenderFrame();
}

RigidBody& Demo::MakeRigidBody(Particle particle, std::unique_ptr<geometry::SimpleShape>&& shape)
{
    m_particles.push_back(particle);
    m_particleForceRegistry.Add(m_particles.back(), m_gravityForce);
    m_rigidBodies.emplace_back(m_particles.back(), std::move(shape));

    return m_rigidBodies.back();
}
