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
    return m_renderer.IsValid();
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

Demo::Object& Demo::MakeLine(mechanics::Body body, glm::vec3 start, glm::vec3 end)
{
    render::primitive::Primitive* shape = new render::primitive::LineSegment(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), start, end
    );

    m_staticObjects.emplace_back(shape);
    m_objects.emplace_back(&m_staticObjects.back());

    return *m_objects.back();
}

Demo::Object& Demo::MakePlane(mechanics::Body body, glm::dvec3 normal, BodyType type)
{
    MakeRigidBody(body, std::make_unique<geometry::Plane>(body.linearMotion.position, normal), type);
    render::primitive::Primitive* shape = new render::primitive::Plane(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), normal
    );

    if (type == BodyType::STATIC) {
        m_staticObjects.emplace_back(shape, &m_scene.staticObjects, m_scene.staticObjects.size() - 1);
        m_objects.emplace_back(&m_staticObjects.back());
    } else if (type == BodyType::DYNAMIC) {
        m_dynamicObjects.emplace_back(shape, &m_scene.dynamicObjects, m_scene.dynamicObjects.size() - 1);
        m_objects.emplace_back(&m_dynamicObjects.back());
    }
    
    return *m_objects.back();
}

Demo::Object& Demo::MakeSphere(mechanics::Body body, double radius, BodyType type)
{
    MakeRigidBody(body, std::make_unique<geometry::Sphere>(body.linearMotion.position, radius), type);
    render::primitive::Primitive* shape = new render::primitive::Sphere(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), radius
    );

    if (type == BodyType::STATIC) {
        m_staticObjects.emplace_back(shape, &m_scene.staticObjects, m_scene.staticObjects.size() - 1);
        m_objects.emplace_back(&m_staticObjects.back());
    }
    else if (type == BodyType::DYNAMIC) {
        m_dynamicObjects.emplace_back(shape, &m_scene.dynamicObjects, m_scene.dynamicObjects.size() - 1);
        m_objects.emplace_back(&m_dynamicObjects.back());
    }

    return *m_objects.back();
}

Demo::Object& Demo::MakeBox(mechanics::Body body, glm::vec3 i, glm::vec3 j, glm::vec3 k, BodyType type)
{
    MakeRigidBody(body, std::make_unique<geometry::Box>(body.linearMotion.position, i, j, k), type);
    render::primitive::Primitive* shape = new render::primitive::Box(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)),
        glm::vec3(0.439, 0.502, 0.565),
        render::primitive::Box::Axes{i, j, k}
    );

    if (type == BodyType::STATIC) {
        m_staticObjects.emplace_back(shape, &m_scene.staticObjects, m_scene.staticObjects.size() - 1);
        m_objects.emplace_back(&m_staticObjects.back());
    }
    else if (type == BodyType::DYNAMIC) {
        m_dynamicObjects.emplace_back(shape, &m_scene.dynamicObjects, m_scene.dynamicObjects.size() - 1);
        m_objects.emplace_back(&m_dynamicObjects.back());
    }

    return *m_objects.back();
}

void Demo::Remove(Object& object)
{
    if (object.GetObject() != nullptr)
    {
        mechanics::Body* body = object.GetObject()->body;
        m_scene.forceRegistry.Remove(*body);

        //Remove body
        for (auto it = m_scene.bodies.begin(); it != m_scene.bodies.end(); ++it)
        {
            if (&*it == body)
            {
                m_scene.bodies.erase(it);
                break;
            }
        }

        //Remove static object
        for (auto it = m_scene.staticObjects.begin(); it != m_scene.staticObjects.end(); ++it)
        {
            if (it->body == body)
            {
                m_scene.staticObjects.erase(it);
                break;
            }
        }

        //Remove dynamic object
        for (auto it = m_scene.dynamicObjects.begin(); it != m_scene.dynamicObjects.end(); ++it)
        {
            if (it->body == body)
            {
                m_scene.dynamicObjects.erase(it);
                break;
            }
        }
    }

    //Remove static object
    for (auto it = m_staticObjects.begin(); it != m_staticObjects.end(); ++it)
    {
        if (&*it == &object)
        {
            m_staticObjects.erase(it);
            break;
        }
    }

    //Remove dynamic object
    for (auto it = m_dynamicObjects.begin(); it != m_dynamicObjects.end(); ++it)
    {
        if (&*it == &object)
        {
            m_dynamicObjects.erase(it);
            break;
        }
    }

    //Remove object
    for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
    {
        if (*it == &object)
        {
            m_objects.erase(it);
            break;
        }
    }
}

Demo::Object::Object(render::primitive::Primitive* shape)
    : shape(shape)
{
}

Demo::Demo()
    : m_renderer(render::Renderer::GetInstance())
    , m_gravityForce(glm::dvec3{0, -9.8, 0})
{
}

void Demo::ComputeFrame(double duration)
{
    m_scene.ComputeFrame(duration);

    //Update positions
    for (Object* object : m_objects)
    {
        if (object->GetObject() != nullptr)
        {
            object->GetObject()->shape->centerOfMass = object->GetObject()->body->linearMotion.position;
            glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(object->GetObject()->shape->centerOfMass));
            object->shape->SetModel(model);
        }
    }
}

void Demo::RenderFrame() const
{
    m_renderer.RenderFrame();
}

void Demo::MakeRigidBody(
        mechanics::Body body, std::unique_ptr<geometry::SimpleShape>&& shape, BodyType type
    )
{
    m_scene.bodies.push_back(body);
    m_scene.forceRegistry.Add(m_scene.bodies.back(), m_gravityForce);

    if (type == BodyType::STATIC)
    {
        m_scene.staticObjects.emplace_back(m_scene.bodies.back(), std::move(shape));
    }
    else if (type == BodyType::DYNAMIC)
    {
        m_scene.dynamicObjects.emplace_back(m_scene.bodies.back(), std::move(shape));
    }
}
