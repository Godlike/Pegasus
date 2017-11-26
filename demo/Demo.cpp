/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"
#include <geometry/Shape.hpp>

#include <glm/glm.hpp>
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

Demo::Primitive& Demo::MakeLine(mechanics::Body body, glm::vec3 start, glm::vec3 end)
{
    render::Primitive* shape = new render::LineSegment(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)),
        glm::vec3(0.439, 0.502, 0.565),
        start, end
    );
    m_primitives.emplace_back(nullptr, shape);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakePlane(mechanics::Body body, glm::dvec3 normal, scene::Primitive::Type type)
{
    scene::Primitive* object = new scene::Plane(type, body, geometry::Plane(body.linearMotion.position, normal));
    render::Primitive* shape = new render::Plane(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), normal
    );
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeSphere(mechanics::Body body, double radius, scene::Primitive::Type type)
{
    scene::Primitive* object = new scene::Sphere(type, body, geometry::Sphere(body.linearMotion.position, radius));
    render::Primitive* shape = new render::Sphere(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)), glm::vec3(0.439, 0.502, 0.565), radius
    );
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeBox(
        mechanics::Body body, glm::vec3 i, glm::vec3 j, glm::vec3 k, scene::Primitive::Type type
    )
{
    scene::Primitive* object = new scene::Box(type, body, geometry::Box(body.linearMotion.position, i, j, k));
    render::Primitive* shape = new render::Box(
        glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position)),
        glm::vec3(0.439, 0.502, 0.565),
        render::Box::Axes{i, j, k}
    );
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

void Demo::Remove(Primitive& primitive)
{
    if (primitive.physicalPrimitive)
    {
        m_pGravityForce->Unbind(*primitive.physicalPrimitive);
    }

    m_primitives.remove_if([&primitive](Primitive& p) { return &primitive == &p; });
}

Demo::Primitive::Primitive(scene::Primitive* body, render::Primitive* shape)
    : physicalPrimitive(body)
    , renderPrimitive(shape)
{
}

Demo::Demo()
    : m_scene(scene::Scene::GetInstance())
    , m_renderer(render::Renderer::GetInstance())
{
    m_scene.Initialize(scene::AssetManager::GetInstance());
    m_pGravityForce = std::make_unique<scene::Force<force::StaticField>>(force::StaticField(glm::dvec3{ 0, -9.8, 0 }));
}

void Demo::ComputeFrame(double duration)
{
    //Compute physical data
    m_scene.ComputeFrame(duration);

    //Update render data
    for (Primitive& primitive : m_primitives)
    {
        if (primitive.physicalPrimitive != nullptr)
        {
            glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(primitive.physicalPrimitive->GetBody().linearMotion.position));
            primitive.renderPrimitive->SetModel(model);
        }
    }
}

void Demo::RenderFrame() const
{
    m_renderer.RenderFrame();
}
