/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"
#include <Arion/Shape.hpp>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <chrono>
#include <thread>

namespace pegasus
{

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
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::LineSegment(model, glm::vec3(0.439, 0.502, 0.565), start, end);
    m_primitives.emplace_back(nullptr, shape);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakePlane(mechanics::Body body, glm::dvec3 normal, scene::Primitive::Type type)
{
    scene::Primitive* object = new scene::Plane(type, body,
        arion::Plane(body.linearMotion.position, body.angularMotion.orientation, normal));
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::Plane(model, glm::vec3(0.439, 0.502, 0.565),  normal);
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeSphere(mechanics::Body body, double radius, scene::Primitive::Type type)
{
    scene::Primitive* object = new scene::Sphere(type, body,
        arion::Sphere(body.linearMotion.position, body.angularMotion.orientation, radius));
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::Sphere(model, glm::vec3(0.439, 0.502, 0.565),  radius);
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeBox(
        mechanics::Body body, glm::vec3 i, glm::vec3 j, glm::vec3 k, scene::Primitive::Type type
    )
{
    scene::Primitive* object = new scene::Box(type, body,
        arion::Box(body.linearMotion.position, body.angularMotion.orientation, i, j, k));
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::Box(model, glm::vec3(0.439, 0.502, 0.565), render::Box::Axes{i, j, k});
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
            mechanics::Body const physicalBody = primitive.physicalPrimitive->GetBody();
            glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(physicalBody.linearMotion.position))
                * glm::mat4(glm::toMat4(physicalBody.angularMotion.orientation));
            primitive.renderPrimitive->SetModel(model);
        }
    }
}

void Demo::RenderFrame() const
{
    m_renderer.RenderFrame();
}
} // namespace pegasus
