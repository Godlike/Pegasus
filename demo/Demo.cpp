/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"
#include <pegasus/Debug.hpp>
#include <Arion/Shape.hpp>
#include <Arion/Debug.hpp>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>

#include <chrono>
#include <thread>

namespace 
{
static bool g_gjkSimplexCheckbox = false;
static bool g_epaPolytopeCheckbox = false;
static bool g_collisionPointsCheckbox = false;
std::list<pegasus::Demo::Primitive*> g_objects;

void CollisionDetectionDebugCallback(
        std::vector<std::vector<pegasus::collision::Contact>>& contacts
    )
{
    static pegasus::Demo& demo = pegasus::Demo::GetInstance();
    static std::vector<pegasus::Demo::Primitive*> contactPoints;

    if (g_collisionPointsCheckbox)
    {
        for (auto p : contactPoints)
        {
            demo.Remove(*p);
            p = nullptr;
        }
        contactPoints.clear();

        for (auto& c : contacts)
        {
            for (auto& contact : c)
            {
                contactPoints.push_back(&demo.MakeSphere(contact.manifold.aContactPoint, 0.05, { 1, 0, 0 }));
                contactPoints.push_back(&demo.MakeSphere(contact.manifold.bContactPoint, 0.05, { 0, 1, 0 }));
            }
        }
    }
}

void EpaDebugCallback(
        epona::QuickhullConvexHull<std::vector<glm::dvec3>>& convexHull,
        std::vector<glm::dvec3>& polytopeVertices,
        arion::intersection::gjk::Simplex& simplex
    )
{
    static pegasus::Demo& demo = pegasus::Demo::GetInstance();
    static pegasus::Demo::Primitive* gjkSimplex = nullptr;
    static pegasus::Demo::Primitive* epaPolytope = nullptr;

    if (gjkSimplex) 
    {
        demo.Remove(*gjkSimplex);
        gjkSimplex = nullptr;
    }
    if (epaPolytope)
    {
        demo.Remove(*epaPolytope);
        epaPolytope = nullptr;
    }

    if (g_epaPolytopeCheckbox) 
    {
        std::vector<glm::mat3> triangles;
        for (auto& face : convexHull.GetFaces())
        {
            auto indices = face.GetIndices();
            triangles.emplace_back(polytopeVertices[indices[0]], polytopeVertices[indices[1]], polytopeVertices[indices[2]]);
        }
        epaPolytope = &demo.MakeTriangleCollection({}, { 0, 1, 0 }, triangles);
    }

    if (g_gjkSimplexCheckbox) 
    {
        gjkSimplex = &demo.MakeTriangleCollection({}, { 1, 0, 0 }, {
            glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[2] },
            glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[3] },
            glm::mat3{ simplex.vertices[1], simplex.vertices[2], simplex.vertices[3] },
            glm::mat3{ simplex.vertices[0], simplex.vertices[2], simplex.vertices[3] },
        });
    }
}

void DrawUi()
{
    static bool physicsDebugWindowVisible = false;

    ImGui::BeginMainMenuBar();
    {
        static bool item = false;
        ImGui::MenuItem("Scene", "ctrl+s", &item);
    }
    ImGui::EndMainMenuBar();


    ImGui::Begin("Physics debug", &physicsDebugWindowVisible);
    {
        auto& demo = pegasus::Demo::GetInstance();

        {
            ImGui::Text("CSO debug");
            ImGui::Spacing();
            ImGui::Checkbox("Draw GJK simplex", &g_gjkSimplexCheckbox);
            ImGui::Checkbox("Draw EPA polytope", &g_epaPolytopeCheckbox);
            ImGui::Spacing();
        }

        const char* primitiveBodyTypeComboItems[] = { "Static", "Dynamic" };
        static int currentPrimitiveBodyType = 1;
        ImGui::Combo("Body type", &currentPrimitiveBodyType, primitiveBodyTypeComboItems, IM_ARRAYSIZE(primitiveBodyTypeComboItems));

        static float position[3] = { 0, 0, 0 };
        ImGui::InputFloat3("Position (X Y Z)", position, 3);

        static float angleAxis[4] = {};
        ImGui::InputFloat4("Angle, Axis (X Y Z)", angleAxis, 3);

        static float sphereRadius = 1;
        static float boxSides[3] = { 0.5f, 0.5f, 0.5f };
        if (currentPrimitiveType == 0)
        {
            ImGui::Separator();
            ImGui::Text("Simulation configs");
            ImGui::Spacing();
            ImGui::Checkbox("Use static frame duration", &demo.useStaticDuration);
            float duration = static_cast<float>(demo.staticDuration);
            ImGui::SliderFloat("Duration", &duration, 0.001f, 0.016f);
            demo.staticDuration = duration;

            if (ImGui::Button(demo.calculatePhysics ? "Pause" : "Run  "))
            {
                demo.calculatePhysics = !demo.calculatePhysics;
            }
            ImGui::Text("Frame: %.3f ms; (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Spacing();
        }
        
        {
            ImGui::Separator();
            ImGui::Text("Scene configs");
            ImGui::Spacing();

            const char* primitiveRenderComboItems[] = { "Wire", "Solid", "Wire&Solid" };
            static int currentPrimitiveRenderType = 2;
            ImGui::Combo("Render type", &currentPrimitiveRenderType, primitiveRenderComboItems, IM_ARRAYSIZE(primitiveRenderComboItems));
            auto& render = pegasus::render::Renderer::GetInstance();
            render.primitiveRenderType = static_cast<pegasus::render::Renderer::PrimitiveRenderType>(currentPrimitiveRenderType);

            const char* primitiveTypeComboItems[] = { "Sphere", "Box"};
            static int currentPrimitiveType = 1;
            ImGui::Combo("Primitive type", &currentPrimitiveType, primitiveTypeComboItems, IM_ARRAYSIZE(primitiveTypeComboItems));

            const char* primitiveBodyTypeComboItems[] = { "Static", "Dynamic" };
            static int currentPrimitiveBodyType = 1;
            ImGui::Combo("Body type", &currentPrimitiveBodyType, primitiveBodyTypeComboItems, IM_ARRAYSIZE(primitiveBodyTypeComboItems));

            static float position[3] = {};
            ImGui::InputFloat3("Position (X Y Z)", position);

            static float sphereRadius = 1;
            static float boxSides[3] = { 1, 1, 1 };
            if (currentPrimitiveType == 0)
            {
                ImGui::InputFloat("Radius", &sphereRadius);
                if (ImGui::Button("Random")) 
                {
                    sphereRadius = rand() % 10 / 10.f;
                }
            }
            else 
            {
                ImGui::InputFloat3("Axes (X Y Z)", boxSides);
                if (ImGui::Button("Random"))
                {
                    boxSides[0] = (rand() % 10 / 10.f + 0.1f); 
                    boxSides[1] = (rand() % 10 / 10.f + 0.1f); 
                    boxSides[2] = (rand() % 10 / 10.f + 0.1f);
                }
            }

            if (ImGui::Button("Make"))
            {
                if (g_objects.size() < demo.maxObjects)
                {
                    pegasus::mechanics::Body body;
                    body.linearMotion.position = glm::make_vec3(position);

                    auto bodyType = (currentPrimitiveBodyType == 0)
                        ? pegasus::scene::Primitive::Type::STATIC 
                        : pegasus::scene::Primitive::Type::DYNAMIC;

                    if (currentPrimitiveType == 0)
                    {
                        g_objects.push_back(&demo.MakeSphere(body, sphereRadius, bodyType));
                    } 
                    else
                    {
                        g_objects.push_back(&demo.MakeBox(
                            body, {boxSides[0], 0, 0}, {0, boxSides[1], 0}, {0, 0, boxSides[2]}, bodyType
                        ));
                    }
                }
            }
            if (ImGui::Button("Clear"))
            {
                while (!g_objects.empty())
                {
                    demo.Remove(*g_objects.back());
                    g_objects.pop_back();
                }
            }
        }
    }
    ImGui::End();
}
} // namespace ::

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
    if (calculatePhysics) 
    {
        ComputeFrame(static_cast<double>(deltaTime.count()) / 1e3);
    }

    std::this_thread::sleep_until(nextFrameTime);
}

Demo::Primitive& Demo::MakeLine(mechanics::Body body, glm::vec3 color, glm::vec3 start, glm::vec3 end)
{
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::LineSegment(model, color, start, end);
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

Demo::Primitive& Demo::MakeTriangle(mechanics::Body body, glm::vec3 color, glm::vec3 a, glm::vec3 b, glm::vec3 c)
{
    glm::mat4 const model{ glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::Triangle(model, color, a, b, c);
    m_primitives.emplace_back(nullptr, shape);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeSphere(mechanics::Body body, double radius, scene::Primitive::Type type)
{
    scene::Primitive* object = new scene::Sphere(type, body,
        arion::Sphere(body.linearMotion.position, body.angularMotion.orientation, radius));
    glm::mat4 const model { glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::Sphere(model, glm::vec3(0.439, 0.502, 0.565), radius);
    m_primitives.emplace_back(object, shape);
    m_pGravityForce->Bind(*object);

    return m_primitives.back();
}

Demo::Primitive& Demo::MakeSphere(glm::dvec3 center, double radius, glm::vec3 color)
{
    glm::mat4 const model{ glm::translate(glm::mat4(1), glm::vec3(center))
        * glm::mat4(glm::toMat4(glm::dquat(glm::angleAxis(0.0, glm::dvec3{ 0, 0, 0 })))) };
    render::Primitive* shape = new render::Sphere(model, color, radius);
    m_primitives.emplace_back(nullptr, shape);

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

Demo::Primitive& Demo::MakeTriangleCollection(mechanics::Body body, glm::vec3 color, std::vector<glm::mat3> triangles)
{
    glm::mat4 const model{ glm::translate(glm::mat4(1), glm::vec3(body.linearMotion.position))
        * glm::mat4(glm::toMat4(body.angularMotion.orientation)) };
    render::Primitive* shape = new render::TriangleCollection(model, color, triangles);
    m_primitives.emplace_back(nullptr, shape);

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
    auto& arionDebug = arion::debug::Debug::GetInstace();
    arionDebug.epaCallback = ::EpaDebugCallback;

    auto& pegasusDebug = pegasus::debug::Debug::GetInstace();
    pegasusDebug.collisionDetectionCall = ::CollisionDetectionDebugCallback;

    m_renderer.drawUiCallback = ::DrawUi;

    m_scene.Initialize(scene::AssetManager::GetInstance());
    m_pGravityForce = std::make_unique<scene::Force<force::StaticField>>(force::StaticField(glm::dvec3{ 0, -9.8, 0 }));
}

void Demo::ComputeFrame(double duration)
{
    //Compute physical data
    m_scene.ComputeFrame(useStaticDuration ? staticDuration : duration);
    
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
