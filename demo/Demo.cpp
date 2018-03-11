/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"
#include <pegasus/Debug.hpp>
#include <Epona/Analysis.hpp>
#include <Arion/Shape.hpp>
#include <Arion/Debug.hpp>

#include <imgui.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <chrono>
#include <thread>

namespace
{
static bool g_gjkSimplexCheckbox = false;
static bool g_epaPolytopeCheckbox = false;
static bool g_gjkHold = false;
static bool g_epaHold = false;
static bool g_epaCsoCheckbox = false;
static bool g_collisionPointsCheckbox = false;
static bool g_contactNormalsCheckbox = false;
static bool g_originAxesCheckbox = true;
static bool g_pauseOnCollisionCheckbox = false;
std::list<pegasus::Demo::Primitive*> g_objects;

void CollisionDetectionDebugCallback(
        std::vector<std::vector<pegasus::collision::Contact>>& contacts
    )
{
    static pegasus::Demo& demo = pegasus::Demo::GetInstance();
    static std::vector<pegasus::Demo::Primitive*> contactPoints;
    static std::vector<pegasus::Demo::Primitive*> contactNormals;

    if (g_collisionPointsCheckbox)
    {
        for (auto p : contactPoints)
        {
            demo.Remove(*p);
        }
        contactPoints.clear();

        for (auto& c : contacts)
        {
            for (auto& contact : c)
            {
                //contactPoints.push_back(&demo.MakeSphere(contact.manifold.aContactPoint, 0.05, { 1, 0, 0 }));
                contactPoints.push_back(&demo.MakeSphere(contact.manifold.bWorldContactPoint, 0.05, { 0, 1, 0 }));
            }
        }
    }

    if (g_contactNormalsCheckbox)
    {
        for (auto p : contactNormals)
        {
            demo.Remove(*p);
        }
        contactNormals.clear();

        for (auto& c : contacts)
        {
            for (auto& contact : c)
            {
                static pegasus::mechanics::Body line;
                line.linearMotion.position = contact.manifold.bWorldContactPoint;
                contactNormals.push_back(&demo.MakeLine(line, { 1, 0, 0 }, {}, contact.manifold.normal));
            }
        }
    }

    if (g_pauseOnCollisionCheckbox)
    {
        for (auto c : contacts)
        {
            if (!c.empty())
            {
                demo.calculatePhysics = false;
                break;
            }
        }
    }
}

void GjkDebugCallback(
        arion::intersection::gjk::Simplex& simplex, bool end
    )
{
    static pegasus::Demo& demo = pegasus::Demo::GetInstance();
    static pegasus::Demo::Primitive* gjkSimplex = nullptr;

    if (gjkSimplex)
    {
        demo.Remove(*gjkSimplex);
        gjkSimplex = nullptr;
    }

    if (g_gjkSimplexCheckbox)
    {
        switch (simplex.size)
        {
            case 2:
                gjkSimplex = &demo.MakeTriangleCollection({}, { 1, 0, 0 }, {
                    glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[0] }
                });
                break;
            case 3:
                gjkSimplex = &demo.MakeTriangleCollection({}, { 1, 0, 0 }, {
                    glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[2] }
                });
                break;
            case 4:
                gjkSimplex = &demo.MakeTriangleCollection({}, { 1, 0, 0 }, {
                    glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[2] },
                    glm::mat3{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[3] },
                    glm::mat3{ simplex.vertices[1], simplex.vertices[2], simplex.vertices[3] },
                    glm::mat3{ simplex.vertices[0], simplex.vertices[2], simplex.vertices[3] },
                });
                break;
            default:
                break;
        }
    }

    g_gjkHold = end && g_gjkSimplexCheckbox;
    while (g_gjkHold)
    {
        demo.RenderFrame();
    }
}

void EpaDebugCallback(
    epona::QuickhullConvexHull<std::vector<glm::dvec3>>& convexHull,
    std::vector<glm::dvec3>& polytopeVertices,
    arion::intersection::gjk::Simplex& simplex,
    arion::SimpleShape const& aShape,
    arion::SimpleShape const& bShape,
    glm::dvec3 supportVertex,
    glm::dvec3 direction
)
{
    static pegasus::Demo& demo = pegasus::Demo::GetInstance();
    static pegasus::Demo::Primitive* epaPolytope = nullptr;
    static pegasus::Demo::Primitive* sphere = nullptr;
    static pegasus::Demo::Primitive* normal = nullptr;
    static std::vector<pegasus::Demo::Primitive*> csoPrimitives;

    if (epaPolytope)
    {
        demo.Remove(*epaPolytope);
        demo.Remove(*sphere);
        demo.Remove(*normal);
        epaPolytope = nullptr;
        sphere = nullptr;
        normal = nullptr;
    }

    if (!csoPrimitives.empty())
    {
        for (auto p : csoPrimitives)
        {
            demo.Remove(*p);
            p = nullptr;
        }
        csoPrimitives.clear();
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
        sphere = &demo.MakeSphere(supportVertex, 0.05, { 1, 1, 1 });
        normal = &demo.MakeLine({}, { 1, 1, 1 }, supportVertex, supportVertex + direction);
    }

    if (   g_epaCsoCheckbox
        && aShape.type == arion::SimpleShape::Type::BOX
        && bShape.type == arion::SimpleShape::Type::BOX)
    {
        //Calculate box vertices
        auto& aBox = static_cast<arion::Box const&>(aShape);
        auto& bBox = static_cast<arion::Box const&>(bShape);
        static std::vector<glm::dvec3> aVertices{ 8 }, bVertices{ 8 };

        epona::CalculateBoxVerticesWorld(aBox.iAxis, aBox.jAxis, aBox.kAxis, 
            aBox.centerOfMass, glm::toMat3(aBox.orientation), aVertices.begin());
        epona::CalculateBoxVerticesWorld(bBox.iAxis, bBox.jAxis, bBox.kAxis,
            bBox.centerOfMass, glm::toMat3(bBox.orientation), bVertices.begin());

        std::vector<glm::dvec3> csoVertices;
        csoVertices.reserve(8 * 8);
        for (glm::dvec3 a : aVertices)
        {
            csoVertices.push_back(a);
            for (glm::dvec3 b : bVertices)
            {
                csoVertices.push_back(b);
                csoVertices.push_back(a - b);
            }
        }

        //Draw CSO vertices
        for (uint8_t i = 0; i < csoVertices.size(); ++i)
        {
            csoPrimitives.push_back(&demo.MakeSphere(csoVertices[i], 0.03, { 1, 0, 0 }));
        }
    }

    g_epaHold = g_epaPolytopeCheckbox;
    while (g_epaHold)
    {
        demo.RenderFrame();
    }
}

void QuickhullConvexHullCallback(
        epona::QuickhullConvexHull<std::vector<glm::dvec3>>& quickHull,
        std::vector<glm::dvec3>& vertexBuffer
    )
{
    static pegasus::Demo::Primitive* ch = nullptr;
    auto& demo = pegasus::Demo::GetInstance();

    if (g_epaCsoCheckbox)
    {
        if (ch)
        {
            demo.Remove(*ch);
            ch = nullptr;
        } 

        if (!ch)
        {
            std::vector<glm::mat3> triangles;
            auto&  faces = quickHull.GetFaces();
        
            for (auto& face : faces)
            {
                auto const i = face.GetIndices();
                triangles.emplace_back(vertexBuffer[i[0]], vertexBuffer[i[1]], vertexBuffer[i[2]]);
            }

            ch = &demo.MakeTriangleCollection({}, { 1, 1, 0 }, triangles);
        }
    }
}

void DrawUi()
{
    static bool collisionDebugWindowVisible = true;
    static bool sceneConfigWindowVisible = true;
    static bool objectsWindowVisible = true;

    ImGui::BeginMainMenuBar();
    {
        if (ImGui::BeginMenu("Tools"))
        {
            if (ImGui::MenuItem("Objects", "Alt+O")) 
            { 
                objectsWindowVisible = true; 
            }
            if (ImGui::MenuItem("Scene configs", "Alt+S")) 
            { 
                sceneConfigWindowVisible = true; 
            }
            if (ImGui::MenuItem("Collision debug", "Alt+C")) 
            { 
                collisionDebugWindowVisible = true; 
            }
            ImGui::EndMenu();
        }
    }
    ImGui::EndMainMenuBar();

    auto& demo = pegasus::Demo::GetInstance();

    if (collisionDebugWindowVisible)
    {
        ImGui::Begin("Collision debug", &collisionDebugWindowVisible);
        ImGui::Checkbox("Draw GJK simplex", &g_gjkSimplexCheckbox);
        ImGui::Checkbox("Draw EPA polytope", &g_epaPolytopeCheckbox);
        ImGui::Checkbox("Draw CSO polytope", &g_epaCsoCheckbox);
        ImGui::Checkbox("Draw contact points", &g_collisionPointsCheckbox);
        ImGui::Checkbox("Draw contact normals", &g_contactNormalsCheckbox);
        ImGui::End();
    }

    if (sceneConfigWindowVisible)
    {
        ImGui::Begin("Scene configs", &sceneConfigWindowVisible);
        ImGui::Checkbox("Draw origin axes", &g_originAxesCheckbox);
        static std::vector<pegasus::Demo::Primitive*> axes;
        if (g_originAxesCheckbox)
        {
            if (axes.empty())
            {
                axes.push_back(&demo.MakeLine({}, { 1, 0, 0 }, { 0, 0, 0 }, { 1, 0, 0 }));
                axes.push_back(&demo.MakeLine({}, { 0, 1, 0 }, { 0, 0, 0 }, { 0, 1, 0 }));
                axes.push_back(&demo.MakeLine({}, { 0, 0, 1 }, { 0, 0, 0 }, { 0, 0, 1 }));
                axes.push_back(&demo.MakeSphere({ 1, 0, 0 }, 0.05, { 1, 0, 0 }));
                axes.push_back(&demo.MakeSphere({ 0, 1, 0 }, 0.05, { 0, 1, 0 }));
                axes.push_back(&demo.MakeSphere({ 0, 0, 1 }, 0.05, { 0, 0, 1 }));
                axes.push_back(&demo.MakeLine({}, { 1, 0, 0 }, { 0, 0, 0 }, { -1,  0,  0 }));
                axes.push_back(&demo.MakeLine({}, { 0, 1, 0 }, { 0, 0, 0 }, { 0, -1,  0 }));
                axes.push_back(&demo.MakeLine({}, { 0, 0, 1 }, { 0, 0, 0 }, { 0,  0, -1 }));
            }
        } 
        else
        {
            for (auto& axis : axes)
            {
                demo.Remove(*axis);
            }
            axes.clear();
        }

        ImGui::Checkbox("Pause on collision", &g_pauseOnCollisionCheckbox);

        ImGui::Checkbox("Use static frame duration", &demo.useStaticDuration);
        float duration = static_cast<float>(demo.staticDuration);
        ImGui::SliderFloat("Duration", &duration, 0.001f, 0.016f);
        demo.staticDuration = duration;

        if (ImGui::Button(demo.calculatePhysics ? "Pause" : "Run  "))
        {
            demo.calculatePhysics = !demo.calculatePhysics;
        }
        ImGui::SameLine();
        if (ImGui::Button("Next"))
        {
            demo.calculatePhysics = false;
            demo.calculatePhysicsNextFrame = true;
        }
        ImGui::Text("Frame: %.3f ms; (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Spacing();
        ImGui::End();
    }

    if (objectsWindowVisible)
    {
        ImGui::Begin("Objects", &objectsWindowVisible);
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

        static float position[3] = { 0, 0, 0 };
        ImGui::InputFloat3("Position (X Y Z)", position, 3);

        static float angleAxis[4] = {};
        ImGui::InputFloat4("Angle, Axis (X Y Z)", angleAxis, 3);

        static float sphereRadius = 1;
        static float boxSides[3] = { 0.5f, 0.5f, 0.5f };
        if (currentPrimitiveType == 0)
        {
            if (ImGui::InputFloat("Radius", &sphereRadius, 1e-1f, 1e-1f, 3))
            {
                sphereRadius = epona::fp::IsZero(sphereRadius) ? 0.1f : sphereRadius;
            }
            if (ImGui::Button("Random"))
            {
                sphereRadius = rand() % 10 / 10.f;
            }
        }
        else
        {
            ImGui::InputFloat3("Axes (X Y Z)", boxSides, 3);
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

                //Orientation
                glm::vec3 const axis{ angleAxis[1], angleAxis[2], angleAxis[3] };
                glm::vec3 const axisNormalized = epona::fp::IsZero(glm::length(axis)) ? axis : glm::normalize(axis);
                angleAxis[1] = axisNormalized.x;
                angleAxis[2] = axisNormalized.y;
                angleAxis[3] = axisNormalized.z;

                body = g_objects.back()->physicalPrimitive->GetBody();
                body.angularMotion.orientation = glm::dquat(
                    glm::angleAxis(static_cast<double>(angleAxis[0]),
                        glm::dvec3{ angleAxis[1], angleAxis[2], angleAxis[3] }
                ));
                g_objects.back()->physicalPrimitive->SetBody(body);
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Clear"))
        {
            while (!g_objects.empty())
            {
                demo.Remove(*g_objects.back());
                g_objects.pop_back();
            }
        }
        ImGui::End();
    }

    static bool epaGjkHoldWindowVisible = true;
    if (g_epaHold || g_gjkHold)
    {
        ImGui::Begin("EPA debug", &epaGjkHoldWindowVisible);
        g_epaHold = !ImGui::Button("Break EPA hold");
        g_gjkHold = !ImGui::Button("Break GJK hold");
        ImGui::End();
    }
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
    if (calculatePhysics || calculatePhysicsNextFrame)
    {
        ComputeFrame(static_cast<double>(deltaTime.count()) / 1e3);
        calculatePhysicsNextFrame = false;
    }

    std::this_thread::sleep_until(nextFrameTime);
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
    body.material.SetMomentOfInertia(mechanics::CalculateSolidSphereMomentOfInertia(radius, body.material.GetMass()));
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
    body.material.SetMomentOfInertia(mechanics::CalculateSolidCuboidMomentOfInertia(
        glm::length(i), glm::length(j), glm::length(k), body.material.GetMass()));
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
    epona::debug::Debug::SetQuickhullConvexHullCallback<std::vector<glm::dvec3>>(::QuickhullConvexHullCallback);

    auto& arionDebug = arion::debug::Debug::GetInstace();
    arionDebug.epaCallback = ::EpaDebugCallback;
    arionDebug.gjkCallback = ::GjkDebugCallback;

    auto& pegasusDebug = pegasus::debug::Debug::GetInstace();
    pegasusDebug.collisionDetectionCall = ::CollisionDetectionDebugCallback;

    m_renderer.drawUiCallback = ::DrawUi;

    m_scene.Initialize(scene::AssetManager::GetInstance());
    m_pGravityForce = std::make_unique<scene::Force<force::StaticField>>(force::StaticField(glm::dvec3{ 0, -9.8, 0 }));
}

} // namespace pegasus