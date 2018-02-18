/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"
#include <Arion/Debug.hpp>
#include <list>

std::list<pegasus::Demo::Primitive*> g_objects;

void KeyButtonCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
        return;

    pegasus::Demo& demo = pegasus::Demo::GetInstance();
    switch (key)
    {
    case GLFW_KEY_M:
        for (uint8_t index = 0; index < demo.maxObjects; ++index)
        {
            pegasus::mechanics::Body body;
            body.linearMotion.position = glm::dvec3(0, std::rand() % 100 / 10., 0);
            //body.linearMotion.position = glm::dvec3(std::rand() % 100 / 10., std::rand() % 100 / 10., std::rand() % 100 / 10.);
            //body.linearMotion.velocity = glm::dvec3(std::rand() % 10 / 10., std::rand() % 10 / 10., std::rand() % 10 / 10.);

            //static uint8_t isBox = false;
            //isBox = !isBox;
            //if (isBox)
            if (true)
            {
                glm::vec3 const k = {0,0,rand() % 10 / 10. + 0.1};
                glm::vec3 const j = {0,rand() % 10 / 10. + 0.1,0};
                glm::vec3 const i = {rand() % 10 / 10. + 0.1,0,0};

                g_objects.push_back(&demo.MakeBox(
                    body, i, j, k, pegasus::scene::Primitive::Type::DYNAMIC
                ));
                //pegasus::mechanics::Body physicalBody = g_objects.back()->physicalPrimitive->GetBody();
                //physicalBody.angularMotion.orientation = glm::angleAxis(
                //    std::rand() % 10 / 10.,
                //    glm::normalize(glm::dvec3{ std::rand() % 10 / 10., std::rand() % 10 / 10., std::rand() % 10 / 10. })
                //);
                //g_objects.back()->physicalPrimitive->SetBody(physicalBody);
            }
            else
            {
                double const radius = rand() % 10 / 10. + 0.1;
                g_objects.push_back(&demo.MakeSphere(
                    body, radius, pegasus::scene::Primitive::Type::DYNAMIC
                ));
            }
        }
        break;
    case GLFW_KEY_R:
        while (g_objects.size() > 1)
        {
            demo.Remove(*g_objects.back());
            g_objects.pop_back();
        }
        break;
    case GLFW_KEY_P:
        if (action == GLFW_RELEASE) 
        {
            demo.calculatePhysics = !demo.calculatePhysics;
        }
        break;
    default:
        break;
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

    if (gjkSimplex) {
        demo.Remove(*gjkSimplex);
        demo.Remove(*epaPolytope);
        gjkSimplex = nullptr;
        epaPolytope = nullptr;
    }

    std::vector<glm::mat3> triangles;
    for (auto& face : convexHull.GetFaces())
    {
        auto indices = face.GetIndices();
        triangles.emplace_back(polytopeVertices[indices[0]], polytopeVertices[indices[1]], polytopeVertices[indices[2]]);
    }

    epaPolytope = &demo.MakeTriangleCollection({}, { 0, 1, 0 }, triangles);
    gjkSimplex = &demo.MakeTriangleCollection({}, { 1, 0, 0 }, { glm::mat3{
            simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]
        }, glm::mat3{
            simplex.vertices[0], simplex.vertices[1], simplex.vertices[3]
        }, glm::mat3{
            simplex.vertices[1], simplex.vertices[2], simplex.vertices[3]
        }, glm::mat3{
            simplex.vertices[0], simplex.vertices[2], simplex.vertices[3]
    }});
}

int main(int argc, char** argv)
{
    pegasus::Demo& demo = pegasus::Demo::GetInstance();
    auto& debug = arion::debug::Debug::GetInstace();
    debug.epaDebugCallback = EpaDebugCallback;

    pegasus::render::Input& input = pegasus::render::Input::GetInstance();
    input.AddKeyButtonCallback(KeyButtonCallback);

    //Ground
    pegasus::mechanics::Body plane;
    plane.linearMotion.position = glm::dvec3(0, -10, 0);
    g_objects.push_back(&demo.MakePlane(plane, glm::vec3(0, 1, 0), pegasus::scene::Primitive::Type::STATIC));

    //Axes
    g_objects.push_back(&demo.MakeLine({}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}));
    g_objects.push_back(&demo.MakeLine({}, {0, 1, 0}, {0, 0, 0}, {0, 1, 0}));
    g_objects.push_back(&demo.MakeLine({}, {0, 0, 1}, {0, 0, 0}, {0, 0, 1}));

    while (demo.IsValid())
    {
        demo.RunFrame();
    }
}
