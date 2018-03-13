/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"

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
            body.linearMotion.position = glm::dvec3(std::rand() % 100 / 10., std::rand() % 100 / 10., std::rand() % 100 / 10.);
            body.linearMotion.velocity = glm::dvec3(std::rand() % 10 / 10., std::rand() % 10 / 10., std::rand() % 10 / 10.);

            static uint8_t isBox = false;
            isBox = !isBox;
            if (isBox)
            {
                glm::vec3 const k = { 0,0,rand() % 10 / 10. + 0.1 };
                glm::vec3 const j = { 0,rand() % 10 / 10. + 0.1,0 };
                glm::vec3 const i = { rand() % 10 / 10. + 0.1,0,0 };

                g_objects.push_back(&demo.MakeBox(
                    body, i, j, k, pegasus::scene::Primitive::Type::DYNAMIC
                ));
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
        while (!g_objects.empty())
        {
            demo.Remove(*g_objects.back());
            g_objects.pop_back();
        }
        break;
    default:
        break;
    }
}

int main(int argc, char** argv)
{
    pegasus::Demo& demo = pegasus::Demo::GetInstance();
    pegasus::render::Input& input = pegasus::render::Input::GetInstance();
    input.AddKeyButtonCallback(KeyButtonCallback);

    //Ground
    pegasus::mechanics::Body plane;
    plane.linearMotion.position = glm::dvec3(0, -5, 0);
    demo.MakePlane(plane, glm::vec3(0, 1, 0), pegasus::scene::Primitive::Type::STATIC);
    
    while (demo.IsValid())
    {
        demo.RunFrame();
    }
}
