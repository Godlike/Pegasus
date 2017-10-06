/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"

std::list<pegasus::Demo::Object*> g_objects;

void KeyButtonCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
        return;

    pegasus::Demo& demo = pegasus::Demo::GetInstance();
    switch (key)
    {
    case GLFW_KEY_M:
        if (g_objects.size() < demo.maxParticles)
        {
            pegasus::Particle particle;
            particle.SetPosition(std::rand() % 100 / 10., std::rand() % 100 / 10., std::rand() % 100 / 10.);
            particle.SetVelocity(std::rand() % 10 / 10., std::rand() % 10 / 10., std::rand() % 10 / 10.);

            static uint8_t isBox = false;
            isBox = !isBox;
            if (isBox)
                g_objects.push_back(&demo.MakeBox(
                    particle, {rand() % 10 / 10. + 0.1,0,0}, {0,rand() % 10 / 10. + 0.1,0}, {0,0,rand() % 10 / 10. + 0.1}
                ));
            else
                g_objects.push_back(&demo.MakeSphere(particle, rand() % 10 / 10. + 0.1));
        }
        break;
    case GLFW_KEY_R:
        if (g_objects.size() > 1)
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

    pegasus::Particle plane;
    plane.SetPosition(0, -10, 0);
    plane.SetInverseMass(0);
    g_objects.push_back(&demo.MakePlane(plane, glm::vec3(0, 1, 0)));

    pegasus::Particle line;
    line.SetPosition(0, -10, 0);
    g_objects.push_back(&demo.MakeLine(line, line.GetPosition(), glm::vec3(0, 10, 0)));

    while (demo.IsValid())
    {
        demo.RunFrame();
    }
}
