/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <demo/Renderer.hpp>

#include <glm/glm.hpp>

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    pegasus::render::Renderer render;

    pegasus::render::primitive::Plane plane(render, glm::mat4(), glm::normalize(glm::dvec3{0, 0, 1}), 0.0);
    pegasus::render::primitive::Sphere sphere(render, glm::mat4(), 0.5, { 1,0,0 });
    pegasus::render::primitive::Box box(render, glm::mat4(), {{ 0.1,0,0 },{ 0,0.1,0 },{ 0,0,0.1 }}, { 1,0,0 });

    while (render.IsValid())
    {
        static std::chrono::steady_clock::time_point nextFrameTime = std::chrono::steady_clock::now();
        static std::chrono::milliseconds const deltaTime(16);
        nextFrameTime += deltaTime;

        plane.SetModel(glm::rotate(plane.GetModel(), 0.01f, glm::vec3(1, 0.5, 0.5)));
        sphere.SetModel(glm::rotate(sphere.GetModel(), 0.01f, glm::vec3(0.5, 1, 0.5)));
        box.SetModel(glm::rotate(box.GetModel(), 0.01f, glm::vec3(0.5, 0.5, 1)));

        render.RenderFrame();
        std::this_thread::sleep_until(nextFrameTime);
    }
}
