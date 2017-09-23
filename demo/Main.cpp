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
    pegasus::render::Renderer& render = pegasus::render::Renderer::GetInstance();
    pegasus::render::Input& input = pegasus::render::Input::GetInstance();

    std::vector<pegasus::render::primitive::Box> boxes;
    for (uint8_t i = 0; i < 10; ++i)
    {
        pegasus::render::primitive::Box::Axes const axes{{0.5, 0, 0},{0, 0.5, 0},{0, 0, 0.5}};
        boxes.emplace_back(render, glm::mat4(1), axes, glm::vec3{1, 0, 0});

        const float angle = 36.0f * i;
        glm::vec4 const vec = glm::rotate(glm::mat4(1), glm::radians(angle), glm::vec3{0, 1, 0}) * glm::vec4(10, 0, 0, 1);
        boxes.back().SetModel(glm::translate(glm::mat4(1),  glm::vec3(vec.x, vec.y, vec.z)));
    }

    while (render.IsValid())
    {
        static std::chrono::steady_clock::time_point nextFrameTime = std::chrono::steady_clock::now();
        static std::chrono::milliseconds const deltaTime(16);
        nextFrameTime += deltaTime;

        for (auto & box : boxes)
        {
            box.SetModel(glm::rotate(box.GetModel(), 0.01f, glm::vec3(1, 1, 1)));
        }

        render.RenderFrame();
        std::this_thread::sleep_until(nextFrameTime);
    }
}
