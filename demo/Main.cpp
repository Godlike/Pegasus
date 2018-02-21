/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include "demo/Demo.hpp"

int main(int argc, char** argv)
{
    pegasus::Demo& demo = pegasus::Demo::GetInstance();

    //Ground
    pegasus::mechanics::Body plane;
    plane.linearMotion.position = glm::dvec3(0, -1, 0);
    demo.MakePlane(plane, glm::vec3(0, 1, 0), pegasus::scene::Primitive::Type::STATIC);

    //Axes
    demo.MakeLine({}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0});
    demo.MakeLine({}, {0, 1, 0}, {0, 0, 0}, {0, 1, 0});
    demo.MakeLine({}, {0, 0, 1}, {0, 0, 0}, {0, 0, 1});
    demo.MakeSphere({1, 0, 0}, 0.05, {1, 0, 0});
    demo.MakeSphere({0, 1, 0}, 0.05, {0, 1, 0});
    demo.MakeSphere({0, 0, 1}, 0.05, {0, 0, 1});
    demo.MakeLine({}, { 1, 0, 0 }, { 0, 0, 0 }, { -1,  0,  0 });
    demo.MakeLine({}, { 0, 1, 0 }, { 0, 0, 0 }, {  0, -1,  0 });
    demo.MakeLine({}, { 0, 0, 1 }, { 0, 0, 0 }, {  0,  0, -1 });

    while (demo.IsValid())
    {
        demo.RunFrame();
    }
}
