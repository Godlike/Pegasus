/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_SCENE_HPP
#define PEGASUS_SCENE_HPP

#include <pegasus/Object.hpp>
#include <pegasus/ParticleForceGenerator.hpp>
#include <pegasus/SharedMacros.hpp>

#include <vector>
#include <list>

namespace pegasus
{

class Scene
{
public:
    PEGASUS_EXPORT void ComputeFrame(double duration);

    std::list<mechanics::Body> bodies;
    std::vector<mechanics::StaticObject> staticObjects;
    std::vector<mechanics::DynamicObject> dynamicObjects;
    ParticleForceRegistry particleForceRegistry;
};
} // namespace pegasus

#endif // PEGASUS_SCENE_HPP
