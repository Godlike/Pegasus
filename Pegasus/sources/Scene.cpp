/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Scene.hpp>
#include <pegasus/Collision.hpp>
#include <pegasus/Integration.hpp>

void pegasus::Scene::ComputeFrame(double duration)
{
    for (mechanics::Body& body : bodies)
    {
        body.linearMotion.force = glm::dvec3(0);
    }

    static collision::Detector detector;
    std::vector<collision::Contact> dContacts = detector.Detect(dynamicObjects);
    std::vector<collision::Contact> dsContacts = detector.Detect(dynamicObjects, staticObjects);

    static collision::Resolver resolver;
    resolver.Resolve(dContacts, duration);
    resolver.Resolve(dsContacts, duration);

    particleForceRegistry.ApplyForces();

    for (mechanics::DynamicObject& dynamicObject : dynamicObjects)
    {
        Integrate(*dynamicObject.body, duration);
    }
}
