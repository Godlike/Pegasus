/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEMO_HPP
#define PEGASUS_DEMO_HPP

#include <pegasus/ParticleWorld.hpp>
#include <demo/Renderer.hpp>

#include <chrono>
#include <thread>

namespace pegasus
{
class Demo
{
public:
    struct Object;

    static Demo& GetInstance();

    bool IsValid() const;

    void RunFrame();

    Object& MakeLine(Particle particle, glm::vec3 start, glm::vec3 end);

    Object& MakePlane(Particle particle, glm::dvec3 normal);

    Object& MakeSphere(Particle particle, double radius);

    Object& MakeBox(Particle particle, glm::vec3 i, glm::vec3 j, glm::vec3 k);

    void Remove(Object& object);

    uint32_t const maxParticles = 200;

    struct Object
    {
        Object(RigidBody* body, std::unique_ptr<render::primitive::Primitive>&& renderShape);

        RigidBody* body;
        std::unique_ptr<render::primitive::Primitive> shape;
    };

private:
    std::list<Object> m_objects;
    render::Renderer& m_pRenderer;
    ParticleWorld m_particleWorld;
    Particles m_particles;
    RigidBodies m_rigidBodies;
    ParticleContactGenerators m_particleContactGenerators;
    ParticleForceRegistry m_particleForceRegistry;
    ParticleGravity m_gravityForce;

    Demo();

    void ComputeFrame(double duration);

    void RenderFrame() const;

    RigidBody& MakeRigidBody(Particle particle, std::unique_ptr<geometry::SimpleShape>&& shape);
};
} // namespace pegasus

#endif // PEGASUS_DEMO_HPP
