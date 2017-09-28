/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <pegasus/ParticleWorld.hpp>
#include <demo/Renderer.hpp>

#include <glm/glm.hpp>

#include <chrono>
#include <thread>

namespace pegasus
{
class Demo
{
public:
    struct Object
    {
        Object(RigidBody* body, std::unique_ptr<render::primitive::Primitive>&& renderShape)
            : body(body)
            , shape(std::move(renderShape))
        {
        }

        RigidBody* body;
        std::unique_ptr<render::primitive::Primitive> shape;
    };

    static Demo& GetInstance()
    {
        static Demo demo;
        return demo;
    }

    bool IsValid() const
    {
        return m_pRenderer.IsValid();
    }

    void RunFrame()
    {
        static std::chrono::steady_clock::time_point nextFrameTime = std::chrono::steady_clock::now();
        static std::chrono::milliseconds const deltaTime(16);
        nextFrameTime += deltaTime;

        RenderFrame();
        ComputeFrame(static_cast<double>(deltaTime.count()) / 1e3);

        std::this_thread::sleep_until(nextFrameTime);
    }

    Object& MakeLine(Particle particle, glm::vec3 start, glm::vec3 end)
    {
        m_objects.emplace_back(
            nullptr,
            std::make_unique<render::primitive::LineSegment>(
                glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), start, end)
        );
        
        return m_objects.back();
    }

    Object& MakePlane(Particle particle, glm::dvec3 normal)
    {
        m_objects.emplace_back(
            &MakeRigidBody(particle, std::make_unique<geometry::Plane>(particle.GetPosition(), normal)),
            std::make_unique<render::primitive::Plane>(
                glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), normal)
        );
        m_particleContactGenerators.emplace_back(
            std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
        );

        return m_objects.back();
    }

    Object& MakeSphere(Particle particle, double radius)
    {
        m_objects.emplace_back(
            &MakeRigidBody(particle, std::make_unique<geometry::Sphere>(particle.GetPosition(), radius)),
            std::make_unique<render::primitive::Sphere>(
                glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())), glm::vec3(0.439, 0.502, 0.565), radius)
        );
        m_particleContactGenerators.emplace_back(
            std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
        );

        return m_objects.back();
    }

    Object& MakeBox(Particle particle, glm::vec3 i, glm::vec3 j, glm::vec3 k)
    {
        m_objects.emplace_back(
            &MakeRigidBody(particle, std::make_unique<geometry::Box>(particle.GetPosition(), i, j, k)),
            std::make_unique<render::primitive::Box>(
                glm::translate(glm::mat4(1), glm::vec3(particle.GetPosition())),
                glm::vec3(0.439, 0.502, 0.565),
                render::primitive::Box::Axes{i, j, k}
            )
        );
        m_particleContactGenerators.emplace_back(
            std::make_unique<ShapeContactGenerator<RigidBodies>>(*m_objects.back().body, m_rigidBodies, 0.7)
        );

        return m_objects.back();
    }

    void Remove(Object& object)
    {
        if (object.body != nullptr)
        {
            Particle& particle = object.body->p;
            m_particleForceRegistry.Remove(particle);

            for (auto it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
                if (&it->p == &particle) {
                    m_rigidBodies.erase(it);
                    break;
                }
            }
            for (auto it = m_particles.begin(); it != m_particles.end(); ++it) {
                if (&*it == &particle) {
                    m_particles.erase(it);
                    break;
                }
            }
        }

        for (auto it = m_objects.begin(); it != m_objects.end(); ++it) {
            if (&*it == &object) {
                m_objects.erase(it);
                break;
            }
        }

        m_particleContactGenerators.clear();
        for (Object& obj : m_objects)
        {
            if (obj.body != nullptr)
            {
                m_particleContactGenerators.emplace_back(
                    std::make_unique<ShapeContactGenerator<RigidBodies>>(*obj.body, m_rigidBodies, 0.7)
                );
            }
        }
    }

    uint32_t const maxParticles = 200;

private:
    std::list<Object> m_objects;
    render::Renderer& m_pRenderer;
    ParticleWorld m_particleWorld;
    Particles m_particles;
    RigidBodies m_rigidBodies;
    ParticleContactGenerators m_particleContactGenerators;
    ParticleForceRegistry m_particleForceRegistry;
    ParticleGravity m_gravityForce;

    Demo()
        : m_pRenderer(render::Renderer::GetInstance())
        , m_particleWorld(m_particles,
            m_particleForceRegistry,
            m_particleContactGenerators,
            glm::pow2(maxParticles),
            maxParticles)
        , m_gravityForce(glm::dvec3{0, -9.8, 0})
    {
    }

    void ComputeFrame(double duration)
    {
        m_particleWorld.StartFrame();
        m_particleWorld.RunPhysics(duration);

        //Update positions
        for (Object& object : m_objects)
        {
            if (object.body != nullptr)
            {
                object.body->s->centerOfMass = object.body->p.GetPosition();
                glm::mat4 const model = glm::translate(glm::mat4(1), glm::vec3(object.body->s->centerOfMass));
                object.shape->SetModel(model);
            }
        }
    }

    void RenderFrame() const
    {
        m_pRenderer.RenderFrame();
    }

    RigidBody& MakeRigidBody(Particle particle, std::unique_ptr<geometry::SimpleShape>&& shape)
    {
        m_particles.push_back(particle);
        m_particleForceRegistry.Add(m_particles.back(), m_gravityForce);
        m_rigidBodies.emplace_back(m_particles.back(), std::move(shape));

        return m_rigidBodies.back();
    }
};
} // namespace pegasus

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
