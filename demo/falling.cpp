#include "demo/app.hpp"
#include "demo/ogl_headers.hpp"
#include "demo/timing.hpp"
#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/mechanics.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include "Pegasus/include/particleworld.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <list>

#define PLANE_COUNT 1
#define BOX_COUNT 1
#define SPHERE_COUNT 1
#define TOTAL_COUNT BOX_COUNT+SPHERE_COUNT+PLANE_COUNT
#define RADIUS 1

class FallingDemo : public Application {
public:
    FallingDemo();
    virtual ~FallingDemo() {}

    const char* getTitle() override;
    void display() override;
    void update() override;
    void key(unsigned char key) override;
    void mouseDrag(int x, int y) override;

private:
    using Particles       = std::list<pegasus::Particle>;
    using RigidBodies     = std::list<pegasus::RigidBody>;
    using ForceGenerators = std::list<std::unique_ptr<pegasus::ParticleForceGenerator>>;

    Particles particles;
    RigidBodies rigidBodies;
    ForceGenerators forces;
    pegasus::ParticleForceRegistry forceRegistry;
    pegasus::ParticleContactGenerators contactGenerators;
    pegasus::ParticleWorld world;

    double xAxis;
    double yAxis;
    double zAxis;
};

// Method definitions
FallingDemo::FallingDemo()
    : world(particles, forceRegistry, contactGenerators, TOTAL_COUNT * TOTAL_COUNT, 10)
    , xAxis(0)
    , yAxis(0)
    , zAxis(0)
{
    //Create particles
    for (unsigned int i = 0; i < TOTAL_COUNT - PLANE_COUNT; ++i) 
    {
        particles.emplace_back();
        particles.back().setPosition(0, double(RADIUS * 3 * (TOTAL_COUNT - PLANE_COUNT - i)), double(0));
        particles.back().setVelocity(0, 0, 0);
        particles.back().setDamping(0.2f);
        particles.back().setMass(1.0f);
    }

    //Create rigid bodies
    for (auto & particle : particles)
    {
        static int i = 0;
        if (i++ < SPHERE_COUNT)
        {
            rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Sphere>(particle.getPosition(), double(RADIUS))
            );
        }
        else 
        {
            rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Box>(
                    particle.getPosition(),
                    pegasus::Vector3{ RADIUS, 0, 0 },
                    pegasus::Vector3{ 0, RADIUS, 0 },
                    pegasus::Vector3{ 0, 0, RADIUS })
            );
        }
    }

    //Create forces
    forces.push_back(std::make_unique<pegasus::ParticleGravity>(pegasus::Vector3{ 0, 0, 0 }));

    //Register forces
    for (auto & particle : particles)
    {
        forceRegistry.add(particle, *forces.front());
    }

    //Create plane particle and rigid body
    particles.emplace_back();
    particles.back().setPosition({1, 0, 0});
    particles.back().setInverseMass(0);
    rigidBodies.emplace_back(
        particles.back(),
        std::make_unique<pegasus::geometry::Plane>(
            particles.back().getPosition(), pegasus::Vector3(0, 1.0, 0).unit()
        )
    );

    //Create contact generators
    for (auto & body : rigidBodies)
    {
        contactGenerators.push_back(
            std::make_unique<pegasus::ShapeContactGenerator<RigidBodies>>(
                body, rigidBodies, 0.0f
            )
        );
    }
}

void FallingDemo::display()
{
    // Clear the view port and set the camera direction
    auto const& pos = particles.front().getPosition();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(pos.x, pos.y + 5, pos.z + 5, pos.x, pos.y, pos.z, 0.0, 1.0, 0.0);

    //Add bodies
    for (auto & body : rigidBodies)
    {
        auto const& p = body.p->getPosition();
        auto const& s = body.s->type;
        glPushMatrix();
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslatef(p.x, p.y, p.z);
        if (s == pegasus::geometry::SimpleShapeType::PLANE) 
        {
            double const planeSideLength = 25;

            auto p0 = static_cast<pegasus::geometry::Plane*>(body.s.get())->getCenterOfMass();
            auto const planeNormal = static_cast<pegasus::geometry::Plane*>(body.s.get())->getNormal();
            auto const posNormalProjection = planeNormal * (p0 * planeNormal);
            auto p1 = p0 + (posNormalProjection - p0) * 2;
            if (p1.x < p0.x) { std::swap(p0.x, p1.x); }
            p1 = ((planeNormal % p1) % planeNormal) * planeSideLength + posNormalProjection;
            p0 = p1.inverse() * planeSideLength + posNormalProjection;

            glBegin(GL_QUADS);
            glColor3f(0.18f, 0.31f, 0.31f);
            glVertex3f(p0.x, p0.y, p0.z + planeSideLength);
            glVertex3f(p1.x, p1.y, p1.z + planeSideLength);
            glVertex3f(p1.x, p1.y, p1.z - planeSideLength);
            glVertex3f(p0.x, p0.y, p0.z - planeSideLength);
            glEnd();
        }
        else if (s == pegasus::geometry::SimpleShapeType::SPHERE)
        {
            glutWireSphere(RADIUS, 20, 20);
            glColor3f(0.0f, 1.0f, 0.0f);
            glutSolidSphere(RADIUS, 20, 20);
        }
        else if (s == pegasus::geometry::SimpleShapeType::BOX)
        {
            glutWireCube(RADIUS * 2);
            glColor3f(0.0f, 0.0f, 1.0f);
            glutSolidCube(RADIUS * 2);
        }
        glPopMatrix();
    }
}

void FallingDemo::update()
{
    world.startFrame();

    auto duration = static_cast<float>(TimingData::get().lastFrameDuration * 0.001f);
    if (duration <= 0.0f)
        return;

    xAxis *= pow(0.1f, duration);
    yAxis *= pow(0.1f, duration);
    zAxis *= pow(0.1f, duration);
    particles.front().addForce(pegasus::Vector3(xAxis * 10.0f, yAxis * 20.0f, zAxis * 10.0f));

    world.runPhysics(0.01f); //(duration);

    for (auto const& body : rigidBodies) {
        body.s->setCenterOfMass(body.p->getPosition());
    }

    Application::update();
}

const char* FallingDemo::getTitle() { return "Pegasus Falling Demo"; }

void FallingDemo::mouseDrag(int x, int y)
{
}

void FallingDemo::key(unsigned char key)
{
    switch (key) {
    case 'w':
    case 'W':
        zAxis = -1.0;
        break;
    case 's':
    case 'S':
        zAxis = 1.0;
        break;
    case 'a':
    case 'A':
        xAxis = -1.0f;
        break;
    case 'd':
    case 'D':
        xAxis = 1.0f;
        break;
    case ' ':
        yAxis = 1.0f;
        break;
    case 'v':
    case 'V':
        yAxis = -1.0f;
        break;
    default:
        break;
    }
}

Application* getApplication() { return new FallingDemo(); }
