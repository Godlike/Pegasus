#include "Pegasus/demo/app.hpp"
#include "Pegasus/demo/ogl_headers.hpp"
#include "Pegasus/demo/timing.hpp"
#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/mechanics.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include "Pegasus/include/particleworld.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#define BLOB_COUNT 2
#define BLOB_RADIUS 1

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
    pegasus::RigidBodies rigidBodies;
    std::vector<pegasus::Particle::Ptr> blobs;
    std::vector<pegasus::ParticleContactGenerator::Ptr> contactGenerators;
    pegasus::ParticleForceRegistry::Ptr forceRegistry;
    pegasus::ParticleWorld world;

    float xAxis;
    float yAxis;
};

// Method definitions
FallingDemo::FallingDemo()
    : forceRegistry(std::make_shared<pegasus::ParticleForceRegistry>())
    , world(1 + BLOB_COUNT + 1, 1 + 1)
    , xAxis(0)
    , yAxis(0)
{
    // Create ground
    contactGenerators.push_back(std::make_shared<pegasus::Platform>(
        pegasus::Vector3(-50.0f, 0, 0), pegasus::Vector3(50.0f, 0, 0), blobs, BLOB_RADIUS));
    auto const & platform = *static_cast<pegasus::Platform*>(contactGenerators.back().get());

    //Create particles
    for (unsigned int i = 0; i < BLOB_COUNT; ++i) {
        auto particle = std::make_shared<pegasus::Particle>();
        particle->setPosition(platform.start + pegasus::Vector3((platform.end.x - platform.start.x) / 2.0f + BLOB_RADIUS * i * 4, BLOB_RADIUS, 0));

        particle->setVelocity(0, 0, 0);
        particle->setDamping(0.2f);
        particle->setAcceleration(pegasus::Vector3(0, -9.8f, 0) * 0.4f);
        particle->setMass(1.0f);
        blobs.push_back(particle);

        rigidBodies.push_back(std::make_shared<pegasus::RigidBody>(particle,
            std::make_shared<pegasus::geometry::Box>(
                particle->getPosition(), 
                pegasus::Vector3{ BLOB_RADIUS, 0, 0 }, 
                pegasus::Vector3{ 0, BLOB_RADIUS, 0 }, 
                pegasus::Vector3{ 0, 0, BLOB_RADIUS })
        ));
    }

    //Create rigid bodies
    for (auto const & body : rigidBodies) {
        contactGenerators.push_back(
            std::make_shared<pegasus::ShapeContactGenerator<pegasus::geometry::Box, pegasus::geometry::Box>>(
                body, rigidBodies, static_cast<pegasus::real>(0)));
    }

    //Init world
    world.setParticleContactGenerators(contactGenerators);
    world.setParticles(blobs);
    world.setParticleForcesRegistry(forceRegistry);
}

void FallingDemo::display()
{
    // Clear the view port and set the camera direction
    auto const & pos = blobs.front()->getPosition();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(pos.x + 10, pos.y + 10, 5, pos.x, pos.y, 0.0, 0.0, 1.0, 0.0);

    //Add ground plane
    glBegin(GL_QUADS);
    glColor3f(0.18f, 0.31f, 0.31f);
    auto const& p0 = static_cast<pegasus::Platform*>(contactGenerators.front().get())->start;
    auto const& p1 = static_cast<pegasus::Platform*>(contactGenerators.front().get())->end;
    glVertex3f(p0.x, p0.y, p0.z + 25);
    glVertex3f(p1.x, p1.y, p1.z + 25);
    glVertex3f(p1.x, p1.y, p1.z - 25);
    glVertex3f(p0.x, p0.y, p0.z - 25);
    glEnd();

    //Add bodies
    for (pegasus::real i = 0; i < BLOB_COUNT; i++) 
    {
        auto const& p = blobs[static_cast<int>(i)]->getPosition();
        glPushMatrix();
        glColor3f((i + 1) / BLOB_COUNT, (i + 1) / BLOB_COUNT, (i + 1) / BLOB_COUNT);
        glTranslatef(p.x, p.y, p.z);
        glutSolidCube(BLOB_RADIUS * 2);
        glPopMatrix();

        glPushMatrix();
        glColor3f(1.0f - (i + 1) / BLOB_COUNT, 1.0f - (i + 1) / BLOB_COUNT, 1.0f - (i + 1) / BLOB_COUNT);
        glTranslatef(p.x, p.y, p.z);
        glutWireCube(BLOB_RADIUS * 2);
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

    // Move the controlled blob
    blobs.front()->addForce(pegasus::Vector3(xAxis, yAxis, 0) * 10.0f);

    world.runPhysics(duration);

    for (auto const & body : rigidBodies) {
        body->s->setCenterOfMass(body->p->getPosition());
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
        yAxis = 1.0;
        break;
    case 's':
    case 'S':
        yAxis = -1.0;
        break;
    case 'a':
    case 'A':
        xAxis = -1.0f;
        break;
    case 'd':
    case 'D':
        xAxis = 1.0f;
        break;
    default:
        break;
    }
}

Application* getApplication() { return new FallingDemo(); }
