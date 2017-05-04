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

#define BLOB_COUNT 5
#define PLATFORM_COUNT 5
#define BLOB_RADIUS 0.5f

class BlobDemo : public Application {
public:
    BlobDemo();
    virtual ~BlobDemo();

    const char* getTitle() override;

    void display() override;

    void update() override;

    void key(unsigned char key) override;

private:
    pegas::RigidBodies rBodies;
    std::vector<pegas::Particle::Ptr> blobs;
    std::vector<pegas::ParticleContactGenerator::Ptr> contactGenerators;
    pegas::ParticleForceRegistry::Ptr forceRegistry;
    pegas::ParticleWorld world;

    float xAxis;
    float yAxis;

    void reset();
};

// Method definitions
BlobDemo::BlobDemo()
    : forceRegistry(std::make_shared<pegas::ParticleForceRegistry>())
    , world(PLATFORM_COUNT + BLOB_COUNT + 1, PLATFORM_COUNT + 1)
    , xAxis(0)
    , yAxis(0)
{
    // Create the platforms
    for (unsigned int i = 0; i < PLATFORM_COUNT; ++i) {
        auto const start = pegas::Vector3(pegas::real(i % 2) * 10.0f - 5.0f,
            pegas::real(i) * 4.0f + ((i % 2) ? 0.0f : 2.0f), 0);

        auto const end = pegas::Vector3(pegas::real(i % 2) * 10.0f + 5.0f,
            pegas::real(i) * 4.0f + ((i % 2) ? 2.0f : 0.0f), 0);

        contactGenerators.push_back(std::make_shared<pegas::Platform>(start, end, blobs, BLOB_RADIUS));
    }

    auto& p = *static_cast<pegas::Platform*>(contactGenerators.back().get());

    for (unsigned int i = 0; i < BLOB_COUNT; ++i) {
        auto blob = std::make_shared<pegas::Particle>();
        blob->setPosition(p.start + pegas::Vector3(5 + (std::rand() % 50) / 100.0f, i * 2 + BLOB_RADIUS * 2, 0));

        blob->setVelocity(0, 0, 0);
        blob->setDamping(0.2f);
        blob->setAcceleration(pegas::Vector3(pegas::real(0), pegas::real(-9.8), pegas::real(0)) * pegas::real(0.4));
        blob->setMass(1.0f);
        blobs.push_back(blob);

        rBodies.push_back(std::make_shared<pegas::RigidBody>(blob,
            std::make_shared<pegas::gmt::Sphere>(blob->getPosition(), BLOB_RADIUS)));
    }

    for (auto body : rBodies) {
        contactGenerators.push_back(std::make_shared<pegas::SphereContactGenerator>(body, rBodies, static_cast<pegas::real>(0)));
    }

    world.setParticleContactGenerators(contactGenerators);
    world.setParticles(blobs);
    world.setParticleForcesRegistry(forceRegistry);
}

BlobDemo::~BlobDemo()
{
}

void BlobDemo::reset()
{
    auto p = static_cast<pegas::Platform*>(contactGenerators.back().get());
    auto fraction = static_cast<pegas::real>(1) / BLOB_COUNT;
    auto delta = p->end - p->start;
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        auto me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
        blobs[i]->setPosition(p->start + delta * (pegas::real(me) * 0.8f * fraction + 0.1f) + pegas::Vector3(0, 1, 0));
        blobs[i]->setVelocity(0, 0, 0);
        blobs[i]->clearForceAccum();
    }
}

void BlobDemo::display()
{
    pegas::Vector3 pos = blobs.front()->getPosition();

    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(pos.x, pos.y, 5, pos.x, pos.y, 0.0, 0.0, 1.0, 0.0);

    glColor3f(0, 0, 0);

    glBegin(GL_LINES);
    glColor3f(0, 0, 1);
    for (unsigned i = 0; i < PLATFORM_COUNT; i++) {
        auto const& p0 = static_cast<pegas::Platform*>(contactGenerators[i].get())->start;
        auto const& p1 = static_cast<pegas::Platform*>(contactGenerators[i].get())->end;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }
    glEnd();

    for (pegas::real i = 0; i < BLOB_COUNT; i++) {
        auto const& p = blobs[static_cast<int>(i)]->getPosition();
        glPushMatrix();
        glColor3f((i + 1) / BLOB_COUNT, (i + 1) / BLOB_COUNT, (i + 1) / BLOB_COUNT);
        glTranslatef(p.x, p.y, p.z);
        glutSolidSphere(BLOB_RADIUS, 12, 12);
        glPopMatrix();
    }

    pegas::Vector3 p = blobs.front()->getPosition();
    pegas::Vector3 v = blobs.front()->getVelocity() * 0.05f;
    v.trim(BLOB_RADIUS * 0.5f);
    p = p + v;
    glPushMatrix();
    glTranslatef(p.x - BLOB_RADIUS * 0.2f, p.y, BLOB_RADIUS);
    glColor3f(1, 1, 1);
    glutSolidSphere(BLOB_RADIUS * 0.2f, 8, 8);
    glTranslatef(0, 0, BLOB_RADIUS * 0.2f);
    glColor3f(0, 0, 0);
    glutSolidSphere(BLOB_RADIUS * 0.1f, 8, 8);
    glTranslatef(BLOB_RADIUS * 0.4f, 0, -BLOB_RADIUS * 0.2f);
    glColor3f(1, 1, 1);
    glutSolidSphere(BLOB_RADIUS * 0.2f, 8, 8);
    glTranslatef(0, 0, BLOB_RADIUS * 0.2f);
    glColor3f(0, 0, 0);
    glutSolidSphere(BLOB_RADIUS * 0.1f, 8, 8);
    glPopMatrix();
}

void BlobDemo::update()
{
    // Clear accumulators
    world.startFrame();

    // Find the duration of the last frame in seconds
    auto duration = static_cast<float>(TimingData::get().lastFrameDuration * 0.001f);
    if (duration <= 0.0f)
        return;

    // Recenter the axes
    xAxis *= pow(0.1f, duration);
    yAxis *= pow(0.1f, duration);

    // Move the controlled blob
    blobs.front()->addForce(pegas::Vector3(xAxis, yAxis, 0) * 10.0f);

    // Run the simulation
    world.runPhysics(duration);

    for (auto body : rBodies) {
        body->s->setCenterOfMass(body->p->getPosition());
    }

    Application::update();
}

const char* BlobDemo::getTitle() { return "pegas > Blob Demo"; }

void BlobDemo::key(unsigned char key)
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
    case 'r':
    case 'R':
        reset();
        break;
    default:
        break;
    }
}

Application* getApplication() { return new BlobDemo(); }
