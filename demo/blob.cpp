#include "Pegasus/demo/app.hpp"
#include "Pegasus/demo/ogl_headers.hpp"
#include "Pegasus/demo/timing.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include "Pegasus/include/particleworld.hpp"
#include "Pegasus/include/particlelinks.hpp"

#include <cmath>
#include <cstdlib>
#include <stdio.h>

#define BLOB_COUNT 5
#define PLATFORM_COUNT 10
#define BLOB_RADIUS 0.3f

class BlobDemo : public Application {
public:
    BlobDemo();

    virtual const char* getTitle() override;

    virtual void display() override;

    virtual void update() override;

    virtual void key(unsigned char key) override;

private:
    std::vector<pegas::Particle::Ptr> blobs;
    std::vector<pegas::ParticleContactGenerator::Ptr> contactGenerators;
    pegas::BlobForceGenerator::Ptr blobForceGenerator;
    pegas::ParticleForceRegistry::Ptr forceRegistry;
    pegas::ParticleWorld world;

    float xAxis;
    float yAxis;

    void reset();
};

// Method definitions
BlobDemo::BlobDemo()
    : blobForceGenerator(std::make_shared<pegas::BlobForceGenerator>(blobs))
    , forceRegistry(std::make_shared<pegas::ParticleForceRegistry>())
    , world(PLATFORM_COUNT + BLOB_COUNT + 1, PLATFORM_COUNT + 1)
    , xAxis(0)
    , yAxis(0)
{
    // Create the force generator
    pegas::BlobForceGenerator& blobForce = *static_cast<pegas::BlobForceGenerator*>(blobForceGenerator.get());
    blobForce.particles = blobs;
    blobForce.maxAttraction = 20.0f;
    blobForce.maxReplusion = 10.0f;
    blobForce.minNaturalDistance = BLOB_RADIUS * 0.75f;
    blobForce.maxNaturalDistance = BLOB_RADIUS * 1.5f;
    blobForce.maxDistance = BLOB_RADIUS * 2.5f;
    blobForce.maxFloat = 2;
    blobForce.floatHead = 8.0f;

    // Create the platforms
    for (unsigned int i = 0; i < PLATFORM_COUNT; ++i) {
        auto const start = pegas::Vector3(pegas::real(i % 2) * 10.0f - 5.0f,
            pegas::real(i) * 4.0f + ((i % 2) ? 0.0f : 2.0f), 0);

        auto const end = pegas::Vector3(pegas::real(i % 2) * 10.0f + 5.0f,
            pegas::real(i) * 4.0f + ((i % 2) ? 2.0f : 0.0f), 0);

        contactGenerators.push_back(std::make_shared<pegas::Platform>(start, end, blobs, BLOB_RADIUS));
    }

    pegas::Platform& p = *static_cast<pegas::Platform*>(contactGenerators.back().get());
    pegas::real const fraction = pegas::real(1.0) / BLOB_COUNT;
    pegas::Vector3 delta = p.end - p.start;

    for (unsigned int i = 0; i < BLOB_COUNT; ++i) {
        auto blob = std::make_shared<pegas::Particle>();
        auto const me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
        blob->setPosition(p.start + delta * (pegas::real(me) * 0.8f * fraction + 0.1f) + pegas::Vector3(0, 1, 0));

        blob->setVelocity(0, 0, 0);
        blob->setDamping(0.2f);
        blob->setAcceleration(pegas::Vector3(0, -9.8, 0) * 0.4f);
        blob->setMass(1.0f);
        blob->clearForceAccum();
        blobs.push_back(blob);
    }

    for (auto& blob : blobs) {
        forceRegistry->add(blob, blobForceGenerator);
    }

    contactGenerators.push_back(
        std::make_shared<pegas::ParticleRod>(
            blobs.front(), blobs.back(), (blobs.back()->getPosition() - blobs.front()->getPosition()).magnitude()));

    world.setParticleContactGenerators(contactGenerators);
    world.setParticles(blobs);
    world.setParticleForcesRegistry(forceRegistry);
}

void BlobDemo::reset()
{
    auto p = static_cast<pegas::Platform*>(contactGenerators.back().get());
    pegas::real fraction = (pegas::real)1.0 / BLOB_COUNT;
    pegas::Vector3 delta = p->end - p->start;
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        unsigned me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
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
    gluLookAt(pos.x, pos.y, 6.0, pos.x, pos.y, 0.0, 0.0, 1.0, 0.0);

    glColor3f(0, 0, 0);

    glBegin(GL_LINES);
    glColor3f(0, 0, 1);
    for (unsigned i = 0; i < PLATFORM_COUNT; i++) {
        const pegas::Vector3& p0 = static_cast<pegas::Platform*>(contactGenerators[i].get())->start;
        const pegas::Vector3& p1 = static_cast<pegas::Platform*>(contactGenerators[i].get())->end;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }
    glEnd();

    pegas::Vector3 pos1 = blobs.back()->getPosition();
    glBegin(GL_LINES);
    glColor3f(0, 0, 0);
    glVertex3f(pos.x, pos.y, pos.z);
    glVertex3f(pos1.x, pos1.y, pos1.z);
    glEnd();

    for (pegas::real i = 0; i < BLOB_COUNT; i++) {
        const pegas::Vector3& p = blobs[i]->getPosition();
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
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f)
        return;

    // Recenter the axes
    xAxis *= pow(0.1f, duration);
    yAxis *= pow(0.1f, duration);

    // Move the controlled blob
    blobs.front()->addForce(pegas::Vector3(xAxis, yAxis, 0) * 10.0f);

    // Run the simulation
    world.runPhysics(duration);

    // Bring all the particles back to 2d
    pegas::Vector3 position;
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        position = blobs[i]->getPosition();
        position.z = 0.0f;
        blobs[i]->setPosition(position);
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
    }
}

Application* getApplication() { return new BlobDemo(); }
