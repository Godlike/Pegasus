/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "demo/app.hpp"
#include "demo/ogl_headers.hpp"
#include "demo/timing.hpp"
#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/mechanics.hpp"
#include "Pegasus/include/particlecontacts.hpp"
#include "Pegasus/include/particleforcegenerator.hpp"
#include "Pegasus/include/particleworld.hpp"

#include <cassert>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <list>
#include <random>
#include <utility>

static const uint32_t BOX_COUNT    = std::pow(3, 3);
static const uint32_t SPHERE_COUNT = std::pow(3, 3);
static const uint32_t TOTAL_COUNT  = BOX_COUNT+SPHERE_COUNT;
static const double   RADIUS       = 5;

class FallingDemo : public Application {
public:
    FallingDemo();
    virtual ~FallingDemo() {}

    const char* getTitle() override;
    void display() override;
    void update() override;
    void key(unsigned char key) override;
    static void displayText(float x, float y, int r, int g, int b, void* vptr);

    void addBox(pegasus::Vector3 const & pos, double boxSide);
    void addSphere(pegasus::Vector3 const & pos, double radius);

    void sceneReset();

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
    double zoom;
    double yEye;
    RigidBodies::iterator activeObject;
};

// Method definitions
FallingDemo::FallingDemo()
    : world(particles, forceRegistry, contactGenerators, TOTAL_COUNT * TOTAL_COUNT, 5000)
    , xAxis(0)
    , yAxis(0)
    , zAxis(0)
    , zoom(8)
    , yEye(0)
    , activeObject(rigidBodies.begin())
{
    sceneReset();
}

void FallingDemo::addBox(pegasus::Vector3 const & pos, double boxSide)
{
    particles.emplace_back();
    particles.back().setPosition(pos);
    particles.back().setInverseMass(0);
    rigidBodies.emplace_back(
        particles.back(),
        std::make_unique<pegasus::geometry::Box>(
            pos,
            pegasus::Vector3{ boxSide, 0, 0 },
            pegasus::Vector3{ 0, boxSide, 0 },
            pegasus::Vector3{ 0, 0, boxSide })
    );
}

void FallingDemo::addSphere(pegasus::Vector3 const & pos, double radius)
{
    particles.emplace_back();
    particles.back().setPosition(pos);
    particles.back().setInverseMass(0);
    rigidBodies.emplace_back(
        particles.back(),
        std::make_unique<pegasus::geometry::Sphere>(pos, radius)
    );
}

void FallingDemo::sceneReset()
{
    rigidBodies.clear();
    forces.clear();
    forceRegistry.clear();
    contactGenerators.clear();
    particles.clear();

    double const boxSide  = 15.0;
    double const position = boxSide;

    static auto randDouble = [](){
        static std::default_random_engine generator;
        static std::uniform_real_distribution<double> distribution(-5.0, 5.0);
        return distribution(generator);
    };

    //Create particles
    for (uint32_t i = 0; i < TOTAL_COUNT; ++i)
    {
        static auto curt = [](auto n) {
            return std::pow(n, 0.34);
        };
        static const int edge  = curt(TOTAL_COUNT);
        static const int plane = edge * edge;
        static const double offset = edge * RADIUS;

        int planeIndex = i / plane;
        int index2d = i - planeIndex * plane;
        int row = index2d / edge;
        int col = index2d - row * edge;

        particles.emplace_back();
        particles.back().setPosition(
            row * RADIUS * 2.3 + RADIUS - offset,
            planeIndex * RADIUS * 2.3 + boxSide * 3,
            col * RADIUS * 2.3 + RADIUS - offset
        );
        particles.back().setVelocity(randDouble(), randDouble() - 5, randDouble());
        particles.back().setDamping(1.0f);
    }

    //Create rigid bodies
    for (auto & particle : particles)
    {
        bool const isBox = randDouble() > 0;

        if (isBox)
        {
            rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Box>(
                    particle.getPosition(),
                    pegasus::Vector3{ RADIUS, 0, 0 } * 0.5,
                    pegasus::Vector3{ 0, RADIUS, 0 } * 0.5,
                    pegasus::Vector3{ 0, 0, RADIUS } * 0.5)
            );
        }
        else
        {
            rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Sphere>(particle.getPosition(), double(randDouble() + 6) / 2)
            );
        }
    }

    //Create forces
    forces.push_back(std::make_unique<pegasus::ParticleGravity>(pegasus::Vector3{ 0, -9.8, 0 }));

    //Register forces
    for (auto & particle : particles)
    {
        forceRegistry.add(particle, *forces.front());
    }

    //Create contact generators
    for (auto & body : rigidBodies)
    {
        contactGenerators.push_back(
            std::make_unique<pegasus::ShapeContactGenerator<RigidBodies>>(
                body, rigidBodies, (randDouble() + 5) / 10
            )
        );
    }

    //Create plane particle and rigid body
    particles.emplace_front();
    particles.front().setPosition({1, -position * 2, 0});
    particles.front().setInverseMass(0);
    rigidBodies.emplace_front(
        particles.front(),
        std::make_unique<pegasus::geometry::Plane>(
           particles.front().getPosition(), pegasus::Vector3(0, 1.0, 0).unit()
        )
    );

    addSphere({  0, -position * 2, 0 }, boxSide);
    addBox({  position * 2, position, 0 }, boxSide);
    addBox({ -position * 2, position, 0 }, boxSide);
    addBox({ 0, position, -position * 2 }, boxSide);

    activeObject = rigidBodies.end();
    std::advance(activeObject, -4);
}

void FallingDemo::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    auto const& pos = activeObject->p.getPosition();
    gluLookAt(pos.x, pos.y + 30 * zoom + yEye * zoom, pos.z + 30 * zoom, pos.x, pos.y, pos.z, 0.0, 1.0, 0.0);

    displayText(0, 0, 0x7FFFFFFF, 0, 0, &activeObject->p);

    //Add bodies
    for (auto & body : rigidBodies)
    {
        glPushMatrix();
        glColor3f(1.0f, 0.0f, 0.0f);

        static int index = -1;
        index += 1;
        index %= rigidBodies.size();

        auto const& p = body.p.getPosition();
        auto const& s = body.s->type;

        int const kekdex = index + 1;
        double red  = (double)(0xb00b1e55 % kekdex) / (double)kekdex;
        double glin = (double)(0x31337420 % kekdex) / (double)kekdex;
        double blue = (double)(0xdeadbeef % kekdex) / (double)kekdex;

        if (s == pegasus::geometry::SimpleShapeType::PLANE) 
        {
            glTranslatef(0, 0, 0);
            double const planeSideLength = 100;

            pegasus::Vector3 p0 = static_cast<pegasus::geometry::Plane*>(body.s.get())->getCenterOfMass();
            pegasus::Vector3 const planeNormal = static_cast<pegasus::geometry::Plane*>(body.s.get())->getNormal();
            pegasus::Vector3 const posNormalProjection = planeNormal * (p0 * planeNormal);
            pegasus::Vector3 p1 = p0 + (posNormalProjection - p0) * 2;

            if (p1.x < p0.x) { std::swap(p0.x, p1.x); }

            pegasus::Vector3 b = (planeNormal % p1 % planeNormal).unit() * planeSideLength;
            pegasus::Vector3 c = (planeNormal % b).unit() * planeSideLength;

            std::array<pegasus::Vector3, 4> quadVertices{
                b.inverse() - c + posNormalProjection,
                b - c + posNormalProjection,
                b + c + posNormalProjection,
                b.inverse() + c + posNormalProjection
            };

            glBegin(GL_QUADS);

            if (&*activeObject != &body)
            {
                glColor3f(red, glin, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            for (auto const & v : quadVertices) {
                glVertex3f(v.x, v.y, v.z);
            }
            glEnd();
        }
        else if (s == pegasus::geometry::SimpleShapeType::SPHERE)
        {
            pegasus::geometry::Sphere * sphere = static_cast<pegasus::geometry::Sphere*>(body.s.get());
            double const r = sphere->getRadius();

            glTranslatef(p.x, p.y, p.z);
            glutWireSphere(r + 0.01, 20, 20);

            if (&*activeObject != &body)
            {
                glColor3f(red, glin, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            glutSolidSphere(r, 20, 20);
        }
        else if (s == pegasus::geometry::SimpleShapeType::BOX)
        {            
            pegasus::geometry::Box * box = static_cast<pegasus::geometry::Box*>(body.s.get());
            std::array<pegasus::Vector3, 3> boxAxes;
            box->getAxes(boxAxes.at(0), boxAxes.at(1), boxAxes.at(2));

            glTranslatef(p.x, p.y, p.z);
            glScalef(boxAxes.at(0).magnitude(), boxAxes.at(1).magnitude(), boxAxes.at(2).magnitude());
            glutWireCube(2.01);

            if (&*activeObject != &body)
            {
                glColor3f(red, glin, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }
            glutSolidCube(2);
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
    activeObject->p.addForce(pegasus::Vector3(xAxis * 10.0f, yAxis * 20.0f, zAxis * 10.0f));

    world.runPhysics(0.01);

    for (auto const& body : rigidBodies) {
        body.s->setCenterOfMass(body.p.getPosition());
    }

    Application::update();
}

const char* FallingDemo::getTitle()
{
    return "Pegasus Falling Demo";
}

void FallingDemo::displayText(float x, float y, int r, int g, int b, void* vptr)
{
    std::stringstream ss;
    ss << "0x";
    ss << std::setw(16) << std::setfill('0') << std::hex << reinterpret_cast<uintptr_t>(vptr);

    std::string string (ss.str());

    glColor3f( r, g, b );
    glRasterPos2f( x, y );
    for (auto & chr : string)
    {
        glutBitmapCharacter( GLUT_BITMAP_TIMES_ROMAN_24, chr );
    }
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
    case 'r':
    case 'R':
        activeObject->p.setVelocity(0, 0, 0);
        break;
    case '+':
        zoom -= 0.1;
        break;
    case '-':
        zoom += 0.1;
        break;
    case '8':
        yEye += 10 * zoom;
        break;
    case '2':
        yEye -= 10 * zoom;
        break;
    case '\t':
        ++activeObject;
        if (rigidBodies.end() == activeObject)
        {
            activeObject = rigidBodies.begin();
        }
        break;
    case ']':
        sceneReset();
        break;
    default:
        break;
    }
}

Application* getApplication()
{
    return new FallingDemo();
}
