/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "demo/Application.hpp"
#include "demo/OglHeaders.hpp"
#include "demo/Timing.hpp"
#include "demo/Bunny.hpp"
#include "Pegasus/include/Particle.hpp"
#include "Pegasus/include/Geometry.hpp"
#include "Pegasus/include/Mechanics.hpp"
#include "Pegasus/include/ParticleContacts.hpp"
#include "Pegasus/include/ParticleForceGenerator.hpp"
#include "Pegasus/include/ParticleWorld.hpp"
#include "Pegasus/include/BoundingVolumes.hpp"

#include <glm/ext.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <list>
#include <random>
#include <utility>

static const uint32_t BOX_COUNT    = static_cast<uint32_t>(std::pow(3, 3));
static const uint32_t SPHERE_COUNT = static_cast<uint32_t>(std::pow(3, 3));
static const uint32_t TOTAL_COUNT  = BOX_COUNT+SPHERE_COUNT;
static const double   RADIUS       = 5;
static const bool     WIRED_ONLY   = true;

class FallingDemo : public Application {
public:
    FallingDemo();
    virtual ~FallingDemo() {}

    void InitGraphics() override;
    const char* GetTitle() override;
    void Display() override;
    void Update() override;
    void Key(unsigned char key) override;

private:
    using Particles       = std::list<pegasus::Particle>;
    using RigidBodies     = std::list<pegasus::RigidBody>;
    using ForceGenerators = std::list<std::unique_ptr<pegasus::ParticleForceGenerator>>;

    Particles m_particles;
    RigidBodies m_rigidBodies;
    ForceGenerators m_forces;
    pegasus::ParticleForceRegistry m_forceRegistry;
    pegasus::ParticleContactGenerators m_contactGenerators;
    pegasus::ParticleWorld m_world;

    double xAxis;
    double yAxis;
    double zAxis;
    double zoom;
    double yEye;
    double yRotationAngle;
    RigidBodies::iterator activeObject;

    std::unique_ptr<pegasus::geometry::volumes::aabb::AxisAlignedBoundingBox> axisAlignedBoundingBox;
    std::unique_ptr<pegasus::geometry::volumes::obb::OrientedBoundingBox>     orientedBoundingBox;
    std::unique_ptr<pegasus::geometry::volumes::sphere::BoundingSphere>       boundingSphere;
    glm::dvec3 aabbTranslate;
    glm::dvec3 obbTranslate;
    glm::dvec3 boundingSphereTranslate;
    GLuint solidBunnyGlListIndex;
    GLuint wiredBunnyGlListIndex;

    void AddCube(glm::dvec3 const& pos, double boxSide);
    void AddBox(glm::dvec3 const& pos, glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k);
    void AddSphere(glm::dvec3 const& pos, double radius);
    void AddBoundingVolumes();
    void SceneReset();
};

// Method definitions
FallingDemo::FallingDemo()
    : m_world(m_particles, m_forceRegistry, m_contactGenerators, TOTAL_COUNT * TOTAL_COUNT, 5000)
    , xAxis(0)
    , yAxis(0)
    , zAxis(0)
    , zoom(5)
    , yEye(0)
    , yRotationAngle(0)
    , activeObject(m_rigidBodies.begin())
    , aabbTranslate(10, 0, 0)
    , obbTranslate(0, 0, 0)
    , boundingSphereTranslate(0, 0, 10)
{
    SceneReset();
}

void FallingDemo::AddCube(glm::dvec3 const& pos, double boxSide)
{
    m_particles.emplace_back();
    m_particles.back().SetPosition(pos);
    m_particles.back().SetInverseMass(0);
    m_rigidBodies.emplace_back(
        m_particles.back(),
        std::make_unique<pegasus::geometry::Box>(
            pos,
            glm::dvec3{ boxSide, 0, 0 },
            glm::dvec3{ 0, boxSide, 0 },
            glm::dvec3{ 0, 0, boxSide })
    );
}

void FallingDemo::AddBox(glm::dvec3 const& pos, glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k)
{
    m_particles.emplace_back();
    m_particles.back().SetPosition(pos);
    m_particles.back().SetInverseMass(0);
    m_rigidBodies.emplace_back(
        m_particles.back(),
        std::make_unique<pegasus::geometry::Box>(pos, i, j, k)
    );
}

void FallingDemo::AddSphere(glm::dvec3 const& pos, double radius)
{
    m_particles.emplace_back();
    m_particles.back().SetPosition(pos);
    m_particles.back().SetInverseMass(0);
    m_rigidBodies.emplace_back(
        m_particles.back(),
        std::make_unique<pegasus::geometry::Sphere>(pos, radius)
    );
}

void FallingDemo::AddBoundingVolumes()
{
    using namespace pegasus::geometry::volumes;

    //Bunny Data
    Vertices vertices;
    std::transform(bunnyVertices, bunnyVertices+kCount, std::back_inserter(vertices),
                   [](GLfloat v[3]) -> glm::dvec3{ return glm::dvec3(v[0], v[1], v[2]); });
    Faces faces;
    std::transform(bunnyFaceIndicies, bunnyFaceIndicies+kCount, std::back_inserter(faces),
        [](short f[6]) -> std::array<size_t, 3> { 
            return {static_cast<size_t>(f[0]), static_cast<size_t>(f[1]), static_cast<size_t>(f[2])}; 
    });
    Indices indices;
    for (size_t i = 0; i < faces.size(); ++i) indices.insert(i);
 
    //OBB
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v) {v += obbTranslate; });
    orientedBoundingBox = std::make_unique<obb::OrientedBoundingBox>(Shape{vertices, faces}, indices);
    auto obb = orientedBoundingBox->GetBox();
    glm::dmat3 obbAxes;
    obb.GetAxes(obbAxes[0], obbAxes[1], obbAxes[2]);
    AddBox(obb.getCenterOfMass(), obbAxes[0], obbAxes[1], obbAxes[2]);

    //AABB
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v) {v += aabbTranslate - obbTranslate; });
    axisAlignedBoundingBox = std::make_unique<aabb::AxisAlignedBoundingBox>(Shape{vertices, faces}, indices);
    auto aabb = axisAlignedBoundingBox->GetBox();
    glm::dmat3 aabbAxes;
    aabb.GetAxes(aabbAxes[0], aabbAxes[1], aabbAxes[2]);
    AddBox(aabb.getCenterOfMass(), aabbAxes[0], aabbAxes[1], aabbAxes[2]); 

    //BS
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v) {v += boundingSphereTranslate - aabbTranslate; });
    boundingSphere = std::make_unique<sphere::BoundingSphere>(Shape{vertices, faces}, indices);
    auto sphere = boundingSphere->GetSphere();
    AddSphere(sphere.getCenterOfMass(), sphere.GetRadius());
}

void FallingDemo::SceneReset()
{
    m_rigidBodies.clear();
    m_forces.clear();
    m_forceRegistry.Clear();
    m_contactGenerators.clear();
    m_particles.clear();

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

        m_particles.emplace_back();
        m_particles.back().SetPosition(
            row * RADIUS * 2.3 + RADIUS - offset,
            planeIndex * RADIUS * 2.3 + boxSide * 3,
            col * RADIUS * 2.3 + RADIUS - offset
        );
        m_particles.back().SetVelocity(randDouble(), randDouble() - 5, randDouble());
        m_particles.back().SetDamping(1.0f);
        m_particles.back().SetMass(1e6);
    }

    //Create rigid bodies
    for (auto& particle : m_particles)
    {
        bool const isBox = randDouble() > 0;

        if (isBox)
        {
            m_rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Box>(
                    particle.GetPosition(),
                    glm::dvec3{ RADIUS, 0, 0 } * 0.5,
                    glm::dvec3{ 0, RADIUS, 0 } * 0.5,
                    glm::dvec3{ 0, 0, RADIUS } * 0.5)
            );
        }
        else
        {
            m_rigidBodies.emplace_back(
                particle,
                std::make_unique<pegasus::geometry::Sphere>(particle.GetPosition(), double(randDouble() + 6) / 2)
            );
        }
    }

    for (auto& particle : m_particles)
    {
        m_forces.push_back(std::make_unique<pegasus::ParticleGravity>(particle));

        for(auto& p : m_particles)
        {
            if (&p != &particle)
                m_forceRegistry.Add(p, *m_forces.back());
        }
    }

    //Create forces
    //m_forces.push_back(std::make_unique<pegasus::ParticleStaticField>(glm::dvec3{ 0, -9.8, 0 }));
    
    //Register forces
    //for (auto& particle : m_particles)
    //{
    //    m_forceRegistry.Add(particle, *m_forces.front());
    //}

    //Create contact generators
    for (auto& body : m_rigidBodies)
    {
        m_contactGenerators.push_back(
            std::make_unique<pegasus::ShapeContactGenerator<RigidBodies>>(body, m_rigidBodies, (randDouble() + 5) / 10));
    }

    //Create plane particle and rigid body
    /*m_particles.emplace_front();
    m_particles.front().SetPosition({1, -position * 2, 0});
    m_particles.front().SetInverseMass(0);
    m_rigidBodies.emplace_front(
        m_particles.front(),
        std::make_unique<pegasus::geometry::Plane>(
           m_particles.front().GetPosition(), glm::normalize(glm::dvec3(0, 1.0, 0))
        )
    );

    AddSphere({  0, -position * 2, 0 }, boxSide);
    AddCube({  position * 2, position, 0 }, boxSide);
    AddCube({ -position * 2, position, 0 }, boxSide);
    AddCube({ 0, position, -position * 2 }, boxSide);
    AddBoundingVolumes();*/

    activeObject = m_rigidBodies.end();
    std::advance(activeObject, -1);
}

void FallingDemo::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    auto const& pos = activeObject->p.GetPosition();
    gluLookAt(pos.x * zoom , pos.y + 30 * zoom + yEye, pos.z + 30 * zoom , pos.x, pos.y, pos.z, 0.0, 1.0, 0.0);

    {
        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(obbTranslate.x, obbTranslate.y, obbTranslate.z);
        glCallList(wiredBunnyGlListIndex);
        glPopMatrix();

        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(aabbTranslate.x, aabbTranslate.y, aabbTranslate.z);
        glCallList(wiredBunnyGlListIndex);
        glPopMatrix();

        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(boundingSphereTranslate.x, boundingSphereTranslate.y, boundingSphereTranslate.z); 
        glCallList(wiredBunnyGlListIndex);
        glPopMatrix();
    }

    //Add bodies
    for (auto& body : m_rigidBodies)
    {
        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);

        static int index = -1;
        index += 1;
        index %= m_rigidBodies.size();

        auto const& p = body.p.GetPosition();
        auto const& s = body.s->type;

        int const kekdex = index + 1;
        double red   = static_cast<double>(0xb00b1e55 % kekdex) / static_cast<double>(kekdex);
        double green = static_cast<double>(0x31337420 % kekdex) / static_cast<double>(kekdex);
        double blue  = static_cast<double>(0xdeadbeef % kekdex) / static_cast<double>(kekdex);

        if (s == pegasus::geometry::SimpleShapeType::PLANE) 
        {
            glTranslatef(0, 0, 0);
            double const planeSideLength = 100;

            glm::dvec3 p0 = static_cast<pegasus::geometry::Plane*>(body.s.get())->getCenterOfMass();
            glm::dvec3 const planeNormal = static_cast<pegasus::geometry::Plane*>(body.s.get())->GetNormal();
            glm::dvec3 const posNormalProjection = planeNormal * (p0 * planeNormal);
            glm::dvec3 p1 = p0 + (posNormalProjection - p0) * 2.0;

            if (p1.x < p0.x) { std::swap(p0.x, p1.x); }

            glm::dvec3 b = glm::normalize(glm::cross(glm::cross(planeNormal, p1), planeNormal)) * planeSideLength;
            glm::dvec3 c = glm::normalize(glm::cross(planeNormal, b)) * planeSideLength;

            std::array<glm::dvec3, 4> quadVertices{
                b * -1.0 - c + posNormalProjection,
                b - c + posNormalProjection,
                b + c + posNormalProjection,
                b * -1.0 + c + posNormalProjection
            };

            glBegin(GL_QUADS);
            if (&*activeObject != &body)
            {
                glColor3d(red, green, blue);
            }
            else
            {
                glColor3d(0.18, 0.31, 0.31);
            }

            for (auto const& v : quadVertices) {
                glVertex3f(v.x, v.y, v.z);
            }
            glEnd();
        }
        else if (s == pegasus::geometry::SimpleShapeType::SPHERE)
        {
            pegasus::geometry::Sphere * sphere = static_cast<pegasus::geometry::Sphere*>(body.s.get());
            double const r = sphere->GetRadius();
            glTranslatef(p.x, p.y, p.z);

            if (&*activeObject != &body)
            {
                glColor3f(red, green, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            if (!WIRED_ONLY) {
                glutSolidSphere(r, 20, 20);
            } 

            if (&*activeObject != &body && !WIRED_ONLY) {
                glColor3f(1.0f, 0.0, 0.0);
            }
            glutWireSphere(r + 0.001, 20, 20);
        }
        else if (s == pegasus::geometry::SimpleShapeType::BOX)
        {            
            pegasus::geometry::Box * box = static_cast<pegasus::geometry::Box*>(body.s.get());
            std::array<glm::dvec3, 3> boxAxes;
            box->GetAxes(boxAxes[0], boxAxes[1], boxAxes[2]);

            glTranslatef(p.x, p.y, p.z);
            glm::dvec3 const& i = boxAxes[0];
            glm::dvec3 const& j = boxAxes[1];
            glm::dvec3 const& k = boxAxes[2];
            std::array<glm::dvec3, 8> boxVertices = { i + j + k, i - j + k, -i + j + k, -i - j + k,
                                                      i + j - k, i - j - k, -i + j - k, -i - j - k };
            if (&*activeObject != &body)
            {
                glColor3f(red, green, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            //Draw solid Cube
            if (!WIRED_ONLY) {
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[0]));
                glVertex3dv(glm::value_ptr(boxVertices[1]));
                glVertex3dv(glm::value_ptr(boxVertices[3]));
                glVertex3dv(glm::value_ptr(boxVertices[2]));
                glEnd();
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[4]));
                glVertex3dv(glm::value_ptr(boxVertices[5]));
                glVertex3dv(glm::value_ptr(boxVertices[7]));
                glVertex3dv(glm::value_ptr(boxVertices[6]));
                glEnd();
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[0]));
                glVertex3dv(glm::value_ptr(boxVertices[1]));
                glVertex3dv(glm::value_ptr(boxVertices[5]));
                glVertex3dv(glm::value_ptr(boxVertices[4]));
                glEnd();
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[2]));
                glVertex3dv(glm::value_ptr(boxVertices[3]));
                glVertex3dv(glm::value_ptr(boxVertices[7]));
                glVertex3dv(glm::value_ptr(boxVertices[6]));
                glEnd();
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[0]));
                glVertex3dv(glm::value_ptr(boxVertices[2]));
                glVertex3dv(glm::value_ptr(boxVertices[6]));
                glVertex3dv(glm::value_ptr(boxVertices[4]));
                glEnd();
                glBegin(GL_QUADS);
                glVertex3dv(glm::value_ptr(boxVertices[1]));
                glVertex3dv(glm::value_ptr(boxVertices[3]));
                glVertex3dv(glm::value_ptr(boxVertices[7]));
                glVertex3dv(glm::value_ptr(boxVertices[5]));
                glEnd();
            }

            //Draw wired Cube
            if (&*activeObject != &body && !WIRED_ONLY) {
                glColor3f(1.0f, 0.0, 0.0);
            }
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(boxVertices[0]));
            glVertex3dv(glm::value_ptr(boxVertices[1]));
            glVertex3dv(glm::value_ptr(boxVertices[1]));
            glVertex3dv(glm::value_ptr(boxVertices[3]));
            glVertex3dv(glm::value_ptr(boxVertices[3]));
            glVertex3dv(glm::value_ptr(boxVertices[2]));
            glVertex3dv(glm::value_ptr(boxVertices[2]));
            glVertex3dv(glm::value_ptr(boxVertices[0]));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(boxVertices[4]));
            glVertex3dv(glm::value_ptr(boxVertices[5]));
            glVertex3dv(glm::value_ptr(boxVertices[5]));
            glVertex3dv(glm::value_ptr(boxVertices[7]));
            glVertex3dv(glm::value_ptr(boxVertices[7]));
            glVertex3dv(glm::value_ptr(boxVertices[6]));
            glVertex3dv(glm::value_ptr(boxVertices[6]));
            glVertex3dv(glm::value_ptr(boxVertices[4]));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(boxVertices[0]));
            glVertex3dv(glm::value_ptr(boxVertices[1]));
            glVertex3dv(glm::value_ptr(boxVertices[1]));
            glVertex3dv(glm::value_ptr(boxVertices[5]));
            glVertex3dv(glm::value_ptr(boxVertices[5]));
            glVertex3dv(glm::value_ptr(boxVertices[4]));
            glVertex3dv(glm::value_ptr(boxVertices[4]));
            glVertex3dv(glm::value_ptr(boxVertices[0]));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(boxVertices[2]));
            glVertex3dv(glm::value_ptr(boxVertices[3]));
            glVertex3dv(glm::value_ptr(boxVertices[3]));
            glVertex3dv(glm::value_ptr(boxVertices[7]));
            glVertex3dv(glm::value_ptr(boxVertices[7]));
            glVertex3dv(glm::value_ptr(boxVertices[6]));
            glVertex3dv(glm::value_ptr(boxVertices[6]));
            glVertex3dv(glm::value_ptr(boxVertices[2]));
            glEnd();
        }
        glPopMatrix();
    }
}

void FallingDemo::Update()
{
    m_world.StartFrame();

    double duration = TimingData::Get().lastFrameDuration * 0.001;
    if (duration <= 0.0)
        return;

    xAxis *= pow(0.1, duration);
    yAxis *= pow(0.1, duration);
    zAxis *= pow(0.1, duration);
    activeObject->p.AddForce(glm::dvec3(xAxis * 10.0, yAxis * 20.0, zAxis * 10.0));

    m_world.RunPhysics(0.01);

    for (auto const& body : m_rigidBodies) {
        body.s->setCenterOfMass(body.p.GetPosition());
    }

    Application::Update();
}

void FallingDemo::InitGraphics()
{
    glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    SetView();

    solidBunnyGlListIndex = glGenLists(2);
    wiredBunnyGlListIndex = solidBunnyGlListIndex + 1;

    glNewList(solidBunnyGlListIndex, GL_COMPILE);
    GenerateSolidStanfordBunny();
    glEndList();

    glNewList(wiredBunnyGlListIndex, GL_COMPILE);
    GenerateWireFrameStanfordBunny();
    glEndList();
}

const char* FallingDemo::GetTitle()
{
    return "Pegasus Falling Demo";
}

void FallingDemo::Key(unsigned char key)
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
        activeObject->p.SetVelocity(0, 0, 0);
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
    case '4':
        yRotationAngle += 3.14 / 90 * 10;
        break;
    case '6':
        yRotationAngle -= 3.14 / 90 * 10;
        break;
    case '\t':
        ++activeObject;
        if (m_rigidBodies.end() == activeObject)
        {
            activeObject = m_rigidBodies.begin();
        }
        break;
    case ']':
        SceneReset();
        break;
    default:
        break;
    }
}

Application* GetApplication()
{
    return new FallingDemo();
}
