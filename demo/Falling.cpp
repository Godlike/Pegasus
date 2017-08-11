/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <cstdlib>
#include <cmath>
#include <list>
#include <random>

#include "Pegasus/include/Particle.hpp"
#include "Pegasus/include/Geometry.hpp"
#include "Pegasus/include/Mechanics.hpp"
#include "Pegasus/include/ParticleContacts.hpp"
#include "Pegasus/include/ParticleForceGenerator.hpp"
#include "Pegasus/include/ParticleWorld.hpp"
#include "Pegasus/include/BoundingVolumes.hpp"
#include "Pegasus/include/Math.hpp"
#include "demo/Application.hpp"
#include "demo/OglHeaders.hpp"
#include "demo/Timing.hpp"
#include "demo/Bunny.hpp"

static const uint32_t BOX_COUNT = static_cast<uint32_t>(std::pow(3, 3));
static const uint32_t SPHERE_COUNT = static_cast<uint32_t>(std::pow(3, 3));
static const uint32_t TOTAL_COUNT = BOX_COUNT + SPHERE_COUNT;
static const double RADIUS = 5;
static const bool WIRED_ONLY = true;
static const bool DRAW_CONVEX = true;
static const bool DRAW_BUNNIES = true;

class FallingDemo : public Application
{
public:
    FallingDemo();

    virtual ~FallingDemo() = default;

    void InitGraphics() override;

    const char* GetTitle() override;

    void Display() override;

    void Update() override;

    void Key(unsigned char key) override;

private:
    using Particles = std::list<pegasus::Particle>;
    using RigidBodies = std::list<pegasus::RigidBody>;
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
    std::unique_ptr<pegasus::geometry::volumes::obb::OrientedBoundingBox> orientedBoundingBox;
    std::unique_ptr<pegasus::geometry::volumes::sphere::BoundingSphere> boundingSphere;
    glm::dvec3 aabbTranslate;
    glm::dvec3 obbTranslate;
    glm::dvec3 boundingSphereTranslate;
    GLuint solidBunnyGlListIndex;
    GLuint wiredBunnyGlListIndex;
    GLuint pointsBunnyGlListIndex;
    using ConvexHull = pegasus::math::QuickhullConvexHull<std::vector<glm::dvec3>>;
    std::unique_ptr<ConvexHull> cv;
    ConvexHull::Faces cvFaces;
    std::list<ConvexHull::Vertices::iterator> cvVertices;

    std::array<bool, 4> m_overlap;
    std::array<glm::dvec3, 4> m_contactNormal;
    std::array<double, 4> m_penetration;
    std::array<pegasus::geometry::Ray, 4> m_rays;
    pegasus::geometry::intersection::IntersectionCache<pegasus::geometry::Ray, pegasus::geometry::Plane> m_rayPlaneCache;
    pegasus::geometry::intersection::IntersectionCache<pegasus::geometry::Ray, pegasus::geometry::Sphere> m_raySphereCache;
    pegasus::geometry::intersection::IntersectionCache<pegasus::geometry::Ray, pegasus::geometry::Box> m_rayObbCache;
    pegasus::geometry::intersection::IntersectionCache<pegasus::geometry::Ray, pegasus::geometry::Box> m_rayAabbCache;

    pegasus::geometry::volumes::Vertices vertices;
    pegasus::geometry::volumes::Faces faces;
    pegasus::geometry::volumes::Indices indices;

    void AddCube(glm::dvec3 const& pos, double boxSide);

    void AddBox(glm::dvec3 const& pos, glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k);

    void AddSphere(glm::dvec3 const& pos, double radius);

    void AddPlane(glm::dvec3 const& normal, glm::dvec3 const& point);

    void AddBoundingVolumes();

    void SceneReset();

    void DrawWiredBox(std::array<glm::dvec3, 8> const& boxVertices) const;
};

// Method definitions
FallingDemo::FallingDemo()
    : m_world(m_particles, m_forceRegistry, m_contactGenerators, TOTAL_COUNT * TOTAL_COUNT, 5000)
    , xAxis(0)
    , yAxis(0)
    , zAxis(0)
    , zoom(0.5)
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
            glm::dvec3{boxSide, 0, 0},
            glm::dvec3{0, boxSide, 0},
            glm::dvec3{0, 0, boxSide})
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

void FallingDemo::AddPlane(glm::dvec3 const& normal, glm::dvec3 const& point)
{
    m_particles.emplace_front();
    m_particles.front().SetPosition(point);
    m_particles.front().SetInverseMass(0);
    m_rigidBodies.emplace_front(
        m_particles.front(),
        std::make_unique<pegasus::geometry::Plane>(
            m_particles.front().GetPosition(), glm::normalize(normal)
        )
    );
}

void FallingDemo::AddBoundingVolumes()
{
    pegasus::geometry::Box gjkTestBox{ glm::dvec3{10, -10, 10}, glm::dvec3{ 1, 0, 0 }, glm::dvec3{ 0, 1, 0 }, glm::dvec3{ 0, 0, 1 } };
    glm::dvec3 pointBox = pegasus::geometry::GjkSupport(gjkTestBox, glm::normalize(glm::dvec3{ 1, 0, 1 }));

    pegasus::geometry::Sphere gjkTestSphere{ glm::dvec3{ 10, -10, 10 }, 1 };
    glm::dvec3 pointSphere = pegasus::geometry::GjkSupport(gjkTestSphere, glm::normalize(glm::dvec3{ 1, 0, 1 }));
    double const length = glm::length(pointSphere - gjkTestSphere.centerOfMass);

    using namespace pegasus::geometry::volumes;

    //Bunny Data
    std::transform(bunnyVertices, bunnyVertices + bunnyVerticesSize, std::back_inserter(vertices),
        [](GLfloat v[3]) -> glm::dvec3
    {
        return glm::dvec3(v[0], v[1], v[2]);
    });
    std::transform(bunnyFaceIndicies, bunnyFaceIndicies + bunnyFaceIndiciesSize, std::back_inserter(faces),
        [](short f[6]) -> std::array<size_t, 3>
    {
        return {static_cast<size_t>(f[0]), static_cast<size_t>(f[1]), static_cast<size_t>(f[2])};
    });
    for (size_t i = 0; i < faces.size(); ++i)
        indices.insert(i);

    //OBB
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v)
    {
        v += obbTranslate;
    });
    orientedBoundingBox = std::make_unique<obb::OrientedBoundingBox>(Shape{vertices, faces}, indices);
    auto obb = orientedBoundingBox->GetBox();
    glm::dmat3 obbAxes{ obb.iAxis, obb.jAxis, obb.kAxis };
    AddBox(obb.centerOfMass, obbAxes[0], obbAxes[1], obbAxes[2]);

    //CV
    cv = std::make_unique<ConvexHull>(vertices);
    cv->Calculate();
    cvVertices = cv->GetVertices();
    cvFaces = cv->GetFaces();

    //AABB
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v)
    {
        v += aabbTranslate - obbTranslate;
    });
    axisAlignedBoundingBox = std::make_unique<aabb::AxisAlignedBoundingBox>(Shape{vertices, faces}, indices);
    auto aabb = axisAlignedBoundingBox->GetBox();
    glm::dmat3 aabbAxes{ aabb.iAxis, aabb.jAxis, aabb.kAxis };
    AddBox(aabb.centerOfMass, aabbAxes[0], aabbAxes[1], aabbAxes[2]);

    //BS
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v)
    {
        v += boundingSphereTranslate - aabbTranslate;
    });
    boundingSphere = std::make_unique<sphere::BoundingSphere>(Shape{vertices, faces}, indices);
    auto sphere = boundingSphere->GetSphere();
    AddSphere(sphere.centerOfMass, sphere.radius);
    std::for_each(vertices.begin(), vertices.end(), [&](auto& v)
    {
        v -= boundingSphereTranslate;
    });

    m_rays = {
        pegasus::geometry::Ray{ sphere.centerOfMass + glm::dvec3{ 1, 0, 0 }, glm::normalize(glm::dvec3{ -1, 0.5, 0}) },
        pegasus::geometry::Ray{ aabb.centerOfMass + glm::dvec3{ 1, 0, 0 }, glm::normalize(glm::dvec3{ -1, 0.5, 0 }) },
        pegasus::geometry::Ray{ obb.centerOfMass + glm::dvec3{ 1, 0, 0 }, glm::normalize(glm::dvec3{ -1, 0.5, 0 }) },
    };

    using namespace pegasus::geometry::intersection;
    
    Initialize<pegasus::geometry::Ray, pegasus::geometry::Sphere>(&m_rays[0], &sphere, &m_raySphereCache);
    m_overlap[0] = CalculateIntersection<pegasus::geometry::Ray, pegasus::geometry::Sphere>(&m_rays[0], &sphere, &m_raySphereCache);
    m_contactNormal[0] = CalculateContactNormal<pegasus::geometry::Ray, pegasus::geometry::Sphere>(&m_rays[0], &sphere, &m_raySphereCache);
    m_penetration[0] = CalculatePenetration<pegasus::geometry::Ray, pegasus::geometry::Sphere>(&m_rays[0], &sphere, &m_raySphereCache);

    Initialize<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[1], &aabb, &m_rayAabbCache);
    m_overlap[1] = CalculateIntersection<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[1], &aabb, &m_rayAabbCache);
    m_contactNormal[1] = CalculateContactNormal<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[1], &aabb, &m_rayAabbCache);
    m_penetration[1] = CalculatePenetration<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[1], &aabb, &m_rayAabbCache);

    Initialize<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[2], &obb, &m_rayObbCache);
    m_overlap[2] = CalculateIntersection<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[2], &obb, &m_rayObbCache);
    m_contactNormal[2] = CalculateContactNormal<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[2], &obb, &m_rayObbCache);
    m_penetration[2] = CalculatePenetration<pegasus::geometry::Ray, pegasus::geometry::Box>(&m_rays[2], &obb, &m_rayObbCache);
}

void FallingDemo::SceneReset()
{
    m_rigidBodies.clear();
    m_forces.clear();
    m_forceRegistry.Clear();
    m_contactGenerators.clear();
    m_particles.clear();

    double const boxSide = 15.0;
    double const position = boxSide;

    static auto randDouble = []()
    {
        static std::default_random_engine generator;
        static std::uniform_real_distribution<double> distribution(-5.0, 5.0);
        return distribution(generator);
    };

    //Create particles
    for (uint32_t i = 0; i < TOTAL_COUNT; ++i)
    {
        static auto curt = [](auto n)
        {
            return std::pow(n, 0.34);
        };
        static const int edge = curt(TOTAL_COUNT);
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
                    glm::dvec3{RADIUS, 0, 0} * 0.5,
                    glm::dvec3{0, RADIUS, 0} * 0.5,
                    glm::dvec3{0, 0, RADIUS} * 0.5)
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

    //Create forces
    m_forces.push_back(std::make_unique<pegasus::ParticleGravity>(glm::dvec3{0, -9.8, 0}));

    //Register forces
    for (auto& particle : m_particles)
    {
        m_forceRegistry.Add(particle, *m_forces.front());
    }

    //Create contact generators
    for (auto& body : m_rigidBodies)
    {
        m_contactGenerators.push_back(
            std::make_unique<pegasus::ShapeContactGenerator<RigidBodies>>(body, m_rigidBodies, (randDouble() + 5) / 10)
        );
    }

    //Create plane particle and rigid body
    AddPlane({0, 1, 0}, {1, -position * 2, 0});
    AddSphere({0, -position * 2, 0}, boxSide);
    AddCube({position * 2, position, 0}, boxSide);
    AddCube({-position * 2, position, 0}, boxSide);
    AddCube({0, position, -position * 2}, boxSide);
    AddBoundingVolumes();

    activeObject = m_rigidBodies.end();
    std::advance(activeObject, -3);
}

void FallingDemo::DrawWiredBox(std::array<glm::dvec3, 8> const& boxVertices) const
{
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

void FallingDemo::Display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(255 / 255.0, 127 / 255.0, 80 / 255.0, 1.0);
    glLoadIdentity();

    auto const& pos = activeObject->p.GetPosition();

    glm::dvec3 from{ pos.x, (pos.y + 30 + yEye), (pos.z + 30) };
    glm::dvec3 to{ pos.x, pos.y, pos.z };
    glm::dvec3 viewVec = from - to;
    viewVec *= zoom;
    from = viewVec + to;

    gluLookAt(from.x, from.y, from.z, to.x, to.y, to.z, 0.0, 1.0, 0.0);
    glPointSize(5);

    //Draw rays
    for (uint32_t i = 0; i < 3; ++i)
    {
        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);

        //Original ray
        {
            if (m_overlap[i]) {
                glColor3f(1.0, 0.0, 0.0);
            }
            else {
                glColor3d(0.0, 1.0, 0.0);
            }
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(m_rays[i].centerOfMass));
            glm::dvec3 rayEnd = m_rays[i].centerOfMass;
            rayEnd += m_rays[i].direction * 2.5;
            glVertex3dv(glm::value_ptr(rayEnd));
            glEnd();
        }

        glPopMatrix();
    }

    {
        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3d(0.0, 1.0, 0.0);

        //Sphere intersection points
        {
            glBegin(GL_POINTS);
            glVertex3dv(glm::value_ptr(m_raySphereCache.inPoint));
            glVertex3dv(glm::value_ptr(m_raySphereCache.outPoint));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(m_raySphereCache.inPoint));
            glVertex3dv(glm::value_ptr(m_raySphereCache.inPoint + m_raySphereCache.sphereContactNormal));
            glEnd();
        }

        //Aabb intersection points
        {
            glBegin(GL_POINTS);
            glVertex3dv(glm::value_ptr(m_rayAabbCache.inPoint));
            glVertex3dv(glm::value_ptr(m_rayAabbCache.outPoint));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(m_rayAabbCache.inPoint));
            glVertex3dv(glm::value_ptr(m_rayAabbCache.inPoint + m_rayAabbCache.boxContactNormal));
            glEnd();
        }

        //Obb intersection points
        {
            glBegin(GL_POINTS);
            glVertex3dv(glm::value_ptr(m_rayObbCache.inPoint));
            glVertex3dv(glm::value_ptr(m_rayObbCache.outPoint));
            glEnd();
            glBegin(GL_LINES);
            glVertex3dv(glm::value_ptr(m_rayObbCache.inPoint));
            glVertex3dv(glm::value_ptr(m_rayObbCache.inPoint + m_rayObbCache.boxContactNormal));
            glEnd();
        }

        glPopMatrix();
    }

    if (DRAW_CONVEX)
    {
        uint32_t index = 0;

        //Draw vertices
        for (auto vertex : cvVertices)
        {
            glPushMatrix();
            glRotated(yRotationAngle, 0, 1, 0);
            glBegin(GL_POINTS);
            glColor3f(1, 0, 0);
            glVertex3dv(glm::value_ptr(*vertex));
            glEnd();
            glPopMatrix();
        }

        //Draw faces and face normals
        for (auto face = cvFaces.begin(); face != cvFaces.end(); ++face)
        {
            uint32_t const kekdex = ++index;
            double red = static_cast<double>(0xb00b1e55 % kekdex) / static_cast<double>(kekdex);
            double green = static_cast<double>(0x31337420 % kekdex) / static_cast<double>(kekdex);
            double blue = static_cast<double>(0xdeadbeef % kekdex) / static_cast<double>(kekdex);

            auto faceIndices = face->GetIndices();

            glm::dvec3 const faceCenter =
                (vertices[faceIndices[0]] + vertices[faceIndices[1]] + vertices[faceIndices[2]]) * (1.0 / 3.0);

            glPushMatrix();
            glRotated(yRotationAngle, 0, 1, 0);
            glBegin(GL_TRIANGLES);
            glColor3f(red, green, blue);
            glVertex3dv(glm::value_ptr(vertices[faceIndices[0]]));
            glVertex3dv(glm::value_ptr(vertices[faceIndices[1]]));
            glVertex3dv(glm::value_ptr(vertices[faceIndices[2]]));
            glEnd();

            glBegin(GL_LINES);
            glColor3f(0, 0, 1);
            glVertex3dv(glm::value_ptr(glm::dvec3{0, 0, 0} + faceCenter));
            glColor3f(0, 1, 0);
            glVertex3dv(glm::value_ptr(face->GetHyperPlane().GetNormal() * 0.1 + faceCenter));
            glEnd();
            glPopMatrix();
        }
    }

    if (DRAW_BUNNIES)
    {
        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(obbTranslate.x, obbTranslate.y, obbTranslate.z);
        glCallList(pointsBunnyGlListIndex);
        glPopMatrix();

        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(aabbTranslate.x, aabbTranslate.y, aabbTranslate.z);
        glCallList(pointsBunnyGlListIndex);
        glPopMatrix();

        glPushMatrix();
        glRotated(yRotationAngle, 0, 1, 0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(boundingSphereTranslate.x, boundingSphereTranslate.y, boundingSphereTranslate.z);
        glCallList(pointsBunnyGlListIndex);
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
        double red = static_cast<double>(0xb00b1e55 % kekdex) / static_cast<double>(kekdex);
        double green = static_cast<double>(0x31337420 % kekdex) / static_cast<double>(kekdex);
        double blue = static_cast<double>(0xdeadbeef % kekdex) / static_cast<double>(kekdex);

        if (s == pegasus::geometry::SimpleShape::Type::PLANE)
        {
            double const planeSideLength = 100;

            glm::dvec3 p0 = static_cast<pegasus::geometry::Plane*>(body.s.get())->centerOfMass;
            glm::dvec3 const planeNormal = static_cast<pegasus::geometry::Plane*>(body.s.get())->normal;
            glm::dvec3 const posNormalProjection = planeNormal * glm::dot(p0, planeNormal);
            glm::dvec3 p1 = p0 + (posNormalProjection - p0) * 2.0;

            if (p1.x < p0.x)
            {
                std::swap(p0.x, p1.x);
            }

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

            for (auto const& v : quadVertices)
            {
                glVertex3f(v.x, v.y, v.z);
            }
            glEnd();
        }
        else if (s == pegasus::geometry::SimpleShape::Type::SPHERE)
        {
            pegasus::geometry::Sphere* sphere = static_cast<pegasus::geometry::Sphere*>(body.s.get());
            double const r = sphere->radius;
            glTranslatef(p.x, p.y, p.z);

            if (&*activeObject != &body)
            {
                glColor3f(red, green, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            if (!WIRED_ONLY)
            {
                glutSolidSphere(r, 20, 20);
            }

            if (&*activeObject != &body && !WIRED_ONLY)
            {
                glColor3f(1.0f, 0.0, 0.0);
            }
            glutWireSphere(r + 0.001, 20, 20);
        }
        else if (s == pegasus::geometry::SimpleShape::Type::BOX)
        {
            pegasus::geometry::Box* box = static_cast<pegasus::geometry::Box*>(body.s.get());
            std::array<glm::dvec3, 3> boxAxes = {
                box->iAxis, box->jAxis, box->kAxis
            };

            glTranslatef(p.x, p.y, p.z);
            glm::dvec3 const& i = boxAxes[0];
            glm::dvec3 const& j = boxAxes[1];
            glm::dvec3 const& k = boxAxes[2];
            std::array<glm::dvec3, 8> boxVertices = {
                i + j + k, i - j + k, -i + j + k, -i - j + k,
                i + j - k, i - j - k, -i + j - k, -i - j - k
            };
            if (&*activeObject != &body)
            {
                glColor3f(red, green, blue);
            }
            else
            {
                glColor3f(0.18f, 0.31f, 0.31f);
            }

            //Draw solid Cube
            if (!WIRED_ONLY)
            {
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
            if (&*activeObject != &body && !WIRED_ONLY)
            {
                glColor3f(1.0f, 0.0, 0.0);
            }
            DrawWiredBox(boxVertices);
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

    for (auto const& body : m_rigidBodies)
    {
        body.s->centerOfMass = body.p.GetPosition();
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

    glNewList(pointsBunnyGlListIndex, GL_COMPILE);
    GeneratePointStanfordBunny();
    glEndList();
}

const char* FallingDemo::GetTitle()
{
    return "Pegasus Falling Demo";
}

void FallingDemo::Key(unsigned char key)
{
    switch (key)
    {
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
        zoom -= zoom * 0.1;
        break;
    case '-':
        zoom += zoom * 0.1;
        break;
    case '8':
        yEye += 10 * zoom;
        break;
    case '2':
        yEye -= 10 * zoom;
        break;
    case '4':
        yRotationAngle += 3.14 / 90 * 36;
        break;
    case '6':
        yRotationAngle -= 3.14 / 90 * 36;
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
