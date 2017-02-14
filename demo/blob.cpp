#include "Pegas/demo/app.hpp"
#include "Pegas/demo/ogl_headers.hpp"
#include "Pegas/demo/random.hpp"
#include "Pegas/demo/timing.hpp"
#include "Pegas/include/particlecontacts.hpp"
#include "Pegas/include/particleforcegenerator.hpp"
#include "Pegas/include/particleworld.hpp"

#include <cassert>
#include <cmath>
#include <stdio.h>
#include <cstdlib>

#define BLOB_COUNT 5
#define PLATFORM_COUNT 10
#define BLOB_RADIUS 0.4f

namespace pegas {
class Platform : public ParticleContactGenerator {
public:
  Vector3 start;
  Vector3 end;
  std::vector<Particle::Ptr> &particles;

  Platform(Vector3 start, Vector3 end, std::vector<Particle::Ptr> &particles)
      : start(start), end(end), particles(particles) {}

  virtual unsigned int addContact(Contacts &contacts,
                                  unsigned limit) const override {
    const static real restitution = 0.0f;

    unsigned int used = 0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
      if (used >= limit) {
        break;
      }

      Vector3 toParticle = particles[i]->getPosition() - start;
      Vector3 const lineDirection = end - start;
      real const projected = toParticle * lineDirection;
      real const platformSqLength = lineDirection.squareMagnitude();

      if (projected <= 0) {
        if (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS) {
          auto contactNormal = toParticle.unit();
          contactNormal.z = 0;
          auto const penetration = BLOB_RADIUS - toParticle.magnitude();
          contacts.push_back(std::make_shared<ParticleContact>(
              particles[i], nullptr, restitution, contactNormal, penetration));
          ++used;
        }

      } else if (projected >= platformSqLength) {
        toParticle = particles[i]->getPosition() - end;
        if (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS) {
          auto contactNormal = toParticle.unit();
          contactNormal.z = 0;
          auto const penetration = BLOB_RADIUS - toParticle.magnitude();
          contacts.push_back(std::make_shared<ParticleContact>(
              particles[i], nullptr, restitution, contactNormal, penetration));
          ++used;
        }
      } else {
        real distanceToPlatform = toParticle.squareMagnitude() -
                                  projected * projected / platformSqLength;
        if (distanceToPlatform < BLOB_RADIUS * BLOB_RADIUS) {
          Vector3 closestPoint =
              start + lineDirection * (projected / platformSqLength);
          auto contactNormal =
              (particles[i]->getPosition() - closestPoint).unit();
          contactNormal.z = 0;
          auto const penetration = BLOB_RADIUS - std::sqrt(distanceToPlatform);
          contacts.push_back(std::make_shared<ParticleContact>(
              particles[i], nullptr, restitution, contactNormal, penetration));
          ++used;
        }
      }
    }
    return used;
  }
};

class BlobForceGenerator : public ParticleForceGenerator {
public:
  std::vector<Particle::Ptr> &particles;
  real maxReplusion;
  real maxAttraction;
  real minNaturalDistance, maxNaturalDistance;
  real floatHead;
  unsigned int maxFloat;
  real maxDistance;

  BlobForceGenerator(std::vector<Particle::Ptr> &particles)
      : particles(particles) {}

  virtual void updateForce(Particle::Ptr const &particle) {
    unsigned joinCount = 0;
    for (unsigned i = 0; i < particles.size(); ++i) {
      if (particles[i] == particle)
        continue;

      // Work out the separation distance
      Vector3 separation =
          particles[i]->getPosition() - particle->getPosition();
      separation.z = 0.0f;
      real distance = separation.magnitude();

      if (distance < minNaturalDistance) {
        // Use a repulsion force.
        distance = 1.0f - distance / minNaturalDistance;
        particle->addForce(separation.unit() * (1.0f - distance) *
                           maxReplusion * -1.0f);
        joinCount++;
      } else if (distance > maxNaturalDistance && distance < maxDistance) {
        // Use an attraction force.
        distance = (distance - maxNaturalDistance) /
                   (maxDistance - maxNaturalDistance);
        particle->addForce(separation.unit() * distance * maxAttraction);
        joinCount++;
      }
    }

    // If the particle is the head, and we've got a join count, then float it.
    if (particle == particles.front() && joinCount > 0 && maxFloat > 0) {
      real force = real(joinCount / maxFloat) * floatHead;
      if (force > floatHead)
        force = floatHead;
      particle->addForce(Vector3(0, force, 0));
    }
  }
};
} // namespace pegas

class BlobDemo : public Application {
public:
  BlobDemo();

  virtual const char *getTitle() override;

  virtual void display() override;

  virtual void update() override;

  virtual void key(unsigned char key) override;

private:
  std::vector<pegas::Particle::Ptr> blobs;
  std::vector<pegas::Platform::Ptr> platforms;
  pegas::BlobForceGenerator::Ptr blobForceGenerator;
  pegas::ParticleForceRegistry::Ptr forceRegistry;
  pegas::ParticleWorld world;

  float xAxis;
  float yAxis;

  void reset();
};

// Method definitions
BlobDemo::BlobDemo()
    : blobForceGenerator(std::make_shared<pegas::BlobForceGenerator>(blobs)),
      forceRegistry(std::make_shared<pegas::ParticleForceRegistry>()),
      world(PLATFORM_COUNT + BLOB_COUNT, PLATFORM_COUNT), xAxis(0), yAxis(0) {
  pegas::Random r;

  // Create the force generator
  pegas::BlobForceGenerator &blobForce =
      *static_cast<pegas::BlobForceGenerator *>(blobForceGenerator.get());
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
    auto const start =
        pegas::Vector3(pegas::real(i % 2) * 10.0f - 5.0f,
                       pegas::real(i) * 4.0f + ((i % 2) ? 0.0f : 2.0f), 0);

    auto const end =
        pegas::Vector3(pegas::real(i % 2) * 10.0f + 5.0f,
                       pegas::real(i) * 4.0f + ((i % 2) ? 2.0f : 0.0f), 0);

    platforms.push_back(std::make_shared<pegas::Platform>(start, end, blobs));
  }

  pegas::Platform &p =
      *static_cast<pegas::Platform *>(platforms.back().get());
  pegas::real const fraction = pegas::real(1.0) / BLOB_COUNT;
  pegas::Vector3 delta = p.end - p.start;

  for (unsigned int i = 0; i < BLOB_COUNT; ++i) {
    auto blob = std::make_shared<pegas::Particle>();
    auto const me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
    blob->setPosition(p.start +
                      delta * (pegas::real(me) * 0.8f * fraction + 0.1f));

    blob->setVelocity(0, 0, 0);
    blob->setDamping(0.2f);
    blob->setAcceleration(pegas::Vector3(0, -9.8, 0) * 0.4f);
    blob->setMass(1.0f);
    blob->clearForceAccum();
    blobs.push_back(blob);
  }

  for (auto & blob : blobs) {
    forceRegistry->add(blob, blobForceGenerator);
  }

  world.setParticleContactGenerators(platforms);
  world.setParticles(blobs);
  world.setParticleForcesRegistry(forceRegistry);
}

void BlobDemo::reset() {
  auto p = static_cast<pegas::Platform *>(platforms.back().get());
  pegas::real fraction = (pegas::real)1.0 / BLOB_COUNT;
  pegas::Vector3 delta = p->end - p->start;
  for (unsigned i = 0; i < BLOB_COUNT; i++) {
    unsigned me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
    blobs[i]->setPosition(p->start +
                      delta * (pegas::real(me) * 0.8f * fraction + 0.1f));
    blobs[i]->setVelocity(0, 0, 0);
    blobs[i]->clearForceAccum();
  }
}

void BlobDemo::display() {
  pegas::Vector3 pos = blobs[0]->getPosition();

  // Clear the view port and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(pos.x, pos.y, 6.0, pos.x, pos.y, 0.0, 0.0, 1.0, 0.0);

  glColor3f(0, 0, 0);

  glBegin(GL_LINES);
  glColor3f(0, 0, 1);
  for (unsigned i = 0; i < PLATFORM_COUNT; i++) {
    const pegas::Vector3 &p0 =
        static_cast<pegas::Platform *>(platforms[i].get())->start;
    const pegas::Vector3 &p1 =
        static_cast<pegas::Platform *>(platforms[i].get())->end;
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
  }
  glEnd();

  glColor3f(1, 0, 0);
  for (unsigned i = 0; i < BLOB_COUNT; i++) {
    const pegas::Vector3 &p = blobs[i]->getPosition();
    glPushMatrix();
    glTranslatef(p.x, p.y, p.z);
    glutSolidSphere(BLOB_RADIUS, 12, 12);
    glPopMatrix();
  }

  pegas::Vector3 p = blobs[0]->getPosition();
  pegas::Vector3 v = blobs[0]->getVelocity() * 0.05f;
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

void BlobDemo::update() {
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
  blobs[0]->addForce(pegas::Vector3(xAxis, yAxis, 0) * 10.0f);
  for (auto & blob : blobs) {
      blob->addForce(pegas::Vector3(std::rand() % 2, std::rand() % 2 , 0));
  }

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

const char *BlobDemo::getTitle() { return "pegas > Blob Demo"; }

void BlobDemo::key(unsigned char key) {
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

Application *getApplication() { return new BlobDemo(); }
