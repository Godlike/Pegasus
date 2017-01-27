#include "Pegas/include/core.hpp"
#include "Pegas/include/particle.hpp"
#include "Pegas/demo/ogl_headers.hpp"
#include "Pegas/demo/app.hpp"
#include "Pegas/demo/timing.hpp"

#include <stdio.h>

class BallisticDemo : public Application
{
    enum ShotType
    {
        UNUSED = 0,
        PISTOL,
        ARTILLERY,
        FIREBALL,
        LASER
    };

    struct AmmoRound
    {
        pegas::Particle particle;
        ShotType type;
        unsigned startTime;

        void render()
        {
            pegas::Vector3 position;
            position = particle.getPosition();

            glColor3f(0, 0, 0);
            glPushMatrix();
            glTranslatef(position.x, position.y, position.z);
            glutSolidSphere(0.3f, 5, 4);
            glPopMatrix();

            glColor3f(0.75, 0.75, 0.75);
            glPushMatrix();
            glTranslatef(position.x, 0, position.z);
            glScalef(1.0f, 0.1f, 1.0f);
            glutSolidSphere(0.6f, 5, 4);
            glPopMatrix();
        }
    };

    const static unsigned ammoRounds = 16;

    AmmoRound ammo[ammoRounds];

    ShotType currentShotType;

    void fire();

public:
    BallisticDemo();

    virtual const char* getTitle();

    virtual void update();

    virtual void display();

    virtual void mouse(int button, int state, int x, int y);

    virtual void key(unsigned char key);
};

// Method definitions
BallisticDemo::BallisticDemo()
: currentShotType(LASER)
{
    // Make all shots unused
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        shot->type = UNUSED;
    }
}

const char* BallisticDemo::getTitle()
{
    return "pegas > Ballistic Demo";
}

void BallisticDemo::fire()
{
    // Find the first available round.
    AmmoRound *shot;
    for (shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type == UNUSED) break;
    }

    // If we didn't find a round, then exit - we can't fire.
    if (shot >= ammo+ammoRounds) return;

    // Set the properties of the particle
    switch(currentShotType)
    {
    case PISTOL:
        shot->particle.setMass(2.0f); // 2.0kg
        shot->particle.setVelocity(0.0f, 0.0f, 35.0f); // 35m/s
        shot->particle.setAcceleration(0.0f, -1.0f, 0.0f);
        shot->particle.setDamping(0.99f);
        break;

    case ARTILLERY:
        shot->particle.setMass(200.0f); // 200.0kg
        shot->particle.setVelocity(0.0f, 30.0f, 40.0f); // 50m/s
        shot->particle.setAcceleration(0.0f, -20.0f, 0.0f);
        shot->particle.setDamping(0.99f);
        break;

    case FIREBALL:
        shot->particle.setMass(1.0f); // 1.0kg - mostly blast damage
        shot->particle.setVelocity(0.0f, 0.0f, 10.0f); // 5m/s
        shot->particle.setAcceleration(0.0f, 0.6f, 0.0f); // Floats up
        shot->particle.setDamping(0.9f);
        break;

    case LASER:
        // Note that this is the kind of laser bolt seen in films,
        // not a realistic laser beam!
        shot->particle.setMass(0.1f); // 0.1kg - almost no weight
        shot->particle.setVelocity(0.0f, 0.0f, 100.0f); // 100m/s
        shot->particle.setAcceleration(0.0f, 0.0f, 0.0f); // No gravity
        shot->particle.setDamping(0.99f);
        break;
    }

    // Set the data common to all particle types
    shot->particle.setPosition(0.0f, 1.5f, 0.0f);
    shot->startTime = TimingData::get().lastFrameTimestamp;
    shot->type = currentShotType;

    // Clear the force accumulators
    shot->particle.clearForceAccum();
}

void BallisticDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Update the physics of each particle in turn
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            // Run the physics
            shot->particle.integrate(duration);

            // Check if the particle is now invalid
            if (shot->particle.getPosition().y < 0.0f ||
                shot->startTime+5000 < TimingData::get().lastFrameTimestamp ||
                shot->particle.getPosition().z > 200.0f)
            {
                // We simply set the shot type to be unused, so the
                // memory it occupies can be reused by another shot.
                shot->type = UNUSED;
            }
        }
    }

    Application::update();
}

void BallisticDemo::display()
{
    // Clear the viewport and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(-25.0, 8.0, 5.0,  0.0, 5.0, 22.0,  0.0, 1.0, 0.0);

    // Draw a sphere at the firing point, and add a shadow projected
    // onto the ground plane.
    glColor3f(0.0f, 0.0f, 0.0f);
    glPushMatrix();
    glTranslatef(0.0f, 1.5f, 0.0f);
    glutSolidSphere(0.1f, 5, 5);
    glTranslatef(0.0f, -1.5f, 0.0f);
    glColor3f(0.75f, 0.75f, 0.75f);
    glScalef(1.0f, 0.1f, 1.0f);
    glutSolidSphere(0.1f, 5, 5);
    glPopMatrix();

    // Draw some scale lines
    glColor3f(0.75f, 0.75f, 0.75f);
    glBegin(GL_LINES);
    for (unsigned i = 0; i < 200; i += 10)
    {
        glVertex3f(-5.0f, 0.0f, i);
        glVertex3f(5.0f, 0.0f, i);
    }
    glEnd();

    // Render each particle in turn
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            shot->render();
        }
    }

    // Render the description
    glColor3f(0.0f, 0.0f, 0.0f);
    renderText(10.0f, 34.0f, "Click: Fire\n1-4: Select Ammo");

    // Render the name of the current shot type
    switch(currentShotType)
    {
    case PISTOL: renderText(10.0f, 10.0f, "Current Ammo: Pistol"); break;
    case ARTILLERY: renderText(10.0f, 10.0f, "Current Ammo: Artillery"); break;
    case FIREBALL: renderText(10.0f, 10.0f, "Current Ammo: Fireball"); break;
    case LASER: renderText(10.0f, 10.0f, "Current Ammo: Laser"); break;
    }
}

void BallisticDemo::mouse(int button, int state, int x, int y)
{
    // Fire the current weapon.
    if (state == GLUT_DOWN) fire();
}

void BallisticDemo::key(unsigned char key)
{
    switch(key)
    {
    case '1': currentShotType = PISTOL; break;
    case '2': currentShotType = ARTILLERY; break;
    case '3': currentShotType = FIREBALL; break;
    case '4': currentShotType = LASER; break;
    }
}

Application* getApplication()
{
    return new BallisticDemo();
}
