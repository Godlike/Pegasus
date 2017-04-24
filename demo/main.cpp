#include "Pegasus/demo/app.hpp"
#include "Pegasus/demo/ogl_headers.hpp"
#include "Pegasus/demo/timing.hpp"

extern Application* getApplication();

Application* app;

void createWindow(const char* title)
{
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 320);
    glutInitWindowPosition(0, 0);
    glutCreateWindow(title);
}

void update()
{
    TimingData::get().update();
    app->update();
}

void display()
{
    app->display();

    // Update the displayed content.
    glFlush();
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
    app->mouse(button, state, x, y);
}

void reshape(int width, int height) { app->resize(width, height); }

void keyboard(unsigned char key, int x, int y)
{
    // Note we omit passing on the x and y: they are rarely needed.
    app->key(key);
}

void motion(int x, int y) { app->mouseDrag(x, y); }

int main(int argc, char** argv)
{
    // Set up GLUT and the timers
    glutInit(&argc, argv);
    TimingData::init();

    // Create the application and its window
    app = getApplication();
    createWindow(app->getTitle());

    // Set up the appropriate handler functions
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutIdleFunc(update);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    // Run the application
    app->initGraphics();
    glutMainLoop();

    // Clean up the application
    app->deinit();
    delete app;
    TimingData::deinit();
}
