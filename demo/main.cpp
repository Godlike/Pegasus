/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/

#include "demo/Application.hpp"
#include "demo/OglHeaders.hpp"
#include "demo/Timing.hpp"

extern Application* GetApplication();

Application* g_application;

void createWindow(const char* title)
{
    glutSetOption(GLUT_MULTISAMPLE, 8);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(1366, 768);
    glutInitWindowPosition(200, 200);
    glutCreateWindow(title);
}

void update()
{
    TimingData::Get().Update();
    g_application->Update();
}

void display()
{
    g_application->Display();

    // Update the displayed content.
    glFlush();
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
    g_application->Mouse(button, state, x, y);
}

void reshape(int width, int height)
{
    g_application->Resize(width, height);
}

void keyboard(unsigned char key, int x, int y)
{
    // Note we omit passing on the x and y: they are rarely needed.
    g_application->Key(key);
}

void motion(int x, int y)
{
    g_application->MouseDrag(x, y);
}

int main(int argc, char** argv)
{
    // Set up GLUT and the timers
    glutInit(&argc, argv);
    TimingData::Init();

    // Create the application and its window
    g_application = GetApplication();
    createWindow(g_application->GetTitle());

    // Set up the appropriate handler functions
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutIdleFunc(update);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    // Run the application
    g_application->InitGraphics();
    glutMainLoop();

    // Clean up the application
    g_application->Deinit();
    delete g_application;
    TimingData::Deinit();
}
