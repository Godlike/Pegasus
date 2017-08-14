/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "demo/Application.hpp"
#include "demo/OglHeaders.hpp"

#include <cstring>

void Application::InitGraphics()
{
    glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    SetView();
}

void Application::SetView()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(m_width) / static_cast<double>(m_height), 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void Application::Display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);
    glVertex2i(1, 1);
    glVertex2i(639, 319);
    glEnd();
}

const char* Application::GetTitle() { return "Pegas Demo"; }

void Application::Deinit() {}

void Application::Update() { glutPostRedisplay(); }

void Application::Key(unsigned char key) {}

void Application::Resize(int width, int height)
{
    // Avoid the divide by zero.
    if (height <= 0)
        height = 1;

    // Set the internal variables and update the view
    Application::m_width = width;
    Application::m_height = height;
    glViewport(0, 0, width, height);
    SetView();
}

void Application::Mouse(int button, int state, int x, int y) {}

void Application::MouseDrag(int x, int y) {}

// The following methods aren't intended to be overloaded
void Application::RenderText(float x, float y, float r, float g, float b, const char* text, void* font) const
{
    glDisable(GL_DEPTH_TEST);

    // Temporarily set up the view in orthographic projection.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, static_cast<double>(m_width), 0.0, static_cast<double>(m_height), -1.0, 1.0);

    // Move to modelview mode.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Ensure we have a font
    if (font == nullptr) {
        font = GLUT_BITMAP_HELVETICA_10;
    }

    // Loop through characters displaying them.
    size_t len = strlen(text);
    
    glColor3f(r, g, b);
    glRasterPos2f(x, y);
    for (const char* letter = text; letter < text + len; letter++) {

        // If we meet a newline, then move down by the line-height
        // TODO: Make the line-height a function of the font
        if (*letter == '\n') {
            y -= 12.0f;
            glRasterPos2f(x, y);
        }
        glutBitmapCharacter(font, *letter);
    }

    // Pop the matrices to return to how we were before.
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}
