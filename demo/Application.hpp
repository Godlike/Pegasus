/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_DEMO_APP_HPP
#define PEGASUS_DEMO_APP_HPP

class Application
{
public:
    virtual const char* GetTitle();
    virtual void InitGraphics();
    virtual void SetView();
    virtual void Deinit();
    virtual void Display();
    virtual void Update();
    virtual void Key(unsigned char key);
    virtual void Resize(int width, int height);
    virtual void Mouse(int button, int state, int x, int y);
    virtual void MouseDrag(int x, int y);
    void RenderText(float x, float y, float r, float g, float b, const char* text, void* font = nullptr) const;

protected:
    int m_height;
    int m_width;
};

#endif // PEGASUS_DEMO_APP_HPP
