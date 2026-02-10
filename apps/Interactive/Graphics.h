#ifndef _GRAPHICS_H
#define _GRAPHICS_H
//#ifdef GLENABLED
#include <GL/glut.h>
//#endif
#include <iostream>
#include <string>
using namespace std;

struct COLOR
{
    GLint r,g,b;
};

class Graphics
{
private:
    float mZoom;
    Graphics(){}
public:
    static Graphics& GetInstance();
    void Initialize(int argc, char *argv[]);
    static void WriteText( string text, double posX, double posY);
    static void RenderFrame (int delta);
    static void KeyReleased (unsigned char key, int x, int y);
    static void DrawCircle(float cx, float cy, float r, int num_segments);
};

#endif
