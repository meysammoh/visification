#include "Graphics.h"
#include "VisiX.h"
#include "State.h"
#include <math.h>

Graphics& Graphics::GetInstance()
{
    static Graphics instance;
    return instance;
}

void Graphics::Initialize( int argc, char *argv[] )
{
    mZoom = 1;
    glutInit(&argc, argv);

    /*Setting up  The Display
        /    -RGB color model + Alpha Channel = GLUT_RGBA
        */
    glutInitDisplayMode(GLUT_RGBA|GLUT_SINGLE);

    //Configure Window Postion
    glutInitWindowPosition(50, 25);

    //Configure Window Size
    glutInitWindowSize(800,600);


    //Create Window
    glutCreateWindow("Visification");
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND );
    //glTranslated(-1, -1, 0);
    glClearColor( 1,1,1,1 );

    glutKeyboardUpFunc( KeyReleased ); // Tell GLUT to use the method "keyPressed" for key presses
    //Call to the drawing function


    //glutDisplayFunc(RenderFrame);
    glutTimerFunc(20, RenderFrame, 0);
    // Loop require by OpenGL
    glutMainLoop();
}

void Graphics::DrawCircle(float cx, float cy, float r, int num_segments)
{
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x + cx, y + cy);//output vertex

    }
    glEnd();
}

void Graphics::WriteText( string text, double posX, double posY)
{
    glColor3f( 0, 0, 0 );
    //    glDisable(GL_LIGHTING);
    glRasterPos2f( posX, posY );
    int len, i;

    len = text.size();
    for (i = 0; i < len; i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, text[i]);
    }
}

void Graphics::RenderFrame (int delta)
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glPushMatrix(); // save the current matrix
    glScaled( Graphics::GetInstance().mZoom * 2 / VisiX::GetInstance().GetWindowDiameter(),
              Graphics::GetInstance().mZoom * 2 / VisiX::GetInstance().GetWindowDiameter(),
              2 / VisiX::GetInstance().GetWindowDiameter() ); // scale the matrix
    glTranslated( -VisiX::GetInstance().GetWindowCenterX(),
                  -VisiX::GetInstance().GetWindowCenterY(), 0 );
    VisiX::GetInstance().Draw();

    glPopMatrix(); // load the unscaled matrix

    double textStartX = -.98;
    double textStartY = 1;
    WriteText( "[n] : See next state, [z] : Zoom In, [x] : Zoom out", textStartX, textStartY-=.05 );
    WriteText( "___________________________________________________", textStartX, textStartY-=.05 );
    //    string text = "Found " +
    //            to_string( StateManager::GetInstance().GetStatesSize() ) +
    //            " new states from "+to_string( Engine::GetInstance().GetTotalStates() )+ " states." ;
    //    time_t elapsedTime = time( 0 ) - Engine::GetInstance().mStartTime;
    //    uint seconds = elapsedTime;
    //    text += " in "+to_string( seconds )+" seconds.";
    //    WriteText( text, textStartX, textStartY-=.05 );

    glFlush();
    glutTimerFunc(33, RenderFrame, 0);
}

void Graphics::KeyReleased (unsigned char key, int x, int y)
{
    //    Engine::GetInstance().KeyReleased( key );
    if (key == 27)
    { // If they ‘Esc’ key was pressed

        //mContinueWorkCR.notify_one();
        cout<<"Finished!"<<endl;
        exit(0);
    }
    else if( key == 'z')
    {
        Graphics::GetInstance().mZoom *= 1.1;
        //glTranslated(0, 0, 0);
        //glScalef(1, 1, 1);
        //glTranslated(0, 0, 0);
    }
    else if( key == 'x')
    {
        if(Graphics::GetInstance().mZoom / 2 > .0002 )
            Graphics::GetInstance().mZoom *= .9;
        //glTranslated(0, 0, 0);
        //glScalef(zoom, zoom, 1);
        //glTranslated(0, 0, 0);
    }
}
