#include <GL/glut.h>
#include "Engine.h"
#include "Config.h"

float zoom = 1;

void WriteText( string text, double posX, double posY)
{
    glColor3f( 1, 1, 1 );
    glRasterPos2f( posX, posY );
    int len, i;

    len = (int)strlen(text.c_str());
    for (i = 0; i < len; i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, text[i]);
    }
}

void RenderFrame (int dummy)
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glPushMatrix(); // save the current matrix
    glScaled( zoom*2/Engine::GetInstance().mBoundingBoxDiameter, zoom*2/Engine::GetInstance().mBoundingBoxDiameter, 2/Engine::GetInstance().mBoundingBoxDiameter); // scale the matrix
    glTranslated( -Engine::GetInstance().mBoundingBoxCenter.first, -Engine::GetInstance().mBoundingBoxCenter.second, 0 );
    Engine::GetInstance().DrawWorld();

    glPopMatrix(); // load the unscaled matrix

    double textStartX = -.98;
    double textStartY = 1;
    WriteText( "[n] : See next state, [z] : Zoom In, [x] : Zoom out", textStartX, textStartY-=.05 );
    WriteText( "___________________________________________________", textStartX, textStartY-=.05 );
    string text = "Found " +
            to_string( StateManager::GetInstance().GetStatesSize() ) +
            " new states from "+to_string( Engine::GetInstance().GetTotalStates() )+ " states." ;
    time_t elapsedTime = time( 0 ) - Engine::GetInstance().mStartTime;
    uint seconds = elapsedTime;
    text += " in "+to_string( seconds )+" seconds.";
    WriteText( text, textStartX, textStartY-=.05 );

    glFlush();
    glutTimerFunc(33, RenderFrame, 0);
}

void keyReleased (unsigned char key, int x, int y)
{
    Engine::GetInstance().KeyReleased( key );

    if( key == 'z')
    {
        zoom *= 1.1;
        //glTranslated(0, 0, 0);
        //glScalef(1, 1, 1);
        //glTranslated(0, 0, 0);
    }
    else if( key == 'x')
    {
        if(zoom / 2 > .0002 )
            zoom *= .9;
        //glTranslated(0, 0, 0);
        //glScalef(zoom, zoom, 1);
        //glTranslated(0, 0, 0);
    }
}


//=========================Main=========================//
int main(int argc, char *argv[])
{
    srand( time(0) );
    string dataPath = "";
    if(argc > 1)
        dataPath = argv[1] ;
    bool result = Engine::GetInstance().Initialize( dataPath );
    if(result)
    {
        Engine::GetInstance().StartWorking();

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
        glClearColor( 0.08,0.04,0.08,0.1 );

        glutKeyboardUpFunc(keyReleased); // Tell GLUT to use the method "keyPressed" for key presses
        //Call to the drawing function


        //glutDisplayFunc(RenderFrame);
        glutTimerFunc(20, RenderFrame, 0);
        // Loop require by OpenGL
        glutMainLoop();

    }
    return 0;
}
