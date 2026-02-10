#include "XVisifier.h"

XVisifier::XVisifier():Visifier()
{

}

void XVisifier::Draw()
{
    //Draw Environment

    glLineWidth(2.0); // Set line width to 2.0
    glBegin(GL_LINES);
    glColor3ub( 128, 99, 64 );

    Edge_const_iterator eit;
    for (eit = Visifier::mEnvironment.edges_begin(); eit != Visifier::mEnvironment.edges_end(); ++eit)
    {
        double fx = CGAL::to_double( eit->source()->point().x() );
        double fy = CGAL::to_double( eit->source()->point().y() );
        double sx = CGAL::to_double( eit->target()->point().x() );
        double sy = CGAL::to_double( eit->target()->point().y() );

        glVertex2d( fx, fy );
        glVertex2d( sx, sy );
    }
    glEnd();

    //Draw Robots
    for(vector<Robot>::iterator robit = mRobots.begin(); robit != mRobots.end(); robit++)
    {
        //robit->Draw();
    }
}

