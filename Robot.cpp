#include "Robot.h"
#include <GL/glut.h>
#include <iostream>


Point_2 Robot::GetPosition() const
{
    return mPosition;
}

void Robot::SetPosition(const Point_2 &position)
{
    mPosition = position;
}


Arrangement_2 Robot::GetVisibilityArea() const
{
    return mVisibilityArea;
}

void Robot::SetVisibilityArea(const Arrangement_2 &visibilityArea)
{
    mVisibilityArea = visibilityArea;
}
Robot::Robot(Point_2 position, Segment_2 path)
{

    mPosition = position;
    mColor.r = rand()%255;
    mColor.g = rand()%255;
    mColor.b = rand()%255;
    mPath = path;
    MoveTo( 50 );
}

vector<Segment_2> &Robot::GetWindows()
{
    return mWindows;
}

void Robot::Draw()
{
    glEnable( GL_POINT_SMOOTH );
    glPointSize( 4.0 );
    glBegin( GL_POINTS );
    glColor3f( (float)mColor.r/255, (float)mColor.g/255,(float) mColor.b/255  );
    double x = CGAL::to_double( mPosition.x() );
    double y = CGAL::to_double( mPosition.y() );

    glVertex2d( x, y );
    glEnd();

    glLineWidth(2.0); // Set line width to 2.0
    glBegin(GL_LINES);
    glColor3ub( mColor.r, mColor.g, mColor.b );
    for (vector<Segment_2>::iterator eit = mWindows.begin();
         eit != mWindows.end();  ++eit)
    {
        double fx = CGAL::to_double( eit->source().x() );
        double fy = CGAL::to_double( eit->source().y() );
        double sx = CGAL::to_double( eit->target().x() );
        double sy = CGAL::to_double( eit->target().y() );

        glVertex2d( fx, fy );
        glVertex2d( sx, sy );
    }
    glEnd();
    //draw path
    glPushAttrib(GL_ENABLE_BIT);
    // glPushAttrib is done to return everything to normal after drawing

    glLineStipple(1, 0xA0A0);
    glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINES);
    glColor3ub( mColor.r, mColor.g, mColor.b );

    double fx = CGAL::to_double( mPath.source().x() );
    double fy = CGAL::to_double( mPath.source().y() );
    double sx = CGAL::to_double( mPath.target().x() );
    double sy = CGAL::to_double( mPath.target().y() );

    glVertex2d( fx, fy );
    glVertex2d( sx, sy );

    glEnd();
    glPopAttrib();
    //Draw Event Points
    for( vector<RobotEventPoint>::iterator evenit = mForwardEventPoints.begin();
         evenit != mForwardEventPoints.end(); evenit++ )
    {
        double x = CGAL::to_double( evenit->mPosition.x() );
        double y = CGAL::to_double( evenit->mPosition.y() );
        DrawCircle( x, y, .2, 30 );
    }
    for( vector<RobotEventPoint>::iterator evenit = mBackwardEventPoints.begin();
         evenit != mBackwardEventPoints.end(); evenit++ )
    {
        double x = CGAL::to_double( evenit->mPosition.x() );
        double y = CGAL::to_double( evenit->mPosition.y() );
        DrawCircle( x, y, .2, 30 );
    }
}
/**
 * @brief Move robot on its path
 * @return true if finished paces
 * \todo  Change this to move on event points
 */
void Robot::MoveTo( Point_2 goal )
{
    mPosition = goal;
    Vector_2 distanceVec = mPosition - mPath.source() ;
    mProgress = sqrt ( CGAL::to_double( distanceVec.squared_length() * 100 / mPath.squared_length() ) );
    //cout<<"Moved to "<<mPosition<<endl;
}

void Robot::MoveTo(double progress )
{
    mProgress = progress;
    Point_2 point = mPath.source() + (mPath.target()-mPath.source()) * mProgress/100;
    MoveTo( point );
}

void Robot::SetGoalRandom(  )
{
    double goal = rand() % 101 ;

    mGoal = mPath.source() + (mPath.target()-mPath.source()) * goal/100;
    //cout<< "---- random goal "<<goal<<" % point "<<mGoal<<endl;
    mReachedGoal = false;
}

bool Robot::Move()
{
    //cout<<"Moving Robot ..."<<endl;
    Point_2 pace;
    if( mMedianGoals.size() > 0)
    {

        pace = mMedianGoals.front();
        mMedianGoals.erase( mMedianGoals.begin() );
    }
    else
    {
        pace = mGoal;
    }

    MoveTo( pace );

    mReachedGoal = ( mPosition == mGoal );
    //cout<<"Yetishdi?! "<<mReachedGoal<<endl;
    return mReachedGoal;
}

void Robot::SortSequences()
{
    SortSequence( mForwardEventPoints );

    SortSequence( mBackwardEventPoints );

}

void Robot::SortSequence( vector<RobotEventPoint>& sequence )
{
    vector<RobotEventPoint> sortedSequence;
    for( vector<RobotEventPoint>::iterator evenit = sequence.begin();
         evenit != sequence.end(); evenit++ )
    {
        vector<RobotEventPoint>::iterator sortedEvenit = sortedSequence.begin();
        for( ;sortedEvenit != sortedSequence.end(); sortedEvenit++ )
        {
            Vector_2 cur_ep2StartVect( evenit->mMappedOnPathPoint -
                                       mPosition );
            Kernel::FT curDistFormStartSq = cur_ep2StartVect.squared_length();

            Vector_2 sort_ep2StartVect( sortedEvenit->mMappedOnPathPoint -
                                        mPosition );
            Kernel::FT sortDistFormStartSq = sort_ep2StartVect.squared_length();

            if( sortDistFormStartSq > curDistFormStartSq )
            {
                break;
            }

        }
        sortedSequence.insert( sortedEvenit, *evenit );
    }
    //    cout<<"list\n";
    //    for( vector<RobotEventPoint>::iterator evenit = sortedSequence.begin();
    //         evenit != sortedSequence.end(); evenit++ )
    //    {
    //        cout<<"++++++++++++++ sorted "<<evenit->mMappedOnPathPoint<<endl;
    //    }
    sequence = sortedSequence;
}

void Robot::ClearAllData()
{
    mForwardEventPoints.clear();
    mBackwardEventPoints.clear();
    ClearExtraData();

}
void Robot::ClearExtraData()
{
    mWindows.clear();
    mVisibilityArea.clear();
}

void Robot::GenerateMedianGoals()
{
    mMedianGoals.clear();
    if( mGoal != mPosition )
    {
        PointPlacementType pt = CheckPointPlacement( mGoal, mPosition, mPath.target() );
//        cout<<mGoal<<" "<< mPosition<<" "<< mPath.target()<<": "<<pt<<endl;
        if( pt == PPT_BEFORE_FIRST )
        {
            for( vector<RobotEventPoint>::iterator evenit = mBackwardEventPoints.begin();
                 evenit != mBackwardEventPoints.end(); evenit++ )
            {
                if( CheckPointPlacement( evenit->mMappedOnPathPoint,
                                         mPosition, mGoal ) != PPT_BETWEEN )
                {
                    break;
                }
                mMedianGoals.push_back( evenit->mMappedOnPathPoint );
            }
        }
        else if( pt == PPT_BETWEEN )
        {
            for( vector<RobotEventPoint>::iterator evenit = mForwardEventPoints.begin();
                 evenit != mForwardEventPoints.end(); evenit++ )
            {
                if( CheckPointPlacement( evenit->mMappedOnPathPoint,
                                         mPosition, mGoal ) != PPT_BETWEEN )
                {
                    break;
                }
                mMedianGoals.push_back( evenit->mMappedOnPathPoint );
            }
        }
        else
        {
            cout<<" GenerateMedianGoals: A bug Found!\n";
            exit( 3 );
        }


    }
}
