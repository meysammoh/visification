#ifndef _EVENTPOINT_H
#define _EVENTPOINT_H
#include "helper.h"

using namespace std;


class EventPointBase
{
    vector< pair< int, int> > mEdges;
public:

    bool ContainsWindow( pair< int, int> window );
    bool operator==( const EventPointBase& rhs );
    bool operator!=( const EventPointBase& rhs );
    vector<pair<int, int> > GetEdges() const;
    void PushEdge( pair< int, int > edge );
};

class EventPoint:public EventPointBase
{
public:
    Point_2 mPosition;

};

struct ReflectivePoint
{
    Point_2 mPosition;
    int mIndexInEnvironment;
    // First and Second edge in CCW order:
    Point_2 mFirstEdgeStartCCW;
    Point_2 mSecondEdgeEndCCW;

    Point_2 mFirstEdgeExtendedStartCCW;
    Point_2 mSecondEdgeExtendedEndCCW;

    Point_2 mFirstEdgeOpposite;
    Point_2 mSecondEdgeOpposite;

};

class RobotEventPoint: public EventPoint
{
public:
    RobotEventPoint( EventPoint& eventPoint );
    //Kernel::FT mAngle;
    //Kernel::FT mSquaredDistance;
    ReflectivePoint mCorrespondingReflective;
    Point_2 mMappedOnPathPoint;

};


struct ReflectiveToPathMapPoint
{
    Point_2 mPosition;
    //Point_2 mRelatedReflective;
    //mStartOrEnd is true for start and false for end of segment where window is valid for robot
    //bool mStartOrEnd;
    //ReflectiveToPathMapPoint* mOppositePoint;
    //Segment_2 mLineToReflective;
};



#endif
