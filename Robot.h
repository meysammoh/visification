#ifndef _ROBOT_H
#define _ROBOT_H
#include "helper.h"

using namespace std;

class Robot
{
private:
    vector< Point_2 > mMedianGoals;

public:
    Point_2 mPosition;
    Arrangement_2 mVisibilityArea;
    COLOR mColor;
    Segment_2 mPath;
    double mProgress = 0;
    vector<RobotEventPoint> mForwardEventPoints;
    vector<RobotEventPoint> mBackwardEventPoints;
    void SortSequence( vector<RobotEventPoint>& sequence );
    bool mReachedGoal = true;
    Point_2 mGoal;
    vector<Segment_2> mWindows;
    Robot(Point_2 position, Segment_2 path);
    Point_2 GetPosition() const;
    void SetPosition(const Point_2 &position);
    Arrangement_2 GetVisibilityArea() const;
    void SetVisibilityArea(const Arrangement_2 &visibilityArea);
    vector<Segment_2>& GetWindows();
    void Draw();
    Segment_2 GetPath() const { return mPath; }
    void MoveTo(Point_2 goal );
    void MoveTo( double progress );
    bool Move();
    vector<RobotEventPoint>& GetForwardSequence() { return mForwardEventPoints; }
    vector<RobotEventPoint>& GetBackwardSequence() { return mBackwardEventPoints; }
    void SortSequences();
    void SetGoalRandom();
    void ClearAllData();
    void GenerateMedianGoals();
    void ClearExtraData();
    vector< pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > > > mWindowRange;
};

#endif
