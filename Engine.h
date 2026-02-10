#ifndef _ENGINE_H
#define _ENGINE_H
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "Robot.h"
#include "helper.h"

using namespace std;
class Engine
{
private:

    Engine();
    bool IsWindow(Segment_2 edge);
    void ComputeRobotsVisibilityArea();
    void SetRobotWindow(int robotIndex, Arrangement_2& visibilityArea );
    void ConstructWindowdEnvironment();
    void ComputeDualGraph();
    void Update();
    thread* mUpdateThread;
    bool mFinished = false;

    Arrangement_2 mEnvironment;
    Polygon_2 mEnvironmentPolygon;
    Arrangement_2 mWindowdEnvironment;
    vector<Robot> mRobots;
    map< pair<Kernel::FT, Kernel::FT>, int> mPointToIndexMap;
    map< pair <pair<Kernel::FT, Kernel::FT>, pair<Kernel::FT, Kernel::FT> >, int > mEdgeToIndexMap;
    int EdgeToIndex( Segment_2 edge );
    int VertexToIndex( Point_2 vertex);
    Graph* mDualGraph;
    bool MoveRobot(int activeRobotIndex);
    vector<EventPoint> mEventPoints, mWindowEventpoints;
    pair< int, int > GetEdgeID( Segment_2 edge);
    void FindAllEventPoints();
    map< pair <pair<Kernel::FT, Kernel::FT>, pair<Kernel::FT, Kernel::FT> >, pair<int,int> > mWindowIDMap;
    bool AreConnectable(Point_2 point1, Point_2 point2);
    int mTotalStates = 0;
    int mActiveRobotIndex = -1;

    void CalculateRobotSequences(int robotIndex);
    bool CheckInside(Point_2 checkingPoint, Point_2 firstEndPoint, Point_2 middlePoint, Point_2 secondEndPoint);
    vector<RobotEventPoint> GetFutureEventPoints(ReflectivePoint reflectivePoint, Segment_2 path );
    void TestFunction();
    mutex mStateMutex;
    mutex mContinueWorkMutex;
    condition_variable mContinueWorkCR;
    bool mFinishedStates = false;

    bool IsReflective(Point_2 firstPoint, Point_2 middlePoint, Point_2 lastPoint);
    map< int, ReflectivePoint > mReflectivePoints;
    void FindAllReflectives();
    void DetermineRobotWindowRanges( int robotIndex );

    vector<int> mRobotIndices;
public:
    time_t mStartTime = 0, mLastWrittenFile = 0;
    double mBoundingBoxDiameter;
    pair< double, double> mBoundingBoxCenter;
    static Engine& GetInstance();
    bool Initialize( string worldDataPath = "" );
    bool LoadData();
    bool LoadData(string worldDataPath);
    void DrawWorld();
    void StartWorking();
    void ShutDown();
    int GetTotalStates() const { return mTotalStates; }
    void KeyReleased( char key );
    void MakeRobotDecision( int robotIndex );
    void WriteSummaryToFile ();
    friend class StateManager;

};

#endif
