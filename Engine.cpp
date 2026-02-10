#include "Engine.h"
#include <fstream>
#include <typeinfo>
using namespace std;

Engine::Engine()
{

}

Engine& Engine::GetInstance()
{
    static Engine instance;
    return instance;
}

bool Engine::Initialize( string worldDataPath )
{
    mStartTime = time( 0 );
    TestFunction();
    bool result = true;
    result = LoadData( worldDataPath );
    if(result)
    {
        CGAL::Bbox_2 boundingBox = mEnvironmentPolygon.bbox();
        mBoundingBoxDiameter = sqrt( pow( (boundingBox.xmax()-boundingBox.xmin()), 2 ) +
                                     pow( (boundingBox.ymax()-boundingBox.ymin()), 2 ) );
        mBoundingBoxCenter.first = (boundingBox.xmax() + boundingBox.xmin())/2;
        mBoundingBoxCenter.second = (boundingBox.ymax() + boundingBox.ymin())/2;

        Arrangement_2::Vertex_iterator vit;

        int index = 0;
        for (vit = mEnvironment.vertices_begin();
             vit != mEnvironment.vertices_end(); ++vit)
        {
            //cout<<endl<<"New Index "<<index<<" for "<<vit->point();
            mPointToIndexMap[pair<Kernel::FT, Kernel::FT>(vit->point().x(),vit->point().y())] = index;
            index++;
        }

        Edge_const_iterator eit;
        int edgeIndex = 0;
        for (eit = mEnvironment.edges_begin(); eit != mEnvironment.edges_end(); ++eit)
        {
            pair<Kernel::FT, Kernel::FT> source(eit->source()->point().x(),
                                                eit->source()->point().y());
            pair<Kernel::FT, Kernel::FT> target(eit->target()->point().x(),
                                                eit->target()->point().y());
            //            cout<<endl<<"New Edge Index "<<edgeIndex<<" for "<<eit->source()->point() <<
            //                  " to "<<eit->target()->point() ;
            mEdgeToIndexMap[pair< pair<Kernel::FT, Kernel::FT>,
                    pair<Kernel::FT, Kernel::FT>> (source, target)] = edgeIndex++;
        }
        //cout<<endl;
        //mActiveRobotIndex = rand() % mRobots.size();
        FindAllReflectives();
        for( int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++ )
        {
            DetermineRobotWindowRanges( robotIndex );
        }

        ComputeDualGraph();
        FindAllEventPoints();
        for( int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++ )
        {
            CalculateRobotSequences( robotIndex );
        }
        mRobotIndices.resize( mRobots.size() );
        StateManager::GetInstance().SaveState( true );
        mTotalStates++;
    }

    return result;
}

int Engine::VertexToIndex( Point_2 vertex )
{
    int index = -1;
    map< pair<Kernel::FT, Kernel::FT>, int>::iterator iter =
            mPointToIndexMap.find( pair<Kernel::FT, Kernel::FT>( vertex.x(), vertex.y() ) );
    if( iter != mPointToIndexMap.end() )
    {
        index = iter->second;

    }
    return index;
}

void Engine::ComputeRobotsVisibilityArea()
{

    Arrangement_2::Face_const_handle * face;
    CGAL::Arr_naive_point_location<Arrangement_2> pl(mEnvironment);
    mWindowIDMap.clear();
    for(int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++)
    {
        CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(mRobots[robotIndex].GetPosition());
        // The query point locates in the interior of a face
        face = boost::get<Arrangement_2::Face_const_handle> (&obj);

        // Define visibiliy object type that computes regularized visibility area
        typedef CGAL::Simple_polygon_visibility_2<Arrangement_2, CGAL::Tag_true> RSPV;
        Arrangement_2 regular_output;
        RSPV regular_visibility(mEnvironment);
        regular_visibility.compute_visibility(mRobots[robotIndex].GetPosition(), *face, regular_output);

        SetRobotWindow( robotIndex, regular_output);
    }
}

/// \todo: Use map to find out if an edge is window
bool Engine::IsWindow( Segment_2 edge)
{
    bool result = true;
    Edge_const_iterator eit;

    for (eit = mEnvironment.edges_begin(); eit != mEnvironment.edges_end(); ++eit)
    {
        Segment_2 alaki (eit->source()->point(),eit->target()->point());
        if( alaki.has_on( edge.source() ) &&
                alaki.has_on( edge.target() )  )
        {
            result = false;
            break;
        }
    }

    return result;
}

void Engine::SetRobotWindow( int robotIndex, Arrangement_2& visibilityArea )
{
    Edge_iterator eit;
    mRobots[robotIndex].GetWindows().clear();

    for (eit = visibilityArea.edges_begin(); eit != visibilityArea.edges_end(); ++eit)
    {
        if( IsWindow( Segment_2( eit->source()->point(),eit->target()->point() ) ))
        {

            pair<Kernel::FT, Kernel::FT> source(eit->source()->point().x(),
                                                eit->source()->point().y());
            pair<Kernel::FT, Kernel::FT> target(eit->target()->point().x(),
                                                eit->target()->point().y());

            int vertexIndex = -1;
            Segment_2 robotoSource( mRobots[robotIndex].GetPosition(), eit->source()->point());
            Segment_2 robotoTarget( mRobots[robotIndex].GetPosition(), eit->target()->point());


            if( robotoSource.squared_length() < robotoTarget.squared_length()) //Source point is reflective vertex
            {
                vertexIndex = VertexToIndex(eit->source()->point());
                mRobots[robotIndex].GetWindows().push_back(
                            Segment_2(eit->source()->point(),eit->target()->point()) );
            }
            else //Target point is reflective
            {
                vertexIndex = VertexToIndex(eit->target()->point());
                mRobots[robotIndex].GetWindows().push_back(
                            Segment_2(eit->target()->point(), eit->source()->point()) );
            }
            mWindowIDMap[pair< pair<Kernel::FT, Kernel::FT>,
                    pair<Kernel::FT, Kernel::FT>> (source, target)] = pair<int , int >(robotIndex, vertexIndex);
            //Add a reversed key to map for faster search
            mWindowIDMap[pair< pair<Kernel::FT, Kernel::FT>,
                    pair<Kernel::FT, Kernel::FT>> (target, source)] = pair<int , int >(robotIndex, vertexIndex);
        }
        else
        {
        }
    }

    mRobots[robotIndex].SetVisibilityArea( visibilityArea );
}

void Engine::ConstructWindowdEnvironment()
{
    //Create windowed environment to detect cells
    mWindowdEnvironment = mEnvironment;
    for(vector<Robot>::iterator robit = mRobots.begin(); robit != mRobots.end(); robit++)
    {
        vector<Segment_2> windows = robit->GetWindows();
        for (vector<Segment_2>::iterator eit = windows.begin(); eit != windows.end(); ++eit)
        {
            CGAL::insert(mWindowdEnvironment,
                         Segment_2(eit->source(), eit->target()));
        }
    }
}

void Engine::ComputeDualGraph()
{
    //cout<<"Computing dual graph ... "<<endl;
    ComputeRobotsVisibilityArea();
    ConstructWindowdEnvironment();

    //Find neighbor of each edge in windowed environment
    map< pair<int, int>, vector<int> > dualTemp;
    Arrangement_2::Face_iterator              fit;
    Arrangement_2::Ccb_halfedge_circulator    curr;
    int                                       faceCounter = 0;
    //Create a map to find index of points in windowed environment
    Arrangement_2::Vertex_iterator vit;
    map< pair<Kernel::FT, Kernel::FT>, int> windowedVertexIndexMap;
    map< int, Point_2> windowedInverseVertexIndexMap;
    int index = 0;
    for (vit = mWindowdEnvironment.vertices_begin();
         vit != mWindowdEnvironment.vertices_end(); ++vit)
    {
        windowedVertexIndexMap[pair<Kernel::FT, Kernel::FT>(vit->point().x(),vit->point().y())] = index;
        windowedInverseVertexIndexMap[index] = vit->point();
        index++;
    }
    mDualGraph = new Graph( mWindowdEnvironment.number_of_faces() - 1 );
    for (fit = mWindowdEnvironment.faces_begin(); fit != mWindowdEnvironment.faces_end(); ++fit)
    {
        if (! fit->is_unbounded())
        {
            curr = fit->outer_ccb();
            //cout<<"Info of cell "<<faceCounter<<endl;
            do
            {
                GraphNode* nodes = mDualGraph->GetNodes();
                vector<pair<int,int>>& cellInfo =  nodes[faceCounter].mCellInfo;
                pair<int,int> edgeID = GetEdgeID(Segment_2(curr->source()->point() , curr->target()->point()));
                if(  edgeID.second > -1 )
                {
                    cellInfo.push_back( edgeID );
                    //cout<<"< "<<edgeID.first<<", "<<edgeID.second<<" >"<<endl;
                }
                int from = windowedVertexIndexMap[pair<Kernel::FT, Kernel::FT>(curr->source()->point().x(),
                                                                               curr->source()->point().y())];
                int to = windowedVertexIndexMap[pair<Kernel::FT, Kernel::FT>(curr->target()->point().x(),
                                                                             curr->target()->point().y())];
                if(to < from)
                {
                    int temp = from;
                    from = to;
                    to = temp;
                }
                vector<int>& edgeFaceNeighbors = dualTemp[pair<int,int>(from, to)];
                if(std::find(edgeFaceNeighbors.begin(),
                             edgeFaceNeighbors.end(), faceCounter) == edgeFaceNeighbors.end())
                {
                    edgeFaceNeighbors.push_back(faceCounter);
                }
                ++curr;
            }
            while (curr != fit->outer_ccb());
            faceCounter++;
        }
    }

    //Construct dual graph

    for(map< pair<int, int>, vector<int> >::iterator dualiter = dualTemp.begin();
        dualiter != dualTemp.end();dualiter++)
    {

        vector<int>& edgeFaceNeighbors = dualiter->second;
        if(edgeFaceNeighbors.size() > 1)
        {
            mDualGraph->AddEdge( edgeFaceNeighbors[0], edgeFaceNeighbors[1] );
        }
    }
}

void Engine::DrawWorld()
{
    //Draw Environment

    glLineWidth(2.0); // Set line width to 2.0
    glBegin(GL_LINES);
    glColor3ub( 128, 99, 64 );

    Edge_const_iterator eit;
    for (eit = mEnvironment.edges_begin(); eit != mEnvironment.edges_end(); ++eit)
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
        robit->Draw();
    }
}

void Engine::StartWorking()
{
    mUpdateThread = new thread( &Engine::Update, this );
    //    Update();
}

void Engine::MakeRobotDecision( int robotIndex )
{
    mRobots[robotIndex].SetGoalRandom();
}

void Engine::Update()
{

    // This calls IncrementingSequence::operator() for each element in the vector,
    // and assigns the result to the element
    generate( mRobotIndices.begin(), mRobotIndices.end(), IncrementingSequence() );
    while( !mFinished )
    {
        bool phaseIncomplete = true;
        for( int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++ )
        {
            MakeRobotDecision( robotIndex );
        }
        while( phaseIncomplete )
        {
            for( vector<int>::iterator permRobotIndex = mRobotIndices.begin();
                 permRobotIndex != mRobotIndices.end(); permRobotIndex++ )
            {

                bool reachedGoal = false;
                mActiveRobotIndex = *permRobotIndex;
                mRobots[mActiveRobotIndex].GenerateMedianGoals();
                while( !reachedGoal )
                {

                    unique_lock<mutex> lk( mContinueWorkMutex );
                    mContinueWorkCR.wait( lk );
                    if( mFinished )
                    {
                        break;
                    }
                    mTotalStates++;
                    reachedGoal = MoveRobot( mActiveRobotIndex );

                    ComputeDualGraph();
                    FindAllEventPoints();
                    for( int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++)
                    {
                        CalculateRobotSequences( robotIndex );
                    }
                    bool isNew = StateManager::GetInstance().SaveState( false );
                    if( isNew )
                    {
                        StateManager::GetInstance().GetWorkingState()->ClearExtraData();
                    }

                    lk.unlock();
                    mContinueWorkCR.notify_one();
                    //                    sleep( 1 );
                }
            }

            phaseIncomplete = next_permutation( mRobotIndices.begin(), mRobotIndices.end() );
            if( phaseIncomplete )
            {
                StateManager::GetInstance().RestoreFirstOfPace();
            }
        }
        StateManager::GetInstance().SetFirstOfPhaseToWorkingState(  );
        if( time(0) - mLastWrittenFile >60 )
        {
            WriteSummaryToFile();
            mLastWrittenFile = time(0);
        }
    }
}

void Engine::ShutDown()
{
    mFinished = true;
    //    mUpdateThread->join();
    //    delete(mUpdateThread);
}

bool Engine::LoadData()
{
    bool result = true;
    //Load environment
    Point_2 p1(0,4), p2(0,0), p3(3,2), p4(4,0), p5(4,4), p6(1,2);
    std::vector<Segment_2> segments;
    segments.push_back(Segment_2(p1, p2));
    segments.push_back(Segment_2(p2, p3));
    segments.push_back(Segment_2(p3, p4));
    segments.push_back(Segment_2(p4, p5));
    segments.push_back(Segment_2(p5, p6));
    segments.push_back(Segment_2(p6, p1));

    mEnvironmentPolygon.push_back(p1);
    mEnvironmentPolygon.push_back(p2);
    mEnvironmentPolygon.push_back(p3);
    mEnvironmentPolygon.push_back(p4);
    mEnvironmentPolygon.push_back(p5);
    mEnvironmentPolygon.push_back(p6);
    CGAL::insert_non_intersecting_curves(mEnvironment,segments.begin(),segments.end());

    //Load robots
    Point_2 q(2, 1.5);
    mRobots.push_back( Robot(q, Segment_2( q, Point_2( 3.9, 3.7 ))) );
    Point_2 q2(2, 2.5);
    mRobots.push_back( Robot(q2, Segment_2( q2, Point_2( .1, .1 ))) );
    Point_2 q3(.5, 2.25);
    mRobots.push_back( Robot(q3, Segment_2( q3, Point_2( 1, 1 ))) );
    cout<<"loaded";
    return result;
}

bool Engine::LoadData(string worldDataPath)
{
    bool result = true;
    if( worldDataPath == "")
    {
        cout << "Loading a predefined data set ..." << endl;
        result = LoadData();
    }
    else
    {
        ifstream infile( worldDataPath );
        string line;
        int state = -1;
        std::vector<Point_2> points;
        while ( getline( infile, line ) )
        {
            istringstream iss(line);
            cout<<line<<endl;
            if(line == "Environment")
            {
                state = 0;
                cout<<"Env State"<<endl;
            }
            else if(line == "Robots")
            {
                state = 1;
                cout<<"robot State"<<endl;
            }
            else if(line != "" && line[0] != '#' )
            {
                float x, y;
                if (!(iss >> x >> y)) { result = false; break; } // error
                if( state == 0 )
                {

                    points.push_back(Point_2( x, y ));
                }
                else if( state == 1 )
                {
                    //Load robots
                    float  pEndX, pEndY;
                    if (!(iss >> pEndX >> pEndY)) { result = false; break; } // error
                    Point_2 rpos ( x ,y ) ;
                    Point_2 pStart ( x ,y) ;
                    Point_2 pEnd ( pEndX ,pEndY ) ;
                    mRobots.push_back( Robot( rpos, Segment_2( pStart, pEnd ) ));
                }
            }

            // process pair (a,b)
        }

        if(result)
        {
            cout<<"Creating world with "<<points.size()<<" points ... "<<endl;
            std::vector<Segment_2> segments;
            for(vector<Point_2>::iterator from = points.begin(); from != points.end(); from++)
            {
                vector<Point_2>::iterator to = from + 1;
                if( to == points.end() )
                {
                    to = points.begin();
                }
                segments.push_back(Segment_2( *from, *to ));
                mEnvironmentPolygon.push_back( *from );

            }

            CGAL::insert_non_intersecting_curves(mEnvironment,segments.begin(),segments.end());

        }
    }
    return result;
}

bool Engine::MoveRobot( int activeRobotIndex )
{
    return mRobots[activeRobotIndex].Move();
}

int Engine::EdgeToIndex( Segment_2 edge )
{

    pair<Kernel::FT, Kernel::FT> source(edge.source().x(),
                                        edge.source().y());
    pair<Kernel::FT, Kernel::FT> target(edge.target().x(),
                                        edge.target().y());
    return mEdgeToIndexMap[pair< pair<Kernel::FT, Kernel::FT>,
            pair<Kernel::FT, Kernel::FT>> (source, target)];
}

/**
 * @brief Finds Id of edge.
 * @param edge
 * @return If edge is a window , return value will be < Robot's ID, Refelctive VertexID > \
 * If edge is environment's edge, return value will be < -1, Edge's ID in environment >
 */
pair< int, int > Engine::GetEdgeID( Segment_2 edge)
{
    pair< int, int > edgeID;
    edgeID.first = -1;
    edgeID.second = -1;


    Edge_const_iterator eit;
    for (eit = mEnvironment.edges_begin(); eit != mEnvironment.edges_end(); ++eit)
    {
        Segment_2 envEdge (eit->source()->point(),eit->target()->point());
        if( envEdge.has_on( edge.source() ) &&
                envEdge.has_on( edge.target() )  )
        {
            edgeID.second = EdgeToIndex( envEdge );
            break;
        }
    }

    if( edgeID.second == -1 )
    {
        map< pair <pair<Kernel::FT, Kernel::FT>, pair<Kernel::FT, Kernel::FT> >,
                pair<int,int> >::iterator winIditer = mWindowIDMap.begin();
        for( winIditer; winIditer != mWindowIDMap.end(); winIditer++)
        {
            Segment_2 window (
                        Point_2(winIditer->first.first.first,winIditer->first.first.second),
                        Point_2(winIditer->first.second.first,winIditer->first.second.second) );
            if( window.has_on( edge.source() ) &&
                    window.has_on( edge.target() )  )
            {
                edgeID = winIditer->second;
                break;
            }
        }
    }
    return edgeID;
}

void Engine::FindAllEventPoints()
{
    mEventPoints.clear();
    mWindowEventpoints.clear();
    Vertex_const_iterator v;
    for (v = mWindowdEnvironment.vertices_begin(); v != mWindowdEnvironment.vertices_end(); ++v)
    {
        Arrangement_2::Halfedge_around_vertex_const_circulator first, curr;
        first = curr = v->incident_halfedges();
        bool isEventPoint = false;
        EventPoint eventPoint;

        do {
            // Note that the current halfedge is directed from u to v:
            Arrangement_2::Vertex_const_handle u = curr->source();
            //u->point() ;
            pair< int, int > edgeID = GetEdgeID( Segment_2( v->point(), u->point()) );
            eventPoint.PushEdge( edgeID );
            if( edgeID.first > -1 )
            {
                //This is part of a robot's window
                isEventPoint = true;

            }

        } while (++curr != first);
        if( isEventPoint )
        {
            eventPoint.mPosition = v->point();
            //cout<<"Found EventPoint ( "<<eventPoint.mPosition <<" )"<<endl;
            //            for(vector<pair<int,int>>::iterator it = eventPoint.mEdges.begin();
            //                it != eventPoint.mEdges.end(); it++)
            //            {
            //                cout<<it->first<<", "<<it->second<<endl;
            //            }
            mEventPoints.push_back( eventPoint );
        }

    }
    mWindowEventpoints = mEventPoints;

    //Add all robots as event point
    for( int robotCounter = 0; robotCounter < mRobots.size(); robotCounter++ )
    {
        EventPoint eventPoint;
        eventPoint.mPosition = mRobots[robotCounter].GetPosition();
        //cout<<"Found EventPoint ( "<<eventPoint.mPosition <<" )"<<endl;
        //For event points which created from robot position add an edge with id containing robot's index and -1.
        eventPoint.PushEdge( pair<int,int>(robotCounter, -1) );
        mEventPoints.push_back( eventPoint );
    }
    //Add vertices of Environment as event points
    Arrangement_2::Vertex_iterator vit;
    int index = 0;
    for (vit = mEnvironment.vertices_begin();
         vit != mEnvironment.vertices_end(); ++vit)
    {
        EventPoint eventPoint;
        eventPoint.mPosition = vit->point();
        //cout<<"Found EventPoint ( "<<eventPoint.mPosition <<" )"<<endl;
        //For event points which created from environment vertices add an edge with id containing -2 and vertex index.
        eventPoint.PushEdge( pair<int,int>( -2, VertexToIndex( vit->point() )) );
        //cout<< "inserted < "<<"-2, "<< VertexToIndex( vit->point() )<<endl;
        mEventPoints.push_back( eventPoint );
    }
}

bool Engine::AreConnectable( Point_2 point1, Point_2 point2)
{
    bool result = true;
    //cout<<"First Point is "<<point1<<endl;
    //cout<<"Second Point is "<<point2<<endl;
    Segment_2 connectingLine( point1, point2 );

    Edge_const_iterator eit;
    for (eit = mEnvironment.edges_begin(); eit != mEnvironment.edges_end(); ++eit)
    {
        Segment_2 edge( eit->source()->point(), eit->target()->point() );

        Point_2 ipoint;
        Segment_2 iseg;
        CGAL::Object intersect = CGAL::intersection( connectingLine, edge );

        if (CGAL::assign(ipoint, intersect))
        {
            if( ipoint != point1 && ipoint != point2 )
            {
                result = false;
                break;
            }
        }
        else if (CGAL::assign(iseg, intersect))
        {
            result = false;
            break;
        }

    }

    if( result )
    {
        //Check Connecting edge not to be visible just from outside
        Point_2 middlePoint( (point1.x() + point2.x() ) / 2, (point1.y() + point2.y() ) / 2);

        if( mEnvironmentPolygon.bounded_side( middlePoint ) == CGAL::ON_UNBOUNDED_SIDE )
        {
            result = false;
        }
    }

    return result;
}

void Engine::CalculateRobotSequences( int robotIndex)
{
    Robot& robot = mRobots[robotIndex];
    robot.GetBackwardSequence().clear();
    robot.GetForwardSequence().clear();

    vector<RobotEventPoint> fPossibleEventpoints, bPossibleEventpoints;

    vector< pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > > >::iterator rangit;
    for( rangit = robot.mWindowRange.begin(); rangit != robot.mWindowRange.end(); rangit++ )
    {
        ReflectivePoint reflectivePoint = rangit->first;
        pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > wValiditySegment = rangit->second;

        PointPlacementType pointPlacement = CheckPointPlacement( robot.mPosition,
                                                                 wValiditySegment.first.mPosition,
                                                                 wValiditySegment.second.mPosition);
        if( pointPlacement == PPT_BETWEEN )
        {
            //            cout<<" >>>>>>>>>>>>>> pp between"<<endl;
            Point_2 forwardEndpoint = wValiditySegment.second.mPosition;
            Point_2 backwardEndpoint = wValiditySegment.first.mPosition;
            vector<RobotEventPoint> fEvents = GetFutureEventPoints( reflectivePoint,
                                                                    Segment_2( robot.mPosition, forwardEndpoint ) );
            vector<RobotEventPoint> bEvents = GetFutureEventPoints( reflectivePoint,
                                                                    Segment_2( robot.mPosition, backwardEndpoint ) );

            fPossibleEventpoints.insert( fPossibleEventpoints.end(),
                                         fEvents.begin(), fEvents.end() );
            bPossibleEventpoints.insert( bPossibleEventpoints.end(),
                                         bEvents.begin(), bEvents.end() );
            ///\todo: Remove event points containing current window

        }
        else if( pointPlacement == PPT_BEFORE_FIRST )
        {
            //            cout<<" >>>>>>>>>>>>>> pp before"<<endl;

            vector<RobotEventPoint> fEvents = GetFutureEventPoints( reflectivePoint,
                                                                    Segment_2( wValiditySegment.first.mPosition,
                                                                               wValiditySegment.second.mPosition ) );
            fPossibleEventpoints.insert( fPossibleEventpoints.end(),
                                         fEvents.begin(), fEvents.end() );
        }
        else if( pointPlacement == PPT_AFTER_SECOND )
        {
            //            cout<<" >>>>>>>>>>>>>> pp after"<<endl;
            vector<RobotEventPoint> bEvents = GetFutureEventPoints( reflectivePoint,
                                                                    Segment_2( wValiditySegment.first.mPosition,
                                                                               wValiditySegment.second.mPosition ) );
            bPossibleEventpoints.insert( bPossibleEventpoints.end(),
                                         bEvents.begin(), bEvents.end() );
        }
        else
        {
            cout<<" >>>>>>>>>>>>>> Bug 1 in CalculateRobotSequences "<<endl;
            exit( 2 );
        }
    }

    robot.GetBackwardSequence() = bPossibleEventpoints;
    robot.GetForwardSequence() = fPossibleEventpoints;
    robot.SortSequences();


}

bool Engine::CheckInside( Point_2 checkingPoint,
                          Point_2 firstEndPoint, Point_2 middlePoint, Point_2 secondEndPoint)
{
    bool result = false;
    //        cout<<"Checking if "<<checkingPoint<<" is between ( "<<
    //              firstEndPoint<<" - "<<middlePoint<<" ) and ( "<<
    //              secondEndPoint<<" - "<<middlePoint<<" ) ? \n";

    Vector_2 firstVector = firstEndPoint - middlePoint;
    Vector_2 secondVector = secondEndPoint - middlePoint;
    Vector_2 checkingVector = checkingPoint - middlePoint;

    double firstSize = CGAL::to_double(firstVector.squared_length());
    double secondSize = CGAL::to_double(secondVector.squared_length());
    double middleSize = CGAL::to_double(checkingVector.squared_length());

    if( firstSize != 0 )
        firstVector = firstVector/sqrt( firstSize );
    if( secondSize != 0 )
        secondVector = secondVector/sqrt( secondSize );
    if( middleSize != 0 )
        checkingVector = checkingVector/sqrt( middleSize );

    //        cout<<"Vectors after normalizing- firstVect ( "<< firstVector<<
    //              " ) checkingVect( "<<checkingVector<<" ) and secondVect ( "<<
    //              secondVector<<" ) \n";

    double cross = Cross(firstVector, secondVector );
    //        cout<< "Cross of first and second vect is "<<cross<<endl;
    if( cross < 0 )
    {
        //We should swap first and second vector
        //                cout<<"We should swap first and second vector"<<endl;
        Vector_2 temp = firstVector;
        firstVector = secondVector;
        secondVector = temp;
    }

    double crossWithFirst = Cross(firstVector, checkingVector );
    //        cout<< "Cross of first and checking vect is "<<crossWithFirst<<endl;
    double crossWithSecond = Cross( checkingVector, secondVector );
    //        cout<< "Cross of checking and second vect is "<<crossWithSecond<<endl;
    if( crossWithFirst >= 0 && crossWithSecond >= 0 )
    {
        result = true;
    }
    return result;
}

vector<RobotEventPoint> Engine::GetFutureEventPoints( ReflectivePoint reflectivePoint, Segment_2 path )
{
    vector<RobotEventPoint> possiblePoints;
    //Create possible area
    //first point
    Point_2 firstPoint = ExtendSegmentFromEnd( reflectivePoint.mPosition, path.source(), mBoundingBoxDiameter, true );
    //second point
    Point_2 secondPoint = ExtendSegmentFromEnd( reflectivePoint.mPosition, path.target(), mBoundingBoxDiameter, true );

    //Find eventpoints positioned on possible area

    for( vector<EventPoint>::iterator eventitor = mEventPoints.begin();
         eventitor != mEventPoints.end(); eventitor++ )
    {
        //        cout<<"Checking point "<<eventitor->mPosition<<endl;
        if( CheckInside( eventitor->mPosition, firstPoint,reflectivePoint.mPosition, secondPoint) &&
                AreConnectable( eventitor->mPosition, reflectivePoint.mPosition ) )
        {
            RobotEventPoint eventPoint = *eventitor ;
            if( reflectivePoint.mPosition != eventitor->mPosition )
            {
                Point_2 extEvetToRef = ExtendSegmentFromEnd( reflectivePoint.mPosition,
                                                             eventitor->mPosition,
                                                             mBoundingBoxDiameter, true );
                CGAL::Object intersection;
                IntersectionType intType = FindIntersection( path,
                                                             Segment_2( reflectivePoint.mPosition,
                                                                        extEvetToRef), intersection );
                if( intType == IT_POINT )
                {
                    CGAL::assign( eventPoint.mMappedOnPathPoint, intersection );
                    eventPoint.mCorrespondingReflective = reflectivePoint;
                    possiblePoints.push_back( eventPoint );
                }
            }

        }
    }
    return possiblePoints;
}



void Engine::TestFunction()
{

    //    Point_2 p1(10,1),p2(4,2),p3(2,8),p4(9,0),p5(0,10),p6(-2,9),
    //            p7(-4,8),p8(-6,2),p9(-12,0),p10(-10,-1),p11(-1,-8),p12(0,-2),
    //            p13(20,-7),p14(9,0), pm(0,0);
    //    cout<<"&&&&&&&& "<<CheckInside( p13,p3,pm,p11)<<" "<<CheckInside( p4,p6,pm,p7)<<endl;
    //    exit(0);
}
void Engine::KeyReleased( char key )
{
    //cout<<key<<endl;
    if (key == 27)
    { // If they ‘Esc’ key was pressed
        ShutDown();
        mContinueWorkCR.notify_one();
        cout<<"Finished!"<<endl;
        exit(0);
    }
    else if (key == 'n')
    {
        mContinueWorkCR.notify_one();
    }

}

bool Engine::IsReflective( Point_2 firstPoint, Point_2 middlePoint, Point_2 lastPoint )
{
    return AngleBetweenThreePoints( firstPoint, middlePoint, lastPoint ) > M_PI;
}

void Engine::FindAllReflectives()
{
    for( int vertexIndex = 0; vertexIndex < mEnvironment.number_of_vertices(); vertexIndex++ )
    {
        int firstIndex = vertexIndex - 1;
        if( firstIndex < 0)
        {
            firstIndex = mEnvironment.number_of_vertices() - 1;
        }
        int middleIndex = vertexIndex;
        int secondIndex = ( vertexIndex + 1 ) % mEnvironment.number_of_vertices();

        if( IsReflective( mEnvironmentPolygon.vertex( firstIndex ),
                          mEnvironmentPolygon.vertex( middleIndex ),
                          mEnvironmentPolygon.vertex( secondIndex ) )  )
        {
            ReflectivePoint reflectivePoint;
            reflectivePoint.mIndexInEnvironment = vertexIndex;
            reflectivePoint.mPosition = mEnvironmentPolygon.vertex( middleIndex );
            reflectivePoint.mFirstEdgeStartCCW =  mEnvironmentPolygon.vertex( firstIndex );
            reflectivePoint.mSecondEdgeEndCCW = mEnvironmentPolygon.vertex( secondIndex );
            reflectivePoint.mFirstEdgeOpposite = ExtendSegmentFromEnd( reflectivePoint.mPosition,
                                                                       reflectivePoint.mFirstEdgeStartCCW,
                                                                       mBoundingBoxDiameter, true);
            //            cout <<"Opposite extension of ("<<reflectivePoint.mPosition<<", "<<
            //                   reflectivePoint.mFirstEdgeStartCCW<<") is "<<reflectivePoint.mFirstEdgeOpposite<<endl;
            reflectivePoint.mSecondEdgeOpposite = ExtendSegmentFromEnd( reflectivePoint.mPosition,
                                                                        reflectivePoint.mSecondEdgeEndCCW,
                                                                        mBoundingBoxDiameter, true);
            //            cout <<"Opposite extension of ("<<reflectivePoint.mPosition<<", "<<
            //                   reflectivePoint.mSecondEdgeEndCCW<<") is "<<reflectivePoint.mSecondEdgeOpposite<<endl;
            //cout<<reflectivePoint.mPosition<<": First Point "<<firstPoint<<" and second point "<<secondPoint<<endl;

            reflectivePoint.mFirstEdgeExtendedStartCCW =
                    ExtendSegmentFromEnd( reflectivePoint.mPosition, reflectivePoint.mFirstEdgeStartCCW,
                                          mBoundingBoxDiameter, false);
            reflectivePoint.mSecondEdgeExtendedEndCCW =
                    ExtendSegmentFromEnd( reflectivePoint.mPosition, reflectivePoint.mSecondEdgeEndCCW,
                                          mBoundingBoxDiameter, false);
            mReflectivePoints[ vertexIndex ] = reflectivePoint;
        }
    }

}

void Engine::DetermineRobotWindowRanges( int robotIndex )
{
    //    cout<<"Determining ranges for robot "<<robotIndex<<endl;

    Robot& robot = mRobots[robotIndex];
    robot.mWindowRange.clear();
    for( map< int, ReflectivePoint >::iterator refit = mReflectivePoints.begin();
         refit != mReflectivePoints.end(); refit++)
    {
        ReflectivePoint reflectivePoint = refit->second;
        //        cout<<"reflective "<<reflectivePoint.mPosition;

        CGAL::Object firstIntersection, secondIntersection;
        IntersectionType firstIntersectionType, secondIntersectionType;

        firstIntersectionType = FindIntersection( robot.mPath,
                                                  Segment_2( reflectivePoint.mPosition,
                                                             reflectivePoint.mSecondEdgeOpposite ), firstIntersection );
        secondIntersectionType = FindIntersection( robot.mPath,
                                                   Segment_2( reflectivePoint.mPosition,
                                                              reflectivePoint.mFirstEdgeOpposite ), secondIntersection );
        //        cout<<"first inters "<<firstIntersectionType<<"sec inters "<<secondIntersectionType<<endl;
        // 0 for path end point means that point is placed at safe point, where window will not be created in that area.
        // -1 or 1 means the end point is placed in one of areas which a window can be created in those areas.
        int pathStartPartNumber = INT_MAX, pathEndPartNumber = INT_MAX;

        if( CheckInside( robot.mPath.source(), reflectivePoint.mFirstEdgeStartCCW, reflectivePoint.mPosition,
                         reflectivePoint.mSecondEdgeOpposite ) )
        {
            pathStartPartNumber = -1;
        }
        //else is removed for situation where start point is placed on border of first and safe area
        // in such situations safe area is better choice due to easier decision making in next steps.
        /*else*/ if( CheckInside( robot.mPath.source(), reflectivePoint.mSecondEdgeOpposite, reflectivePoint.mPosition,
                                  reflectivePoint.mFirstEdgeOpposite ) )
        {
            pathStartPartNumber = 0;
        }
        else if( CheckInside( robot.mPath.source(), reflectivePoint.mFirstEdgeOpposite, reflectivePoint.mPosition,
                              reflectivePoint.mSecondEdgeEndCCW ) )
        {
            pathStartPartNumber = 1;
        }

        if( CheckInside( robot.mPath.target(), reflectivePoint.mFirstEdgeStartCCW, reflectivePoint.mPosition,
                         reflectivePoint.mSecondEdgeOpposite ) )
        {
            pathEndPartNumber = -1;
        }
        //else is removed for situation where end point is placed on border of first and safe area
        // in such situations safe area is better choice due to easier decision making in next steps.
        /*else*/ if( CheckInside( robot.mPath.target(), reflectivePoint.mSecondEdgeOpposite, reflectivePoint.mPosition,
                                  reflectivePoint.mFirstEdgeOpposite ) )
        {
            pathEndPartNumber = 0;
        }
        else if( CheckInside( robot.mPath.target(), reflectivePoint.mFirstEdgeOpposite, reflectivePoint.mPosition,
                              reflectivePoint.mSecondEdgeEndCCW ) )
        {
            pathEndPartNumber = 1;
        }
        //        cout<< "Path start number: "<<pathStartPartNumber<<" pathend num: "<<pathEndPartNumber<<endl;

        if( pathStartPartNumber == 0 && pathEndPartNumber == 0)
        {
            //This reflective point is not important for us
            continue;
        }
        else if( pathEndPartNumber == pathStartPartNumber)
        {
            //This reflective point is important through all path
            ReflectiveToPathMapPoint firstRef, secondRef;
            firstRef.mPosition = robot.mPath.source();
            secondRef.mPosition = robot.mPath.target();
            pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment;
            segment.first = firstRef;
            segment.second = secondRef;

            robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                          ( reflectivePoint, segment ) );
        }
        else if( abs( pathStartPartNumber - pathEndPartNumber) == 1 )
        {
            //            cout<<"*************** 1\n";
            Point_2 start, end;
            if( pathStartPartNumber == -1 )
            {
                start = robot.mPath.source();
                if( !CGAL::assign( end, firstIntersection ) )
                {
                    cout<<"########################### 1: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                    exit(1);
                }
            }
            else if( pathStartPartNumber == 0)
            {
                end = robot.mPath.target();
                if( firstIntersectionType == IT_POINT )
                {
                    if( !CGAL::assign( start, firstIntersection ) )
                    {
                        cout<<"########################### 2: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else if( secondIntersectionType == IT_POINT )
                {
                    if( !CGAL::assign( start, secondIntersection ) )
                    {
                        cout<<"########################### 3: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    cout<<"########################### 4: A BUG FOUND! [IT = "<<firstIntersectionType<<
                          ", "<<secondIntersectionType<<"]"<<endl;
                    exit(1);
                }

            }
            else if( pathStartPartNumber == 1)
            {
                start = robot.mPath.source();
                if( !CGAL::assign( end, secondIntersection ) )
                {
                    cout<<"########################### 5: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                    exit(1);
                }
            }

            ReflectiveToPathMapPoint firstRef, secondRef;
            firstRef.mPosition = start;
            secondRef.mPosition = end;
            pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment;
            segment.first = firstRef;
            segment.second = secondRef;

            robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                          ( reflectivePoint, segment ) );
        }
        //
        else if( abs( pathStartPartNumber - pathEndPartNumber) == 2 )
        {
            //            cout<<"*************** 2\n";
            Point_2 start1, end1, start2, end2;
            if( pathStartPartNumber == -1 )
            {
                if( firstIntersectionType == IT_POINT )
                {
                    start1 = robot.mPath.source();
                    if( !CGAL::assign( end1, firstIntersection ) )
                    {
                        cout<<"########################### 6: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    cout<<"########################### 7: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                    exit(1);
                }
                if( secondIntersectionType == IT_POINT )
                {
                    end2 = robot.mPath.target();
                    if( !CGAL::assign( start2, secondIntersection ) )
                    {
                        cout<<"########################### 8: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    cout<<"########################### 9: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                    exit(1);
                }
            }
            else //if( pathStartPartNumber == 1 )
            {
                if( secondIntersectionType == IT_POINT )
                {
                    start1 = robot.mPath.source();

                    if( !CGAL::assign( end1, secondIntersection ) )
                    {
                        cout<<"########################### 10: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    cout<<"########################### 11: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                    exit(1);
                }
                if( firstIntersectionType == IT_POINT )
                {
                    end2 = robot.mPath.target();
                    if( !CGAL::assign( start2, firstIntersection ) )
                    {
                        cout<<"########################### 12: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    cout<<"########################### 13: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                    exit(1);
                }
            }

            ReflectiveToPathMapPoint firstRef1, secondRef1;
            firstRef1.mPosition = start1;
            secondRef1.mPosition = end1;
            pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment1;
            segment1.first = firstRef1;
            segment1.second = secondRef1;

            robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                          ( reflectivePoint, segment1 ) );

            ReflectiveToPathMapPoint firstRef2, secondRef2;
            firstRef2.mPosition = start2;
            secondRef2.mPosition = end2;
            pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment2;
            segment2.first = firstRef2;
            segment2.second = secondRef2;

            robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                          ( reflectivePoint, segment2 ) );
        }
        else //This means one of end points is placed at CW area of reflective point
        {
            //            cout<<"*************** max int\n";
            Point_2 start, end;
            CGAL::Object firstExtendedIntersection, secondExtendedIntersection;
            IntersectionType firstExtendedIntersectionType, secondExtendedIntersectionType;

            firstExtendedIntersectionType = FindIntersection( robot.mPath,
                                                              Segment_2( reflectivePoint.mPosition,
                                                                         reflectivePoint.mFirstEdgeExtendedStartCCW ),
                                                              firstExtendedIntersection );
            secondExtendedIntersectionType = FindIntersection( robot.mPath,
                                                               Segment_2( reflectivePoint.mPosition,
                                                                          reflectivePoint.mSecondEdgeExtendedEndCCW),
                                                               secondExtendedIntersection );
            if( pathEndPartNumber == INT_MAX)
            {// We supposed that environment has not any hole inside
                ///\todo: Change here to consider holes inside environment
                if( pathStartPartNumber == -1 )
                {
                    start = robot.mPath.source();
                    if( !CGAL::assign( end, firstExtendedIntersection ) )
                    {
                        cout<<"########################### 14: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else if( pathStartPartNumber == 0 )
                {
                    if( firstExtendedIntersectionType == IT_POINT )
                    {
                        if( !CGAL::assign( start, firstIntersection ) )
                        {
                            cout<<"########################### 15: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                        if( !CGAL::assign( end, firstExtendedIntersection ) )
                        {
                            cout<<"########################### 16: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                    }
                    else if( secondExtendedIntersectionType == IT_POINT )
                    {
                        if( !CGAL::assign( start, secondIntersection ) )
                        {
                            cout<<"########################### 17: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                        if( !CGAL::assign( end, secondExtendedIntersection ) )
                        {
                            cout<<"########################### 18: A BUG FOUND! [IT = "<<secondExtendedIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                    }
                    else
                    {
                        cout<<"########################### 19: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<
                              ", "<<secondExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    start = robot.mPath.source();
                    if( !CGAL::assign( end, secondExtendedIntersection ) )
                    {
                        cout<<"########################### 20: A BUG FOUND! [IT = "<<secondExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                ReflectiveToPathMapPoint firstRef, secondRef;
                firstRef.mPosition = start;
                secondRef.mPosition = end;
                pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment;
                segment.first = firstRef;
                segment.second = secondRef;

                robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                              ( reflectivePoint, segment ) );
            }
            else
            {
                if( pathEndPartNumber == -1 )
                {
                    end = robot.mPath.target();
                    if( !CGAL::assign( start, firstExtendedIntersection ) )
                    {
                        cout<<"########################### 21: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else if( pathEndPartNumber == 0 )
                {
                    if( firstExtendedIntersectionType == IT_POINT )
                    {
                        if( !CGAL::assign( end, firstIntersection ) )
                        {
                            cout<<"########################### 22: A BUG FOUND! [IT = "<<firstIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                        if( !CGAL::assign( start, firstExtendedIntersection ) )
                        {
                            cout<<"########################### 23: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                    }
                    else if( secondExtendedIntersectionType == IT_POINT )
                    {
                        if( !CGAL::assign( end, secondIntersection ) )
                        {
                            cout<<"########################### 24: A BUG FOUND! [IT = "<<secondIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                        if( !CGAL::assign( start, secondExtendedIntersection ) )
                        {
                            cout<<"########################### 25: A BUG FOUND! [IT = "<<secondExtendedIntersectionType<<"]"<<endl;
                            exit(1);
                        }
                    }
                    else
                    {
                        cout<<"########################### 26: A BUG FOUND! [IT = "<<firstExtendedIntersectionType<<
                              ", "<<secondExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                else
                {
                    end = robot.mPath.target();
                    if( !CGAL::assign( start, secondExtendedIntersection ) )
                    {
                        cout<<"########################### 27: A BUG FOUND! [IT = "<<secondExtendedIntersectionType<<"]"<<endl;
                        exit(1);
                    }
                }
                ReflectiveToPathMapPoint firstRef, secondRef;
                firstRef.mPosition = start;
                secondRef.mPosition = end;
                pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > segment;
                segment.first = firstRef;
                segment.second = secondRef;

                robot.mWindowRange.push_back( pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > >
                                              ( reflectivePoint, segment ) );
            }

        }

    }
    //    vector< pair< ReflectivePoint, pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > > >::iterator rangit;
    //    cout<<"_________________________________\n";
    //    for( rangit = robot.mWindowRange.begin(); rangit != robot.mWindowRange.end(); rangit++)
    //    {

    //        cout << "Robot "<<robotIndex<<" and reflective "<<rangit->first.mPosition<<endl;

    //        pair< ReflectiveToPathMapPoint, ReflectiveToPathMapPoint > seg = rangit->second;
    //        cout<<"Segment "<<seg.first.mPosition<<", "<<seg.second.mPosition<<endl;
    //        cout<<"---------------------------------\n";
    //    }

}

void Engine::WriteSummaryToFile ()
{
    ofstream myfile;
    myfile.open ("summary.txt", ios_base::app);
    myfile << "Found "<<StateManager::GetInstance().GetStatesSize()<<" new states from "<<
              mTotalStates<<" states in "<<time(0) - mStartTime<<" seconds.( "<<
              (float)StateManager::GetInstance().GetStatesSize()/(float)mTotalStates<<
              " )"<<endl;

    myfile.close();
}
