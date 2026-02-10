#include "EventPoint.h"


vector<pair<int, int> > EventPointBase::GetEdges() const
{
    return mEdges;
}

void EventPointBase::PushEdge( pair< int, int > edge )
{
    vector< pair< int, int> >::iterator edgit = mEdges.begin();
    bool isDuplicate = false;
    for( ; edgit != mEdges.end() ;  )
    {
        if( edgit->first > edge.first )
        {
            break;
        }
        else if( edgit->first == edge.first )
        {
            if( edgit->second > edge.second )
            {
                break;
            }
            else if( edgit->second == edge.second )
            {
                isDuplicate = true;
                break;
            }
            else
            {
                edgit++;
            }
        }
        else
        {
            edgit++;
        }

    }
    if( ! isDuplicate )
    {
        mEdges.insert( edgit, edge );
    }
}

bool EventPointBase::ContainsWindow( pair< int, int> window )
{
    vector< pair< int, int> >::iterator it = find( mEdges.begin(), mEdges.end(), window);
    return it != mEdges.end();

}

bool EventPointBase::operator==( const EventPointBase& rhs )
{
    bool result = true;//mEdges.size() == rhs.mEdges.size() ;
//    if( result )
//    {
        vector< pair< int, int> >::iterator edgit = mEdges.begin();
        for( ; edgit != mEdges.end() ; edgit++ )
        {
            vector< pair< int, int> >::const_iterator searchRes =
                    find( rhs.mEdges.begin(), rhs.mEdges.end(), *edgit );
            if( searchRes == rhs.mEdges.end() )
            {
                result = false;
                break;
            }
        }

//    }
    return result;
}

bool EventPointBase::operator!=( const EventPointBase& rhs )
{
    return !( *this == rhs );
}

RobotEventPoint::RobotEventPoint( EventPoint& eventPoint ) : EventPoint( eventPoint )
{
//    mAngle = 0;
//    mSquaredDistance = 0;
}

