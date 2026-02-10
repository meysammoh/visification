#include "State.h"
#include "Engine.h"


int State::GetID() const
{
    return mID;
}

State::State()
{
    static int id = 0;
    mID = id++;
    //cout<<"Id "<<mID<<" generated for state.\n ";
}
State::~State()
{
    delete mDualGraph;
    mDualGraph = nullptr;
    //mNextStates.clear();
    mRobotsForward.clear();
    mRobotsBackward.clear();
}

void State::AddNextState( State* state )
{
    //mNextStates[state->GetID()] = state;
}

StateManager& StateManager::GetInstance()
{
    static StateManager instance;
    return instance;
}

State* StateManager::SaveSnapshot()
{
    State* currentState = new State();
    currentState->mDualGraph = Engine::GetInstance().mDualGraph;
    for( int robotIndex = 0; robotIndex < Engine::GetInstance().mRobots.size(); robotIndex++)
    {
        currentState->mRobotsForward[robotIndex].insert( currentState->mRobotsForward[robotIndex].end(),
                Engine::GetInstance().mRobots[robotIndex].mForwardEventPoints.begin(),
                                                         Engine::GetInstance().mRobots[robotIndex].mForwardEventPoints.end());
        currentState->mRobotsBackward[robotIndex].insert( currentState->mRobotsBackward[robotIndex].end(),
                                                          Engine::GetInstance().mRobots[robotIndex].mBackwardEventPoints.begin(),
                                                                                                   Engine::GetInstance().mRobots[robotIndex].mBackwardEventPoints.end());
    }

    return currentState;
}

//void StateManager::SaveInitialState()
//{
//    State* currentState = SaveSnapshot();
//    mStates.push_back(currentState);
//    mWorkingState = currentState;
//    //cout << "Saved first state."<<endl;
//}

void StateManager::RestoreFirstOfPace()
{

    //Engine::GetInstance().mRobots = mWorkingState->mRobots;
    for( int robotIndex = 0; robotIndex < mFirstOfPaceRobots.size(); robotIndex++ )
    {
        Engine::GetInstance().mRobots[robotIndex].MoveTo( mFirstOfPaceRobots[robotIndex].GetPosition() );
        Engine::GetInstance().mRobots[robotIndex].mWindows = mFirstOfPaceRobots[robotIndex].mWindows;
        Engine::GetInstance().mRobots[robotIndex].mForwardEventPoints = mFirstOfPaceRobots[robotIndex].mForwardEventPoints;
        Engine::GetInstance().mRobots[robotIndex].mBackwardEventPoints = mFirstOfPaceRobots[robotIndex].mBackwardEventPoints;
    }
    //Engine::GetInstance().mRemainingRobotsIndex = mWorkingState->mRemainingRobots;
    //cout<<"Restored working state."<<endl;
}

bool State::AreSequencesEqual( State* state )
{
    bool result = true;
    for( int robotIndex = 0; robotIndex < mRobotsForward.size(); robotIndex++ )
    {

        vector<EventPointBase>& curForward = mRobotsForward[robotIndex];
        vector<EventPointBase>& curBackward= mRobotsBackward[robotIndex];
        vector<EventPointBase>& prevForward= state->mRobotsForward[robotIndex];
        vector<EventPointBase>& prevBackward= state->mRobotsBackward[robotIndex];
        if( curForward.size() != prevForward.size() || curBackward.size() != prevBackward.size() )
        {
            result = false;
            break;
        }
    }
    if( result )
    {
        int robotsSize = mRobotsForward.size();
        for( int robotIndex = 0; robotIndex < robotsSize; robotIndex++ )
        {
            vector<EventPointBase>& curForward = mRobotsForward[robotIndex];
            vector<EventPointBase>& curBackward= mRobotsBackward[robotIndex];
            vector<EventPointBase>& prevForward= state->mRobotsForward[robotIndex];
            vector<EventPointBase>& prevBackward= state->mRobotsBackward[robotIndex];

            //Check Forward sequences
            for( int forwardIndex = 0; forwardIndex < curForward.size(); forwardIndex++ )
            {
                if( curForward[forwardIndex] != prevForward[forwardIndex] )
                {
                    result = false;
                    break;
                }
            }
            if( !result )
            {
                break;
            }
            //Check Backward sequences
            for( int backwardIndex = 0; backwardIndex < curBackward.size(); backwardIndex++ )
            {
                EventPointBase re1 = curBackward[backwardIndex];
                EventPointBase re2 = prevBackward[backwardIndex];
                if( re1 != re2 )
                {
                    result = false;
                    break;
                }
            }
            if( !result )
            {
                break;
            }

        }
    }
    return result;
}

State* StateManager::FindEquivalentState( State* currentState )
{
    State* equivalent = nullptr;
    for(map< int, State* >::iterator statitor = mStates.begin();
        statitor != mStates.end(); statitor++ )
    {
        //Narenji
        if( currentState->mDualGraph->IsEqual( (statitor->second)->mDualGraph  )
                && currentState->AreSequencesEqual( statitor->second ) )
        {
            //cout<<"Found Equivalent ... "<<endl;
            equivalent = statitor->second;
            break;
        }
    }
    return equivalent;
}

bool StateManager::SaveState( bool isFirstOfPace )
{
    bool result = true; //True if state is unique

    State* currentState = SaveSnapshot();
    State* searchResult = FindEquivalentState( currentState );
    if( searchResult == nullptr )
    {
        //Found New State
        //cout<< "Found New State ... "<<endl;
        if( mWorkingState != nullptr )
        {
            mWorkingState->AddNextState( currentState );
        }
        mStates[currentState->GetID()] = currentState;
        mWorkingState = currentState;
    }
    else
    {
        //cout<< "Current state is not new! "<<endl;
        mWorkingState->AddNextState( searchResult );
        mWorkingState = searchResult;
        result = false;
        delete currentState;
    }
    if( isFirstOfPace )
    {
        mFirstOfPace = mWorkingState;
        mFirstOfPaceRobots = Engine::GetInstance().mRobots;
    }
    return result;
}

void StateManager::SetFirstOfPhaseToWorkingState(  )
{
    mFirstOfPace = mWorkingState;
    mFirstOfPaceRobots = Engine::GetInstance().mRobots;
}


void State::ClearExtraData()
{
//    for( int robotIndex = 0; robotIndex < mRobots.size(); robotIndex++)
//    {
//        mRobots[robotIndex].ClearExtraData();
//    }
}

