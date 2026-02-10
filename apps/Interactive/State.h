#ifndef _STATE_H
#define _STATE_H
#include "EventPointBase.h"
#include "Graph.h"
#include "Robot.h"

class State
{
private:
    //map< int, State* > mNextStates;

    int mID = -1;
public:
    State();
//    ~State();
//    Graph* mDualGraph = nullptr;
//    map< int, vector<EventPointBase> >  mRobotsForward, mRobotsBackward;
//    void AddNextState(State* state);
//    bool AreSequencesEqual( State* state );

//    int GetID() const;
//    void ClearExtraData();
};

class StateManager
{
//private:
//    StateManager(){}
//    map< int, State*> mStates;
//    State* mWorkingState = nullptr;
//    State* mFirstOfPace = nullptr;
//    State *SaveSnapshot();
//    vector<Robot> mFirstOfPaceRobots;
//public:
//    static StateManager& GetInstance();

//    State* FindEquivalentState( State* currentState );
//    bool SaveState( bool isFirstOfPace );
//    State* GetWorkingState(){ return mWorkingState; }
//    void SetFirstOfPhaseToWorkingState(  );
//    void RestoreFirstOfPace();
//    int GetStatesSize() {
//        return  mStates.size();
//         }
};

#endif
