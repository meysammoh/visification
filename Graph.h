#ifndef _GRAPH_H
#define _GRAPH_H
#include "helper.h"

using namespace std;

struct GraphNode
{
    ~GraphNode();
    vector< int > mNeighbors;
    vector< pair< int, int > > mCellInfo;
    void Draw();
};

class Graph
{
    GraphNode* mNodes;
    int mSize = 0;
public:
    Graph( int size );
    ~Graph();
    void Draw();
    void AddEdge( int from, int to );
    //bool operator==( const Graph& rhs );
    bool IsEqual( Graph* graph);
    int GetSize() const;
    GraphNode* GetNodes() const;
};

#endif
