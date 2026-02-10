#include "Graph.h"

GraphNode::~GraphNode()
{
    mNeighbors.clear();
    mCellInfo.clear();
}

void GraphNode::Draw()
{

}

void Graph::Draw()
{
}
Graph::~Graph()
{
    delete [] mNodes;
}

int Graph::GetSize() const
{
    return mSize;
}

Graph::Graph( int size)
{
    mSize = size;
    mNodes = new GraphNode[size];
}

void Graph::AddEdge( int from, int to )
{
    //cout<<"Graph:: Adding edge between "<<from <<" and "<<to<<endl;
    mNodes[from].mNeighbors.push_back(to);
    mNodes[to].mNeighbors.push_back(from);
}

GraphNode* Graph::GetNodes() const
{
    return mNodes;
}
bool Graph::IsEqual( Graph* graph)
{

    if( mSize != graph->GetSize())
        return false;
    bool result = true;
    for( int nodeIndex = 0; nodeIndex < mSize; nodeIndex++ )
    {
        //        if( mNodes[nodeIndex].mNeighbors.size() != (rhs.GetNodes())[nodeIndex].mNeighbors.size())
        //            return false;
        //        for(int neighborsIndex = 0; neighborsIndex < mNodes[nodeIndex].mNeighbors.size(); neighborsIndex++)
        //            if(mNodes[nodeIndex].mNeighbors[neighborsIndex] != (rhs.GetNodes())[nodeIndex].mNeighbors[neighborsIndex])
        //                return false;
        GraphNode leftRoot, rightRoot;
        bool equivalentFound = false;
        for( int nodeIndexRhs = 0; nodeIndexRhs < mSize; nodeIndexRhs++ )
        {
            if( mNodes[nodeIndex].mCellInfo.size() != graph->GetNodes()[nodeIndexRhs].mCellInfo.size() )
            {
                continue;
            }
            else
            {

                bool edgesFound = true;
                for( vector< pair<int,int> >::iterator infoit = mNodes[nodeIndex].mCellInfo.begin();
                     infoit != mNodes[nodeIndex].mCellInfo.end(); infoit++)
                {
                    vector< pair<int,int> >::iterator searchRes =
                            find( graph->GetNodes()[nodeIndexRhs].mCellInfo.begin(),
                                  graph->GetNodes()[nodeIndexRhs].mCellInfo.end(), *infoit);
                    if( searchRes == graph->GetNodes()[nodeIndexRhs].mCellInfo.end() )
                    {
                        edgesFound = false;
                        break;
                    }
                }
                if( !edgesFound )
                {
                    continue;
                }
                leftRoot = mNodes[nodeIndex];
                rightRoot = graph->GetNodes()[nodeIndexRhs];
                equivalentFound = true;
                break;
            }
        }
        if(!equivalentFound)
        {
            result = false;
        }

    }
    return result;
}
