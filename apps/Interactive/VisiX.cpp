#include "VisiX.h"


#include <fstream>
#include <streambuf>



VisiX::VisiX()
{

}

VisiX& VisiX::GetInstance()
{
    static VisiX instance;
    return instance;
}

bool VisiX::Init( string worldDataPath )
{
    bool result = true;
    string worldData = "";
    if( worldDataPath == "" )
    {

        worldData = "Environment\n0 4\n0 0\n3 2\n4 0\n4 4\n1 2\n";
        worldData += "Robots\n2 1.5 3.9 3.7\n2 2.5 .1 .1\n.5 2.25 1 1\n";
        cout<<"Loading a defaul world ...\n"<<worldData<<endl;
    }
    else
    {
        ifstream worldFile( worldDataPath );
        worldData = string( ( istreambuf_iterator<char>( worldFile ) ),
                            istreambuf_iterator<char>() );
    }

    result = mVisifier.Initialize( worldData );
    mWinDiameter = mVisifier.mBoundingBoxDiameter;
    mWinCenterX = mVisifier.mBoundingBoxCenter.first;
    mWinCenterY = mVisifier.mBoundingBoxCenter.second;
    return result;
}

void VisiX::Draw()
{
    mVisifier.Draw();
}
