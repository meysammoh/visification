#include "VisiX.h"
#include "Config.h"
#include "State.h"
#include "Graphics.h"

//=========================Main=========================//
int main(int argc, char *argv[])
{
    srand( time(0) );
    string dataPath = "";
    if(argc > 1)
        dataPath = argv[1] ;
    bool result = VisiX::GetInstance().Init( dataPath );
    if(result)
    {
        //Engine::GetInstance().StartWorking();
        Graphics::GetInstance().Initialize( argc, argv );


    }
    return 0;
}
