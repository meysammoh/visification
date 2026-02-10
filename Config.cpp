#include "Config.h"

Config::Config()
{

}

Config& Config::GetInstance()
{
    static Config instance;
    return instance;
}
