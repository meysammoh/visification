#ifndef _VISIX_H
#define _VISIX_H
#include <string>
#include "XVisifier.h"
#include "Drawable.hpp"

using namespace std;
/**
 * @brief The VisiX class uses Visification library to graphically show analysis of visibility properties of an environment.
 *
 */
class VisiX : Drawable
{
private:

    VisiX();
    XVisifier mVisifier;
    float mWinDiameter = 100;
    float mWinCenterX = 0;
    float mWinCenterY = 0;
public:

    bool Init( string worldDataPath );
    static VisiX& GetInstance();
    void Draw();
    float GetWindowDiameter(){ return mWinDiameter;}
    float GetWindowCenterX(){ return mWinCenterX;}
    float GetWindowCenterY(){ return mWinCenterY;}
};


#endif
