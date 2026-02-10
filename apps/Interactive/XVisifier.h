#ifndef _X_VISIFIER_H
#define _X_VISIFIER_H
#include "Visifier.h"
#include "Drawable.hpp"

class XVisifier : public Visifier, public Drawable
{
private:


public:
    XVisifier();
    virtual void Draw();
    void StartWorking();
};


#endif
