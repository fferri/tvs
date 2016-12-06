//
// Created by peci1 on 11.1.16.
//

#ifndef TRACKED_MOTION_FLIPPER_H
#define TRACKED_MOTION_FLIPPER_H

#include "TrackBase.h"

#define NUM_FLIPPER_STRUT_GEOMS 2

class Flipper : public TrackBase {
public:
    LinVelProfInt velocity;

    Flipper(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_);
    virtual ~Flipper();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocity(dReal velocity);


protected:
    dGeomID strutGeoms[NUM_FLIPPER_STRUT_GEOMS];

    virtual Category::Category getWheelCategory();
    virtual Category::Category getGuideCategory();
    virtual Category::Category getGrouserCategory();
};


#endif //TRACKED_MOTION_FLIPPER_H
