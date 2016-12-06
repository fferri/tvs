//
// Created by peci1 on 11.1.16.
//

#ifndef TRACKED_MOTION_SIMPLE_FLIPPER_H
#define TRACKED_MOTION_SIMPLE_FLIPPER_H

#include "SimpleTrackBase.h"

class Environment;

class SimpleFlipper : public SimpleTrackBase {
public:
    LinVelProfInt velocity;

    SimpleFlipper(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, dReal trackDepth_, unsigned long additionalCategory);
    virtual ~SimpleFlipper();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocity(dReal velocity);
    virtual dReal getVelocity();

protected:
    virtual unsigned long getGrouserCategory();
};


#endif //TRACKED_MOTION_SIMPLE_FLIPPER_H
