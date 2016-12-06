//
//  Track.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACK_H_INCLUDED
#define TRACK_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "TrackBase.h"
#include "TrackKinematicModel.h"
#include "ODEUtils.h"
#include "Flipper.h"

#define NUM_FLIPPERS 2
#define NUM_TRACK_STRUT_GEOMS 1

class Environment;

class Track : public TrackBase {
public:
    LinVelProfInt velocity;

    Track(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_, int yDirection);
    virtual ~Track();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocity(dReal velocity);
    void prepareFlippers(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_,
                             dReal grouserHeight_, dReal flipperWidth_);
    void setFlipperAngularVelocity(size_t flipperNumber, dReal velocity);

protected:
    dGeomID strutGeoms[NUM_TRACK_STRUT_GEOMS];
    Flipper* flippers[NUM_FLIPPERS];
    dJointID flipperJoints[NUM_FLIPPERS];
    dJointID flipperMotors[NUM_FLIPPERS];
    int yDirection;


    Category::Category getWheelCategory();
    Category::Category getGuideCategory();
    Category::Category getGrouserCategory();
};

#endif // TRACK_H_INCLUDED
