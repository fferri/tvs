//
//  Track.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef SIMPLE_TRACK_H_INCLUDED
#define SIMPLE_TRACK_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "TrackKinematicModel.h"
#include "ODEUtils.h"
#include "SimpleFlipper.h"
#include "SimpleTrackBase.h"

#define NUM_FLIPPERS 2

class Environment;

class SimpleTrack : public SimpleTrackBase {
public:
    SimpleTrack(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, dReal trackDepth_, int yDirection, unsigned long additionalCategory);
    virtual ~SimpleTrack();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocity(dReal velocity);
    void prepareFlippers(dReal radius1_, dReal radius2_, dReal distance_, dReal flipperWidth_);
    void setFlipperAngularVelocity(size_t flipperNumber, dReal velocity);
    virtual dReal getVelocity();

    LinVelProfInt velocity;

protected:
    SimpleFlipper* flippers[NUM_FLIPPERS];
    dJointID flipperJoints[NUM_FLIPPERS];
    dJointID flipperMotors[NUM_FLIPPERS];

    int yDirection;

    unsigned long getGrouserCategory();
};

#endif // SIMPLE_TRACK_H_INCLUDED
