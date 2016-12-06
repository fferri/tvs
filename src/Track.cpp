//
//  Track.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Track.h"
#include "Environment.h"
#include <drawstuff/drawstuff.h>

Track::Track(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_, int yDirection) : TrackBase(environment_, name_, 0, radius1_, radius2_, distance_, numGrousers_, linkThickness_, grouserHeight_, trackDepth_), yDirection(yDirection) {
    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i] = NULL;
        flipperJoints[i] = NULL;
    }
}

Track::~Track() {
    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        if (flippers[i] != NULL) {
            flippers[i]->destroy();
        }
        if (flipperJoints[i] != NULL) {
            dJointDestroy(flipperJoints[i]);
        }
    }
}

void Track::prepareFlippers(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal flipperWidth_) {
    if (NUM_FLIPPERS > 0) {
        this->flippers[0] = new Flipper(this->environment, name + ".frontFlipper",  radius1_, radius2_, distance_, numGrousers_, linkThickness_, grouserHeight_, flipperWidth_);
    }

    if (NUM_FLIPPERS > 1) {
        this->flippers[1] = new Flipper(this->environment, name + ".rearFlipper",  radius1_, radius2_, distance_, numGrousers_, linkThickness_, grouserHeight_, flipperWidth_);
    }
}

void Track::create() {
    TrackBase::create();

#if NUM_TRACK_STRUT_GEOMS >= 1
    this->strutGeoms[0] = dCreateBox(this->environment->space, 0.9*this->m->distance, 0.95*this->m->trackDepth, 1.95*this->m->radius[0]);
    this->environment->setGeomName(this->strutGeoms[0], this->name + ".strut");
    dGeomSetCategoryBits(this->strutGeoms[0], Category::TRACK_GUIDE);
    dGeomSetCollideBits(this->strutGeoms[0], Category::TRACK_GROUSER);
    dGeomSetBody(this->strutGeoms[0], this->trackBody);
    dGeomSetOffsetPosition(this->strutGeoms[0], 0.5 * this->m->distance, 0, 0.0);
#endif


    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->create();

        // origin of the track is in the front wheel
        dRigidBodyArraySetPosition(flippers[i]->bodyArray, i * this->m->distance, yDirection * (this->m->trackDepth / 2.0 + flippers[i]->m->trackDepth / 2 + 0.005), 0);

        flipperJoints[i] = dJointCreateHinge(this->environment->world, 0);
        dJointAttach(flipperJoints[i], trackBody, flippers[i]->trackBody);
        dJointSetHingeAnchor(flipperJoints[i], i * this->m->distance, yDirection * (this->m->trackDepth / 2 + flippers[i]->m->trackDepth / 2), 0);
        dJointSetHingeAxis(flipperJoints[i], 0, 1, 0);

        flipperMotors[i] = dJointCreateAMotor(this->environment->world, 0);
        dJointSetAMotorMode(flipperMotors[i], dAMotorUser);
        dJointSetAMotorNumAxes(flipperMotors[i], 1);
        dJointAttach(flipperMotors[i], trackBody, flippers[i]->trackBody);
        dJointSetAMotorAxis(flipperMotors[i], 0, 1, 0, 1, 0);
        dJointSetAMotorParam(flipperMotors[i], dParamFMax, 10);

        dRigidBodyArrayAdd(bodyArray, flippers[i]->bodyArray);
    }

    if (NUM_FLIPPERS > 0) {
        // rotate the front flipper by 180 degrees
        dMatrix3 rot;
        dRFromAxisAndAngle(rot, 0, 1, 0, M_PI);
        dRigidBodyArraySetRotationRelative(this->flippers[0]->bodyArray, rot);
    }
}

void Track::destroy() {
    TrackBase::destroy();

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->destroy();
        flippers[i] = NULL;

        dJointDestroy(flipperJoints[i]);
        flipperJoints[i] = NULL;
    }
}

void Track::step(dReal stepSize) {
    this->velocity.step(stepSize);
    dJointSetHingeParam(this->wheelJoint[this->drivingWheelIndex], dParamVel, this->velocity.get());

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->step(stepSize);
    }
}

void Track::draw() {
    TrackBase::draw();

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->draw();
    }

    dsSetColorAlpha(0, 0, 0, 1);
    for(int w = 0; w < NUM_TRACK_STRUT_GEOMS; w++) {
        const dReal *pos = dGeomGetPosition(this->strutGeoms[w]);
        const dReal *R = dGeomGetRotation(this->strutGeoms[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->strutGeoms[w], sides);
        dsDrawBoxD(pos, R, sides);
    }

#if USE_GUIDE_GEOMS
#define DEBUG_DRAW_GROUSER_GUIDES
#ifdef DEBUG_DRAW_GROUSER_GUIDES
    dsSetColorAlpha(0, 1, 0, 0.3);
    for(int w = 0; w < NUM_GUIDE_GEOMS; w++) {
        const dReal *pos = dGeomGetPosition(this->guideGeom[w]);
        const dReal *R = dGeomGetRotation(this->guideGeom[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->guideGeom[w], sides);
        dsDrawBoxD(pos, R, sides);
    }
#endif
#endif
}

void Track::setVelocity(dReal velocity) {
    velocity = fmin(1.0, fmax(-1.0, velocity)) * this->environment->config.world.max_track_speed;

    this->velocity.set(velocity);

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->setVelocity(velocity);
    }
}

void Track::setFlipperAngularVelocity(size_t flipperNumber, dReal velocity) {
    if (flipperNumber < NUM_FLIPPERS) {
        if (flipperNumber == 1)
            velocity = -velocity;
        dJointSetAMotorParam(flipperMotors[flipperNumber], dParamVel, velocity);
    }
}

Category::Category Track::getWheelCategory() {
    return Category::TRACK_WHEEL;
}

Category::Category Track::getGuideCategory() {
    return Category::TRACK_GUIDE;
}

Category::Category Track::getGrouserCategory() {
    return Category::TRACK_GROUSER;
}
