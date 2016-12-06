//
//  SimpleTrack.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "SimpleTrack.h"
#include "Environment.h"
#include <drawstuff/drawstuff.h>

SimpleTrack::SimpleTrack(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, dReal trackDepth_, int yDirection, unsigned long additionalCategory) : SimpleTrackBase(environment_, name_, radius1_, radius2_, distance_, trackDepth_, additionalCategory), yDirection(yDirection) {
    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i] = NULL;
        flipperJoints[i] = NULL;
    }
}

SimpleTrack::~SimpleTrack() {
    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        if (flippers[i] != NULL) {
            flippers[i]->destroy();
        }
        if (flipperJoints[i] != NULL) {
            dJointDestroy(flipperJoints[i]);
        }
    }
}

void SimpleTrack::prepareFlippers(dReal radius1_, dReal radius2_, dReal distance_, dReal flipperWidth_) {
    if (NUM_FLIPPERS > 0) {
        this->flippers[0] = new SimpleFlipper(this->environment, name + ".frontFlipper", radius1_, radius2_, distance_, flipperWidth_, this->additionalCategory | Category::FRONT);
    }

    if (NUM_FLIPPERS > 1) {
        this->flippers[1] = new SimpleFlipper(this->environment, name + ".rearFlipper", radius1_, radius2_, distance_, flipperWidth_, this->additionalCategory | Category::REAR);
    }
}

void SimpleTrack::create() {
    SimpleTrackBase::create();

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->create();

        // origin of the track is in the front wheel
        dRigidBodyArraySetPosition(flippers[i]->bodyArray, i * this->betweenWheelsDistance, yDirection * (this->trackDepth / 2.0 + flippers[i]->trackDepth / 2 + 0.005), 0);

        flipperJoints[i] = dJointCreateHinge(this->environment->world, 0);
        dJointAttach(flipperJoints[i], trackBody, flippers[i]->trackBody);
        dJointSetHingeAnchor(flipperJoints[i], i * this->betweenWheelsDistance, yDirection * (this->trackDepth / 2 + flippers[i]->trackDepth / 2), 0);
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

void SimpleTrack::destroy() {
    SimpleTrackBase::destroy();

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->destroy();
        flippers[i] = NULL;

        dJointDestroy(flipperJoints[i]);
        flipperJoints[i] = NULL;
    }
}

void SimpleTrack::step(dReal stepSize) {
    this->velocity.step(stepSize);

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->step(stepSize);
    }
}

void SimpleTrack::draw() {
    SimpleTrackBase::draw();

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->draw();
    }
}

void SimpleTrack::setVelocity(dReal velocity) {
    velocity = fmin(1.0, fmax(-1.0, velocity)) * this->environment->config.world.max_track_speed;

    this->velocity.set(velocity);

    for (size_t i=0; i < NUM_FLIPPERS; i++) {
        flippers[i]->setVelocity(velocity);
    }
}

void SimpleTrack::setFlipperAngularVelocity(size_t flipperNumber, dReal velocity) {
    if (flipperNumber < NUM_FLIPPERS) {
        if (flipperNumber == 1)
            velocity = -velocity;
        dJointSetAMotorParam(flipperMotors[flipperNumber], dParamVel, velocity);
    }
}

unsigned long SimpleTrack::getGrouserCategory() {
    return Category::TRACK_GROUSER | Category::TRACK | this->additionalCategory;
}

dReal SimpleTrack::getVelocity() {
    return this->velocity.get();
}
