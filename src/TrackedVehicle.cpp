//
//  TrackedVehicle.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "TrackedVehicle.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <drawstuff/drawstuff.h>
#include "ODEUtils.h"

TrackedVehicle::TrackedVehicle(Environment *environment_, const std::string &name_) : Vehicle(environment_, name_) {
    wheelRadius = 0.078;
    wheelBase = 0.4997;
    trackWidth = 0.097;
    vehicleBodyWidth = 0.254;
    const size_t numGrousers = 30;
    const dReal linkThickness = 0.01;
    const dReal grouserHeight = 0.015;
    trackVehicleSpace = 0.024;
    flipperRadius = 0.016;
    flipperBase = 0.32;
    flipperWidth = 0.05;

    this->density = 1.4;
    this->width = vehicleBodyWidth;

    this->leftTrack = new Track(this->environment, name + ".leftTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, -1);
    this->rightTrack = new Track(this->environment, name + ".rightTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, 1);
    this->leftTrack->prepareFlippers(wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth);
    this->rightTrack->prepareFlippers(wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth);
}

TrackedVehicle::~TrackedVehicle() {
    delete this->leftTrack;
    delete this->rightTrack;
}

void TrackedVehicle::create() {
    this->vehicleBody = dBodyCreate(this->environment->world);
    this->vehicleGeom = dCreateBox(this->environment->space, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    this->environment->setGeomName(this->vehicleGeom, name + ".vehicleGeom");
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    //dMassAdjust(&this->vehicleMass, 2.40);
    dGeomSetCategoryBits(this->vehicleGeom, Category::OBSTACLE);
    dGeomSetCollideBits(this->vehicleGeom, Category::OBSTACLE | Category::TERRAIN);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    dGeomSetOffsetPosition(this->vehicleGeom, 0, 0, this->leftTrack->m->radius[0]);

    this->leftTrack->create();
    this->rightTrack->create();

    dReal w = this->width + 2*trackWidth + 2 * trackVehicleSpace;
    dRigidBodyArraySetPosition(leftTrack->bodyArray,  -wheelBase/2, -(w - trackWidth)/2, 0);
    dRigidBodyArraySetPosition(rightTrack->bodyArray, -wheelBase/2,  (w - trackWidth)/2, 0);

    this->leftTrackJoint = dJointCreateFixed(this->environment->world, 0);
    this->rightTrackJoint = dJointCreateFixed(this->environment->world, 0);
    dJointAttach(this->leftTrackJoint, this->vehicleBody, this->leftTrack->trackBody);
    dJointAttach(this->rightTrackJoint, this->vehicleBody, this->rightTrack->trackBody);
    dJointSetFixed(this->leftTrackJoint);
    dJointSetFixed(this->rightTrackJoint);

    this->bodyArray = dRigidBodyArrayCreate(this->vehicleBody);
    dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->bodyArray);
    dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->bodyArray);
}

void TrackedVehicle::destroy() {
    this->leftTrack->destroy();
    dJointDestroy(this->leftTrackJoint);
    this->rightTrack->destroy();
    dJointDestroy(this->rightTrackJoint);

    dBodyDestroy(this->vehicleBody);
    dGeomDestroy(this->vehicleGeom);
    
    dRigidBodyArrayDestroy(this->bodyArray);
}

void TrackedVehicle::step(dReal stepSize) {
    this->leftTrack->step(stepSize);
    this->rightTrack->step(stepSize);
}

void TrackedVehicle::draw() {
    {
        dsSetColor(0, 0, 1);
        const dReal *pos = dGeomGetPosition(this->vehicleGeom);
        const dReal *R = dGeomGetRotation(this->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(this->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }

    this->leftTrack->draw();
    this->rightTrack->draw();
}

void TrackedVehicle::setVelocities(dReal a, dReal b) {
    setTrackVelocities(a, b);
}

void TrackedVehicle::setTrackVelocities(dReal left, dReal right) {
    this->leftTrack->setVelocity(left);
    this->rightTrack->setVelocity(right);
}

const dReal * TrackedVehicle::getPosition() {
    return dBodyGetPosition(this->vehicleBody);
}

const dReal * TrackedVehicle::getLinearVel() {
    return dBodyGetLinearVel(this->vehicleBody);
}

const dReal * TrackedVehicle::getAngularVel() {
    return dBodyGetAngularVel(this->vehicleBody);
}

const dReal * TrackedVehicle::getQuaternion() {
    return dBodyGetQuaternion(this->vehicleBody);
}

const dReal * TrackedVehicle::getRotation() {
    return dBodyGetRotation(this->vehicleBody);
}

void TrackedVehicle::setPosition(const dReal *p) {
    dRigidBodyArraySetPosition(this->bodyArray, p[0], p[1], p[2]);
}

void TrackedVehicle::setVel(const dReal *linear, const dReal *angular) {
    dRigidBodyArraySetVel(this->bodyArray, linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]);
}

void TrackedVehicle::setQuaternion(const dReal *q) {
    dRigidBodyArraySetQuaternion(this->bodyArray, q);
}

void TrackedVehicle::setRotation(const dReal *R) {
    dRigidBodyArraySetRotation(this->bodyArray, R);
}
