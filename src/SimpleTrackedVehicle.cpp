//
//  SimpleTrackedVehicle.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <drawstuff/drawstuff.h>
#include "SimpleTrackedVehicle.h"
#include "ODEUtils.h"

SimpleTrackedVehicle::SimpleTrackedVehicle(Environment *environment_, const std::string &name_) : Vehicle(environment_, name_) {
    wheelRadius = 0.078;
    wheelBase = 0.4997;
    trackWidth = 0.097;
    vehicleBodyWidth = 0.254;
    trackVehicleSpace = 0.024;
    flipperRadius = 0.016;
    flipperBase = 0.32;
    flipperWidth = 0.05;

    this->density = 1.4;
    this->width = vehicleBodyWidth;

    this->leftTrack = new SimpleTrack(this->environment, name + ".leftTrack", wheelRadius, wheelRadius, wheelBase, trackWidth, -1, Category::LEFT);
    this->rightTrack = new SimpleTrack(this->environment, name + ".rightTrack", wheelRadius, wheelRadius, wheelBase, trackWidth, 1, Category::RIGHT);
    this->leftTrack->prepareFlippers(wheelRadius, flipperRadius, flipperBase, flipperWidth);
    this->rightTrack->prepareFlippers(wheelRadius, flipperRadius, flipperBase, flipperWidth);
}

SimpleTrackedVehicle::~SimpleTrackedVehicle() {
    delete this->leftTrack;
    delete this->rightTrack;
}

void SimpleTrackedVehicle::create() {
    this->vehicleBody = dBodyCreate(this->environment->world);
    this->vehicleGeom = dCreateBox(this->environment->space, this->leftTrack->betweenWheelsDistance, this->width, this->leftTrack->rearRadius);
    this->environment->setGeomName(this->vehicleGeom, name + ".vehicleGeom");
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->betweenWheelsDistance, this->width, this->leftTrack->rearRadius);
    //dMassAdjust(&this->vehicleMass, 2.40);
    dGeomSetCategoryBits(this->vehicleGeom, Category::OBSTACLE);
    dGeomSetCollideBits(this->vehicleGeom, Category::OBSTACLE | Category::TERRAIN);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    dGeomSetOffsetPosition(this->vehicleGeom, 0, 0, this->leftTrack->rearRadius);

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

void SimpleTrackedVehicle::destroy() {
    this->leftTrack->destroy();
    dJointDestroy(this->leftTrackJoint);
    this->rightTrack->destroy();
    dJointDestroy(this->rightTrackJoint);

    dBodyDestroy(this->vehicleBody);
    dGeomDestroy(this->vehicleGeom);
    
    dRigidBodyArrayDestroy(this->bodyArray);
}

void SimpleTrackedVehicle::step(dReal stepSize) {
    this->leftTrack->step(stepSize);
    this->rightTrack->step(stepSize);
}

void SimpleTrackedVehicle::draw() {
    {
        dsSetColor(0, 0, 1);
        const dReal *pos = dGeomGetPosition(this->vehicleGeom);
        const dReal *R = dGeomGetRotation(this->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(this->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);

        dVector3 frontMarkerPos;
        dBodyGetRelPointPos(this->vehicleBody, -0.25, 0, 0.1, frontMarkerPos);
        dsDrawSphereD(frontMarkerPos, R, 0.1);
    }

    this->leftTrack->draw();
    this->rightTrack->draw();
}

void SimpleTrackedVehicle::setVelocities(dReal a, dReal b) {
    setTrackVelocities(a, b);
}

void SimpleTrackedVehicle::setTrackVelocities(dReal left, dReal right) {
    this->leftTrack->setVelocity(left);
    this->rightTrack->setVelocity(right);
}

const dReal * SimpleTrackedVehicle::getPosition() {
    return dBodyGetPosition(this->vehicleBody);
}

const dReal * SimpleTrackedVehicle::getLinearVel() {
    return dBodyGetLinearVel(this->vehicleBody);
}

const dReal * SimpleTrackedVehicle::getAngularVel() {
    return dBodyGetAngularVel(this->vehicleBody);
}

const dReal * SimpleTrackedVehicle::getQuaternion() {
    return dBodyGetQuaternion(this->vehicleBody);
}

const dReal * SimpleTrackedVehicle::getRotation() {
    return dBodyGetRotation(this->vehicleBody);
}

void SimpleTrackedVehicle::setPosition(const dReal *p) {
    dRigidBodyArraySetPosition(this->bodyArray, p[0], p[1], p[2]);
}

void SimpleTrackedVehicle::setVel(const dReal *linear, const dReal *angular) {
    dRigidBodyArraySetVel(this->bodyArray, linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]);
}

void SimpleTrackedVehicle::setQuaternion(const dReal *q) {
    dRigidBodyArraySetQuaternion(this->bodyArray, q);
}

void SimpleTrackedVehicle::setRotation(const dReal *R) {
    dRigidBodyArraySetRotation(this->bodyArray, R);
}
