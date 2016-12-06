//
//  SkidSteeringVehicle.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "SkidSteeringVehicle.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <drawstuff/drawstuff.h>
#include "ODEUtils.h"

SkidSteeringVehicle::SkidSteeringVehicle(Environment *environment_, const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset) : Vehicle(environment_, name_) {
    this->wheelRadius = 0.078;
    this->wheelBase = 0.4997;
    this->wheelWidth = 0.097;
    this->vehicleBodyWidth = 0.254;
    this->vehicleBodyLength = this->wheelBase;
    this->vehicleBodyHeight = 2 * this->wheelRadius;
    this->trackVehicleSpace = 0.024;
    this->density = 1.4;
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->zOffset = zOffset;
}

SkidSteeringVehicle::~SkidSteeringVehicle() {
}

void SkidSteeringVehicle::create() {
    this->vehicleBody = dBodyCreate(this->environment->world);
    this->vehicleGeom = dCreateBox(this->environment->space, this->vehicleBodyLength, this->vehicleBodyWidth, this->vehicleBodyHeight);
    this->environment->setGeomName(this->vehicleGeom, name + ".vehicleGeom");
    dMassSetBox(&this->vehicleMass, this->density, this->vehicleBodyLength, this->vehicleBodyWidth, this->vehicleBodyHeight);
    dGeomSetCategoryBits(this->vehicleGeom, Category::OBSTACLE);
    dGeomSetCollideBits(this->vehicleGeom, Category::OBSTACLE | Category::TERRAIN);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dBodySetPosition(this->vehicleBody, this->xOffset, this->yOffset, this->zOffset);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    dGeomSetOffsetPosition(this->vehicleGeom, 0, 0, this->wheelRadius);

    dReal w = this->vehicleBodyWidth + this->wheelWidth + 2 * this->trackVehicleSpace;
    for(int fr = 0; fr < 2; fr++) {
        for(int lr = 0; lr < 2; lr++) {
            this->wheelGeom[fr][lr] = dCreateCylinder(this->environment->space, this->wheelRadius, this->wheelWidth);
            this->environment->setGeomName(this->wheelGeom[fr][lr], this->name + "." + (!fr ? "front" : "rear") + (!lr ? "Left" : "Right") + "Wheel");
            dGeomSetCategoryBits(this->wheelGeom[fr][lr], Category::TRACK_GROUSER);
            dGeomSetCollideBits(this->wheelGeom[fr][lr], Category::TERRAIN);
            dMassSetCylinder(&this->wheelMass[fr][lr], this->density, 3, this->wheelRadius, this->wheelWidth);
            this->wheelBody[fr][lr] = dBodyCreate(this->environment->world);
            dBodySetMass(this->wheelBody[fr][lr], &this->wheelMass[fr][lr]);
            dGeomSetBody(this->wheelGeom[fr][lr], this->wheelBody[fr][lr]);
            dBodySetPosition(this->wheelBody[fr][lr], this->xOffset + (fr - 0.5) * this->wheelBase, this->yOffset + w * (lr - 0.5), this->zOffset);
            dMatrix3 wheelR;
            dRFromZAxis(wheelR, 0, 2 * lr - 1, 0);
            dBodySetRotation(this->wheelBody[fr][lr], wheelR);
            this->wheelJoint[fr][lr] = dJointCreateHinge(this->environment->world, 0);
            dJointAttach(this->wheelJoint[fr][lr], this->vehicleBody, this->wheelBody[fr][lr]);
            dJointSetHingeAnchor(this->wheelJoint[fr][lr], this->xOffset + (fr - 0.5) * this->wheelBase, this->yOffset + this->vehicleBodyWidth * (lr - 0.5), this->zOffset);
            dJointSetHingeAxis(this->wheelJoint[fr][lr], 0, 1, 0);
            dJointSetHingeParam(this->wheelJoint[fr][lr], dParamFMax, 5.0);
        }
    }
    
    this->bodyArray = dRigidBodyArrayCreate(this->vehicleBody);
    for(int fr = 0; fr < 2; fr++) {
        for(int lr = 0; lr < 2; lr++) {
            dRigidBodyArrayAdd(this->bodyArray, this->wheelBody[fr][lr]);
        }
    }
}

void SkidSteeringVehicle::destroy() {
    dBodyDestroy(this->vehicleBody);
    dGeomDestroy(this->vehicleGeom);
    
    for(int fr = 0; fr < 2; fr++) {
        for(int lr = 0; lr < 2; lr++) {
            dBodyDestroy(this->wheelBody[fr][lr]);
            dGeomDestroy(this->wheelGeom[fr][lr]);
            dJointDestroy(this->wheelJoint[fr][lr]);
        }
    }
    
    dRigidBodyArrayDestroy(this->bodyArray);
}

void SkidSteeringVehicle::step(dReal stepSize) {
    this->velocityLeft.step(stepSize);
    this->velocityRight.step(stepSize);

    for(int fr = 0; fr < 2; fr++) {
        dJointSetHingeParam(this->wheelJoint[fr][0], dParamVel, this->velocityLeft.get());
        dJointSetHingeParam(this->wheelJoint[fr][1], dParamVel, this->velocityRight.get());
    }
}

void SkidSteeringVehicle::draw() {
    {
        dsSetColor(0, 0, 1);
        const dReal *pos = dGeomGetPosition(this->vehicleGeom);
        const dReal *R = dGeomGetRotation(this->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(this->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }

    dsSetColor(1, 1, 0);
    for(int fr = 0; fr < 2; fr++) {
        for(int lr = 0; lr < 2; lr++) {
            const dReal *pos = dGeomGetPosition(this->wheelGeom[fr][lr]);
            const dReal *R = dGeomGetRotation(this->wheelGeom[fr][lr]);
            dReal radius, length;
            dGeomCylinderGetParams(this->wheelGeom[fr][lr], &radius, &length);
            dsDrawCylinderD(pos, R, length, radius);
        }
    }
}

void SkidSteeringVehicle::setVelocities(dReal a, dReal b) {
    setWheelVelocities(a, b);
}

void SkidSteeringVehicle::setWheelVelocities(dReal left, dReal right) {
    this->velocityLeft.set(left);
    this->velocityRight.set(right);
}

const dReal * SkidSteeringVehicle::getPosition() {
    return dBodyGetPosition(this->vehicleBody);
}

const dReal * SkidSteeringVehicle::getLinearVel() {
    return dBodyGetLinearVel(this->vehicleBody);
}

const dReal * SkidSteeringVehicle::getAngularVel() {
    return dBodyGetAngularVel(this->vehicleBody);
}

const dReal * SkidSteeringVehicle::getQuaternion() {
    return dBodyGetQuaternion(this->vehicleBody);
}

const dReal * SkidSteeringVehicle::getRotation() {
    return dBodyGetRotation(this->vehicleBody);
}

void SkidSteeringVehicle::setPosition(const dReal *p) {
    dRigidBodyArraySetPosition(this->bodyArray, p[0], p[1], p[2]);
}

void SkidSteeringVehicle::setVel(const dReal *linear, const dReal *angular) {
    dRigidBodyArraySetVel(this->bodyArray, linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]);
}

void SkidSteeringVehicle::setQuaternion(const dReal *q) {
    dRigidBodyArraySetQuaternion(this->bodyArray, q);
}

void SkidSteeringVehicle::setRotation(const dReal *R) {
    dRigidBodyArraySetRotation(this->bodyArray, R);
}
