//
//  Vehicle.h
//  tvs
//
//  Created by Federico Ferri on 19/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "ODEUtils.h"

class Environment;

class Vehicle {
public:
    Environment *environment;
    std::string name;
    dRigidBodyArrayID bodyArray;
    
    Vehicle(Environment *environment_, const std::string& name_) : environment(environment_), name(name_) {}
    virtual ~Vehicle() {}
    virtual void create() = 0;
    virtual void destroy() = 0;
    virtual void step(dReal stepSize) {}
    virtual void draw() = 0;
    virtual void setVelocities(dReal a, dReal b) = 0;
    virtual const dReal * getPosition() = 0;
    virtual const dReal * getLinearVel() = 0;
    virtual const dReal * getAngularVel() = 0;
    virtual const dReal * getQuaternion() = 0;
    virtual const dReal * getRotation() = 0;
    virtual void setPosition(const dReal *p) = 0;
    virtual void setVel(const dReal *linear, const dReal *angular) = 0;
    virtual void setQuaternion(const dReal *q) = 0;
    virtual void setRotation(const dReal *R) = 0;
};

#endif // VEHICLE_H_INCLUDED

