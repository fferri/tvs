//
//  TrackedVehicle.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef SIMPLE_TRACKED_VEHICLE_H_INCLUDED
#define SIMPLE_TRACKED_VEHICLE_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "Vehicle.h"
#include "ODEUtils.h"
#include "SimpleTrack.h"

class Environment;

class SimpleTrackedVehicle : public Vehicle {
public:
    SimpleTrack *leftTrack;
    SimpleTrack *rightTrack;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;
    dJointID leftTrackJoint;
    dJointID rightTrackJoint;
    dReal width;
    dReal wheelBase;
    dReal wheelRadius;
    dReal flipperRadius;
    dReal trackWidth;
    dReal flipperWidth;
    dReal flipperBase;
    dReal trackVehicleSpace;
    dReal vehicleBodyWidth;

    SimpleTrackedVehicle(Environment *environment_, const std::string &name_);
    virtual ~SimpleTrackedVehicle();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocities(dReal a, dReal b);
    void setTrackVelocities(dReal left, dReal right);
    const dReal * getPosition();
    const dReal * getLinearVel();
    const dReal * getAngularVel();
    const dReal * getQuaternion();
    const dReal * getRotation();
    void setPosition(const dReal *p);
    void setVel(const dReal *linear, const dReal *angular);
    void setQuaternion(const dReal *q);
    void setRotation(const dReal *R);
};

#endif // SIMPLE_TRACKED_VEHICLE_H_INCLUDED

