//
//  TrackedVehicle.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACKED_VEHICLE_H_INCLUDED
#define TRACKED_VEHICLE_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "Vehicle.h"
#include "Track.h"
#include "ODEUtils.h"

class Environment;

class TrackedVehicle : public Vehicle {
public:
    Track *leftTrack;
    Track *rightTrack;
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
    
    TrackedVehicle(Environment *environment_, const std::string &name_);
    virtual ~TrackedVehicle();
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

#endif // TRACKED_VEHICLE_H_INCLUDED

