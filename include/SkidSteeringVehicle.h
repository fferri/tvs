//
//  SkidSteeringVehicle.h
//  tvs
//
//  Created by Federico Ferri on 16/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef SKID_STEERING_VEHICLE_H_INCLUDED
#define SKID_STEERING_VEHICLE_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "Vehicle.h"
#include "ODEUtils.h"

class Environment;

class SkidSteeringVehicle : public Vehicle {
public:
    dReal density;
    dReal vehicleBodyWidth;
    dReal vehicleBodyLength;
    dReal vehicleBodyHeight;
    dBodyID vehicleBody;
    dGeomID vehicleGeom;
    dMass vehicleMass;
    dReal trackVehicleSpace;
    dReal wheelBase;
    dReal wheelRadius;
    dReal wheelWidth;
    dJointID wheelJoint[2][2];
    dBodyID wheelBody[2][2];
    dGeomID wheelGeom[2][2];
    dMass wheelMass[2][2];
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
    LinVelProfInt velocityLeft;
    LinVelProfInt velocityRight;
    
    SkidSteeringVehicle(Environment *environment_, const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~SkidSteeringVehicle();
    void create();
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocities(dReal a, dReal b);
    void setWheelVelocities(dReal left, dReal right);
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

#endif // SKID_STEERING_VEHICLE_H_INCLUDED

