//
// Created by peci1 on 6.1.16.
//

#include "SkidSteerVehicleEnvironment.h"
#include "SkidSteeringVehicle.h"

SkidSteerVehicleEnvironment::SkidSteerVehicleEnvironment() {
    SkidSteeringVehicle *ssv = new SkidSteeringVehicle(this, "robot", 1, -2, 0.301);
    ssv->velocityLeft.setSlope(config.world.track_acceleration);
    ssv->velocityRight.setSlope(config.world.track_acceleration);
    this->v = ssv;
}

SkidSteerVehicleEnvironment::~SkidSteerVehicleEnvironment() {

}

bool SkidSteerVehicleEnvironment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    return false;
}

void SkidSteerVehicleEnvironment::nearCallback(dGeomID o1, dGeomID o2){
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
        this->nearCallbackGrouserTerrain(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}

void SkidSteerVehicleEnvironment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_grouser_terrain.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b1); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = config.contact_grouser_terrain.bounce;
        contact[i].surface.bounce_vel = config.contact_grouser_terrain.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_grouser_terrain.soft_cfm;
        contact[i].surface.mu = config.contact_grouser_terrain.mu;
        contact[i].surface.mu2 = config.contact_grouser_terrain.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_grouser_terrain.debug)
            this->contacts.push_back(contact[i].geom);
    }
}
