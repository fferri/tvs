//
// Created by peci1 on 6.1.16.
//

#include "TrackedVehicleEnvironment.h"
#include "TrackedVehicle.h"

TrackedVehicleEnvironment::TrackedVehicleEnvironment() {
    TrackedVehicle *tv = new TrackedVehicle(this, "robot");
    tv->leftTrack->velocity.setSlope(config.world.track_acceleration);
    tv->rightTrack->velocity.setSlope(config.world.track_acceleration);
    this->v = tv;
}

TrackedVehicleEnvironment::~TrackedVehicleEnvironment() {

}

bool TrackedVehicleEnvironment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    if(isCatPair(Category::TRACK_GROUSER, Category::TRACK_GUIDE, &o1, &o2))
        return true;
    if(isCatPair(Category::TRACK_WHEEL, Category::TRACK_GUIDE, &o1, &o2)) // XXX: not needed really
        return true;
    if(isCatPair(Category::TRACK_WHEEL, Category::TRACK_GROUSER, &o1, &o2))
        return true;
    if(isCatPair(Category::TRACK_WHEEL, Category::TERRAIN, &o1, &o2))
        return true;

    if(isCatPair(Category::FLIPPER_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    if(isCatPair(Category::FLIPPER_GROUSER, Category::FLIPPER_GUIDE, &o1, &o2))
        return true;
    if(isCatPair(Category::FLIPPER_WHEEL, Category::FLIPPER_GUIDE, &o1, &o2)) // XXX: not needed really
        return true;
    if(isCatPair(Category::FLIPPER_WHEEL, Category::FLIPPER_GROUSER, &o1, &o2))
        return true;
    if(isCatPair(Category::FLIPPER_WHEEL, Category::TERRAIN, &o1, &o2))
        return true;
    
    return false;
}

void TrackedVehicleEnvironment::nearCallbackWheelGrouser(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_wheel_grouser.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b2); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = config.contact_wheel_grouser.bounce;
        contact[i].surface.bounce_vel = config.contact_wheel_grouser.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_wheel_grouser.soft_cfm;
        contact[i].surface.mu = config.contact_wheel_grouser.mu;
        contact[i].surface.mu2 = config.contact_wheel_grouser.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_wheel_grouser.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void TrackedVehicleEnvironment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
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

void TrackedVehicleEnvironment::nearCallbackGrouserGuide(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
//    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;  // we connect grousers to tracks (having guides) via a planar joint
    int maxc = config.contact_grouser_guide.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = config.contact_grouser_guide.bounce;
        contact[i].surface.bounce_vel = config.contact_grouser_guide.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_grouser_guide.soft_cfm;
        contact[i].surface.mu = config.contact_grouser_guide.mu;
        contact[i].surface.mu2 = config.contact_grouser_guide.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_grouser_guide.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void TrackedVehicleEnvironment::nearCallback(dGeomID o1, dGeomID o2){
    if(isCatPair(Category::TRACK_WHEEL, Category::TRACK_GROUSER, &o1, &o2) || isCatPair(Category::FLIPPER_WHEEL, Category::FLIPPER_GROUSER, &o1, &o2))
        this->nearCallbackWheelGrouser(o1, o2);
    else if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2) || isCatPair(Category::FLIPPER_GROUSER, Category::TERRAIN, &o1, &o2))
        this->nearCallbackGrouserTerrain(o1, o2);
    else if(isCatPair(Category::TRACK_GROUSER, Category::TRACK_GUIDE, &o1, &o2) || isCatPair(Category::FLIPPER_GROUSER, Category::FLIPPER_GUIDE, &o1, &o2))
        this->nearCallbackGrouserGuide(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}
