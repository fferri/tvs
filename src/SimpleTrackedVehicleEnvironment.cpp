//
// Created by peci1 on 6.1.16.
//

#include "SimpleTrackedVehicleEnvironment.h"
#include "SimpleTrackedVehicle.h"
#include <drawstuff/drawstuff.h>
#include <objects.h>
#include <collision_util.h>

SimpleTrackedVehicleEnvironment::SimpleTrackedVehicleEnvironment() {
    SimpleTrackedVehicle *ssv = new SimpleTrackedVehicle(this, "robot");
    ssv->leftTrack->velocity.setSlope(config.world.track_acceleration);
    ssv->rightTrack->velocity.setSlope(config.world.track_acceleration);
    this->v = ssv;
}

SimpleTrackedVehicleEnvironment::~SimpleTrackedVehicleEnvironment() {

}

bool SimpleTrackedVehicleEnvironment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;

    if(isCatPair(Category::FLIPPER_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;

    return false;
}

struct posr {
    dVector3 pos;
    dMatrix3 R;
};
std::vector<posr> forces;
const dReal tracksDistance = 0.4; // TODO should be loaded from config
const dReal steeringEfficiency = 0.5; // TODO should be loaded from config

void SimpleTrackedVehicleEnvironment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

    // body of the whole vehicle
    dBodyID vehicleBody = ((SimpleTrackedVehicle*)this->v)->vehicleBody;

    unsigned long geom1Categories = dGeomGetCategoryBits(o1);

    // speeds of the belts
    const dReal leftBeltSpeed = ((SimpleTrackedVehicle*)this->v)->leftTrack->getVelocity();
    const dReal rightBeltSpeed = ((SimpleTrackedVehicle*)this->v)->rightTrack->getVelocity();

    dReal beltSpeed = 0; // speed of the belt which is in collision and examined right now
    if (geom1Categories & Category::LEFT) {
        beltSpeed = leftBeltSpeed;
    } else {
        beltSpeed = rightBeltSpeed;
    }

    // the desired linear and angular speeds (set by desired track velocities)
    const dReal linearSpeed = (leftBeltSpeed + rightBeltSpeed) / 2;
    const dReal angularSpeed = (leftBeltSpeed - rightBeltSpeed) * steeringEfficiency / tracksDistance;

    // radius of the turn the robot is doing
    const dReal desiredRotationRadiusSigned = (fabs(angularSpeed) < 0.1) ?
                                                dInfinity : // is driving straight
                                                ((fabs(linearSpeed) < 0.1) ?
                                                    0 : // is rotating about a single point
                                                    linearSpeed / angularSpeed // general movement
                                                );


    dVector3 yAxisGlobal; // vector pointing from vehicle body center in the direction of +y axis
    dVector3 centerOfRotation; // at infinity if driving straight, so we need to distinguish the case
    { // compute the center of rotation
        dBodyVectorToWorld(vehicleBody, 0, 1, 0, yAxisGlobal);

        dCopyVector3(centerOfRotation, yAxisGlobal);
        // make the unit vector as long as we need (and change orientation if needed; the radius is a signed number)
        dScaleVector3(centerOfRotation, desiredRotationRadiusSigned);

        const dReal *vehicleBodyPos = dBodyGetPosition(vehicleBody);
        dAddVectors3(centerOfRotation, centerOfRotation, vehicleBodyPos);
    }

    int maxContacts = 20;
    dContact contact[maxContacts];
    int numContacts = dCollide(o1, o2, maxContacts, &contact[0].geom, sizeof(dContact));

    for(size_t i = 0; i < numContacts; i++) {
        dVector3 contactInVehiclePos; // position of the contact point relative to vehicle body
        dBodyGetPosRelPoint(vehicleBody, contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2], contactInVehiclePos);

        dVector3 beltDirection; // vector tangent to the belt pointing in the belt's movement direction
        dCalcVectorCross3(beltDirection, contact[i].geom.normal, yAxisGlobal);
        if (beltSpeed > 0) {
            dNegateVector3(beltDirection);
        }

        if (desiredRotationRadiusSigned != dInfinity) { // non-straight drive

            dVector3 COR2Contact; // vector pointing from the center of rotation to the contact point
            dSubtractVectors3(COR2Contact, contact[i].geom.pos, centerOfRotation);
            // the friction force should be perpendicular to COR2Contact
            dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, COR2Contact);

            const dReal linearSpeedSignum = (fabs(linearSpeed) > 0.1) ? sgn(linearSpeed) : 1;

            // contactInVehiclePos[0] > 0 means the contact is in the front part of the track
            if (sgn(angularSpeed) * sgn(dCalcVectorDot3(yAxisGlobal, contact[i].fdir1)) !=
                    sgn(contactInVehiclePos[0]) * linearSpeedSignum) {
                dNegateVector3(contact[i].fdir1);
            }

        } else { // straight drive

            dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, yAxisGlobal);

            if (dCalcVectorDot3(contact[i].fdir1, beltDirection) < 0) {
                dNegateVector3(contact[i].fdir1);
            }

        }

        // use friction direction and motion1 to simulate the track movement
        contact[i].surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2;
        contact[i].surface.mu = 0.5;
        contact[i].surface.mu2 = 10;
        // the dot product <beltDirection,fdir1> is the cosine of the angle they form (because both are unit vectors)
        contact[i].surface.motion1 = -dCalcVectorDot3(beltDirection, contact[i].fdir1) * fabs(beltSpeed) * 0.07;

        // friction force visualization
        dMatrix3 forceRotation;
        dVector3 vec;
        dBodyVectorToWorld(vehicleBody, 1, 0, 0, vec);
        dRFrom2Axes(forceRotation, contact[i].fdir1[0], contact[i].fdir1[1], contact[i].fdir1[2], vec[0], vec[1], vec[2]);
        posr data;
        dCopyVector3(data.pos, contact[i].geom.pos);
        dCopyMatrix4x3(data.R, forceRotation);
        forces.push_back(data);

        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_grouser_terrain.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void SimpleTrackedVehicleEnvironment::nearCallback(dGeomID o1, dGeomID o2){
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2) ||
       isCatPair(Category::FLIPPER_GROUSER, Category::TERRAIN, &o1, &o2) )
        this->nearCallbackGrouserTerrain(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}



void SimpleTrackedVehicleEnvironment::draw() {
    Environment::draw();

//    for (size_t i=0; i < forces.size(); ++i) {
//        posr data = forces[i];
//        dReal sides[3] = {0.4, 0.03, 0.03};
//        dVector3 point = {sides[0]/2, 0, 0};
//        dVector3 pos = {0, 0, 0};
//        dMultiply0_331(pos, data.R, point);
//        dSubtractVectors3(pos, data.pos, pos);
//
//        dsDrawBoxD(pos, data.R, sides);
//        dsDrawSphereD(data.pos, data.R, 0.03);
//    }

    forces.clear();
}
