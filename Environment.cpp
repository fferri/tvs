//
//  Environment.cpp
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "ODEUtils.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <drawstuff/drawstuff.h>
#include <ompl/util/Console.h>

static const dVector3 center = {3,3,0};
static const dVector3 extents = {7,7,7};
static const dReal limit = 2.0;

Environment::Environment() {
    boost::property_tree::ini_parser::read_ini("simulator.ini", this->config);
    this->stepSize = 0.01;
    this->simulationStepsPerFrame = 4;

#define vehicleZoffset 0.0
#if 0
#define vehicleZoffset 0.4
    this->pcl = new PointCloud("problem01-ds.pcd", 0.25);
    this->mesh = 0L;
#elif 0
#define vehicleZoffset 0.0
    this->pcl = 0L;
    this->mesh = new TriMesh();
#endif
    this->v = new TrackedVehicle("robot", 1, -2, 0.301+vehicleZoffset);
}

Environment::~Environment() {
    dJointGroupDestroy(this->contactGroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
    delete this->v;
    if(this->pcl) delete this->pcl;
    if(this->mesh) delete this->mesh;
}

inline dGeomID createAABox(Environment *e, dReal x1, dReal y1, dReal z1, dReal x2, dReal y2, dReal z2, dReal rx = 1.0, dReal ry = 0.0, dReal rz = 0.0, dReal rAngle = 0.0) {
    static int i = 0;
    dGeomID g = dCreateBox(e->space, x2 - x1, y2 - y1, z2 - z1);
    dGeomSetPosition(g, x1 + 0.5 * (x2 - x1), y1 + 0.5 * (y2 - y1), z1 + 0.5 * (z2 - z1));
    dMatrix3 R;
    dRFromAxisAndAngle(R, rx, ry, rz, rAngle);
    dGeomSetRotation(g, R);
    dGeomSetCategoryBits(g, Category::TERRAIN);
    dGeomSetCollideBits(g, Category::GROUSER);
    e->setGeomName(g, "panel" + boost::lexical_cast<std::string>(i++));
    return g;
}

void Environment::create() {
    this->world = dWorldCreate();
#if 0
    this->space = dQuadTreeSpaceCreate(0, center, extents, 6);
#else
    this->space = dHashSpaceCreate(0);
#endif
    this->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, 0, 0, -9.81);
    //dWorldSetERP(this->world, 0.7);
    //dWorldSetCFM(this->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(this->world, 0.9);
    //dWorldSetContactSurfaceLayer(this->world, 0.001);
    dWorldSetAutoDisableFlag(this->world, 1);

    this->planeGeom = dCreatePlane(this->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    setGeomName(this->planeGeom, "worldPlane");
    dGeomSetCategoryBits(this->planeGeom, Category::TERRAIN);
    dGeomSetCollideBits(this->planeGeom, Category::GROUSER | Category::OBSTACLE);

    this->v->create(this);
    if(this->pcl) this->pcl->create(this, this->v->getPosition(), limit);
    if(this->mesh) {
        this->mesh->create(this, "models/maze.stl");
        //dGeomSetPosition(this->mesh->geom, 4, 4, -0.001);
        //dMatrix3 R; dRFromAxisAndAngle(R, 1, 0, 0, M_PI_2);
        //dGeomSetRotation(this->mesh->geom, R);
    }
    
#if 0
    dVector3 sides = {1,1,1};
    dGeomID g = dCreateBox(this->space, sides[0], sides[1], sides[2]);
    dMass m;
    dMassSetBox(&m, 1, sides[0], sides[1], sides[2]);
    dBodyID b = dBodyCreate(this->world);
    dBodySetMass(b, &m);
    dGeomSetBody(g, b);
    dBodySetPosition(b, 1, 1, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 2, 2, 0.3);
    dBodySetRotation(b, R);
#endif

    const dReal h = 1.3; // wall height
    const dReal t = 0.1; // wall thickness
    const dReal T = 0.25; // arch thickness
    const dReal l = 4; // ramp length
    const dReal W = 9; // total width
    const dReal D = 8; // total depth
    const dReal O = 3; // arch width
    const dReal w = 2; // ramp width
    dReal a = atan2(h,l), l2 = hypot(h,l);
    this->boxes.push_back(createAABox(this, 0,   0,   0,   W-O, t,   h));
    this->boxes.push_back(createAABox(this, 0,   D-t, 0,   W,   D,   h));
    this->boxes.push_back(createAABox(this, 0,   0,   0,   t,   D,   h));
    this->boxes.push_back(createAABox(this, W-t, 0,   0,   W,   D,   h));
    this->boxes.push_back(createAABox(this, W-O, 0,   0,   W-O+T, w, h));
    this->boxes.push_back(createAABox(this, W-T, 0,   0,   W,   w, h));
    this->boxes.push_back(createAABox(this, W-O, 0,   h-t, W,   w, h));
    this->boxes.push_back(createAABox(this, W-O-0.5*l-0.5*l2, 0, 0.5*h-0.5*t-0.5*t, W-O-0.5*l+0.5*l2, 1.5, 0.5*h+0.5*t-0.5*t, 0, 1, 0, -a));
}

void Environment::destroy() {
    this->v->destroy();
    if(this->pcl) this->pcl->destroy();
    if(this->mesh) this->mesh->destroy();
}

std::string Environment::getGeomName(dGeomID geom) const
{
    std::map<dGeomID, std::string>::const_iterator it = geomNames.find(geom);
    if (it == geomNames.end())
        return boost::lexical_cast<std::string>(reinterpret_cast<unsigned long>(geom));
    else
        return it->second;
}

void Environment::setGeomName(dGeomID geom, const std::string &name)
{
    geomNames[geom] = name;
}

int Environment::getMaxContacts(dGeomID o1, dGeomID o2) {
    return 10;
}

bool Environment::isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2) {
    unsigned long catBits1 = dGeomGetCategoryBits(*o1);
    unsigned long catBits2 = dGeomGetCategoryBits(*o2);

    if((catBits1 & cat1) && (catBits2 & cat2)) {
        return true;
    }
    if((catBits1 & cat2) && (catBits2 & cat1)) {
        // swap o1 and o2
        dGeomID tmp = *o1;
        *o1 = *o2;
        *o2 = tmp;
        return true;
    }
    return false;
}

bool Environment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    if(isCatPair(Category::GROUSER, Category::G_GUIDE, &o1, &o2))
        return true;
    if(isCatPair(Category::WHEEL, Category::G_GUIDE, &o1, &o2)) // XXX: not needed really
        return true;
    if(isCatPair(Category::WHEEL, Category::GROUSER, &o1, &o2))
        return true;
    return false;
}

static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->nearCallback(o1, o2);
}

void Environment::nearCallback(dGeomID o1, dGeomID o2) {
    if(isCatPair(Category::WHEEL, Category::GROUSER, &o1, &o2))
        nearCallbackWheelGrouser(o1, o2);
    else if(isCatPair(Category::GROUSER, Category::TERRAIN, &o1, &o2))
        nearCallbackGrouserTerrain(o1, o2);
    else if(isCatPair(Category::GROUSER, Category::G_GUIDE, &o1, &o2))
        nearCallbackGrouserGuide(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}

void Environment::nearCallbackWheelGrouser(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b2); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = dInfinity;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
    }
}

void Environment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b1); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = 2.0;
        contact[i].surface.mu2 = 0.5;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
    }
}

void Environment::nearCallbackGrouserGuide(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = 3; // 3 should be enough  //getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.2;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = 0;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
    }
}

void Environment::nearCallbackDefault(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = 5.0;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
    }
}

bool Environment::step(dReal stepSize, int simulationStepsPerFrame) {
    stepNum++;
    if(!(stepNum % 10) && this->pcl) this->pcl->create(this, this->v->getPosition(), limit);

    this->badCollision = false;
    for(size_t i = 0; i < simulationStepsPerFrame; i++) {
        // find collisions and add contact joints
        dSpaceCollide(this->space, this, &nearCallbackWrapper);
        // step the simulation
        dWorldQuickStep(this->world, stepSize / (dReal)simulationStepsPerFrame);
        // remove all contact joints
        dJointGroupEmpty(this->contactGroup);
    }
    return this->badCollision;
}

static void evaluateCollisionNearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->evaluateCollisionNearCallback(o1, o2);
}

void Environment::evaluateCollisionNearCallback(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    dContact contact[1];  // one contact is sufficient
    int numc = dCollide(o1, o2, 1, &contact[0].geom, sizeof(dContact));
    // flag collision is there is really a collision and is of those not allowed
    if(numc > 0 && !isValidCollision(o1, o2, contact[0])) {
        badCollision = true;
        OMPL_INFORM("Collision between %s (%s) and %s (%s)",
                    getGeomName(o1).c_str(), dClassGetName(dGeomGetClass(o1)),
                    getGeomName(o2).c_str(), dClassGetName(dGeomGetClass(o2)));
    }
}

bool Environment::evaluateCollision() {
    this->badCollision = false;
    dSpaceCollide(this->space, this, &evaluateCollisionNearCallbackWrapper);
    return this->badCollision;
}

void Environment::draw() {
    if(this->pcl) this->pcl->draw();
    if(this->mesh) this->mesh->draw();
    this->v->draw();


    for(std::vector<dGeomID>::iterator it = boxes.begin(); it != boxes.end(); it++) {
        dsSetColor(0.6, 0.6, 0.7);
        const dReal *pos = dGeomGetPosition(*it);
        const dReal *R = dGeomGetRotation(*it);
        dReal sides[3];
        dGeomBoxGetLengths(*it, sides);
        dsDrawBoxD(pos, R, sides);
    }
}

