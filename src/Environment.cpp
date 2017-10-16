//
//  Environment.cpp
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "config.h"
#include "Environment.h"
#include "ODEUtils.h"
#ifdef OCTOMAP_FOUND
#include "OcTreeBuilder.h"
#endif
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <drawstuff/drawstuff.h>
#include <ompl/util/Console.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <PlanarJoint.h>
#include "utils.h"
#include "TrackedVehicle.h"
#include "SkidSteeringVehicle.h"

Environment::Environment() {
    datetime = getDateTimeString();
    readConfig();
}

Environment::~Environment() {
    dJointGroupDestroy(this->contactGroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
    if(this->v) delete this->v;
}

static void readContactParams(std::string section, ContactParams* p, const boost::property_tree::ptree& pt) {
    p->max_contacts = pt.get<int>(section + ".max_contacts");
    p->bounce = pt.get<dReal>(section + ".bounce");
    p->bounce_vel = pt.get<dReal>(section + ".bounce_vel");
    p->soft_cfm = pt.get<dReal>(section + ".soft_cfm");
    if(pt.get<std::string>(section + ".mu") == "infinity") p->mu = dInfinity; else p->mu = pt.get<dReal>(section + ".mu");
    if(pt.get<std::string>(section + ".mu2", "") == "infinity") p->mu2 = dInfinity; else p->mu2 = pt.get<dReal>(section + ".mu2", p->mu);
    p->debug = pt.get<bool>(section + ".debug", false);
}

void Environment::readConfig() {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(CONFIG_PATH "/simulator.ini", pt);
    config.step.step_size = pt.get<float>("step.step_size", 0.01);
    config.step.simulation_steps_per_frame = pt.get<int>("step.simulation_steps_per_frame", 4);
    readContactParams("contact_wheel_grouser", &config.contact_wheel_grouser, pt);
    readContactParams("contact_grouser_terrain", &config.contact_grouser_terrain, pt);
    readContactParams("contact_grouser_guide", &config.contact_grouser_guide, pt);
    readContactParams("contact_default", &config.contact_default, pt);
    config.world.gravity_x = pt.get<dReal>("world.gravity_x", 0.0);
    config.world.gravity_y = pt.get<dReal>("world.gravity_y", 0.0);
    config.world.gravity_z = pt.get<dReal>("world.gravity_z", 0.0);
    config.world.max_track_speed = pt.get<dReal>("world.max_track_speed", 5.0);
    config.world.track_acceleration = pt.get<dReal>("world.track_acceleration", 200.0);
    config.world.simulator_threads = pt.get<dReal>("world.simulator_threads", 0);
    config.joystick.enabled = pt.get<unsigned short>("joystick.enabled", 0);
    config.joystick.device = pt.get<unsigned short>("joystick.device", 0);
    config.joystick.gain = pt.get<dReal>("joystick.gain", 1.0);
    config.show_contact_points = false;
}

bool readPosition(const boost::property_tree::ptree &ini, std::string section, dVector3 &pos) {
    boost::optional<float> x = ini.get_optional<float>(section + ".position_x");
    boost::optional<float> y = ini.get_optional<float>(section + ".position_y");
    boost::optional<float> z = ini.get_optional<float>(section + ".position_z");
    if(x) pos[0] = *x;
    if(y) pos[1] = *y;
    if(z) pos[2] = *z;
    return x || y || z;
}

bool readOrientation(const boost::property_tree::ptree &ini, std::string section, dMatrix3 &R) {
    boost::optional<float> qx = ini.get_optional<float>(section + ".orientation_quaternion_x");
    boost::optional<float> qy = ini.get_optional<float>(section + ".orientation_quaternion_y");
    boost::optional<float> qz = ini.get_optional<float>(section + ".orientation_quaternion_z");
    boost::optional<float> qw = ini.get_optional<float>(section + ".orientation_quaternion_w");
    if(qx && qy && qz && qw) {
        dQuaternion q = {*qx, *qy, *qz, *qw};
        dQtoR(q, R);
        return true;
    }
    boost::optional<float> ax = ini.get_optional<float>(section + ".orientation_axis_x");
    boost::optional<float> ay = ini.get_optional<float>(section + ".orientation_axis_y");
    boost::optional<float> az = ini.get_optional<float>(section + ".orientation_axis_z");
    boost::optional<float> angle = ini.get_optional<float>(section + ".orientation_axis_angle");
    if(ax && ay && az && angle) {
        dRFromAxisAndAngle(R, *ax, *ay, *az, *angle);
        return true;
    }
    boost::optional<float> ex = ini.get_optional<float>(section + ".orientation_euler_x");
    boost::optional<float> ey = ini.get_optional<float>(section + ".orientation_euler_y");
    boost::optional<float> ez = ini.get_optional<float>(section + ".orientation_euler_z");
    if(ex && ey && ez) {
        dRFromEulerAngles(R, *ex, *ey, *ez);
        return true;
    }
    return false;
}

bool readColor(const boost::property_tree::ptree &ini, std::string section, float &r, float &g, float &b, float &a) {
    boost::optional<float> cr = ini.get_optional<float>(section + ".color_r");
    boost::optional<float> cg = ini.get_optional<float>(section + ".color_g");
    boost::optional<float> cb = ini.get_optional<float>(section + ".color_b");
    boost::optional<float> ca = ini.get_optional<float>(section + ".alpha");
    if(cr && cg && cb) {
        r = cr.get();
        g = cg.get();
        b = cb.get();
        if(ca) a = ca.get();
        return true;
    } else {
        return false;
    }
}

void Environment::create() {
    this->world = dWorldCreate();
#if 0
    this->space = dQuadTreeSpaceCreate(0, center, extents, 6);
#else
    this->space = dHashSpaceCreate(0);
#endif
    this->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, config.world.gravity_x, config.world.gravity_y, config.world.gravity_z);
    //dWorldSetERP(this->world, 0.7);
    //dWorldSetCFM(this->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(this->world, 0.9);
    //dWorldSetContactSurfaceLayer(this->world, 0.001);
    dWorldSetAutoDisableFlag(this->world, 1);

    if(config.world.simulator_threads > 1) {
        this->threading = dThreadingAllocateMultiThreadedImplementation();
        this->pool = dThreadingAllocateThreadPool(config.world.simulator_threads, 0, dAllocateFlagBasicData, NULL);
        dThreadingThreadPoolServeMultiThreadedImplementation(this->pool, this->threading);
        dWorldSetStepThreadingImplementation(this->world, dThreadingImplementationGetFunctions(this->threading), this->threading);
    }

    this->planeGeom = dCreatePlane(this->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    setGeomName(this->planeGeom, "worldPlane");
    dGeomSetCategoryBits(this->planeGeom, Category::TERRAIN);
    dGeomSetCollideBits(this->planeGeom, Category::TRACK_GROUSER | Category::FLIPPER_GROUSER | Category::OBSTACLE);

    boost::property_tree::ptree scene;
    boost::property_tree::ini_parser::read_ini(CONFIG_PATH "/scene.ini", scene);

    if(this->v) this->v->create();

    size_t meshIndex = 0;
    while(true) {
        std::string k = "mesh";
        k += boost::lexical_cast<std::string>(meshIndex++);
        boost::optional<std::string> file = scene.get_optional<std::string>(k + ".file");
        if(file) {
            std::cout << "Loading mesh " << file.get() << std::endl;
            TriMeshPtr m(new TriMesh);
            std::string filefmt = scene.get<std::string>(k + ".file_format", "binary");
            float scale = scene.get<float>(k + ".scale_factor", 1.0);
            m->create(this, file.get().c_str(), filefmt == "binary", scale);
            m->color.r = 0.6;
            m->color.g = 0.6;
            m->color.b = 0.7;
            m->color.a = 0.8;
            readColor(scene, k, m->color.r, m->color.g, m->color.b, m->color.a);
            dGeomSetCategoryBits(m->geom, 0);
            dGeomSetCollideBits(m->geom, Category::TRACK_GROUSER | Category::TRACK_WHEEL | Category::FLIPPER_GROUSER | Category::FLIPPER_WHEEL | Category::TRACK | Category::FLIPPER | Category::OBSTACLE | Category::TERRAIN);
            this->meshes[scene.get<std::string>(k + ".name", k)] = m;
        }
        else break;
    }

    this->setObjectsPositions();

#ifdef OCTOMAP_FOUND
    if(false) {
        OcTreeBuilder occGridBuilder(this, 7, 10.);
        occGridBuilder.check();
        occGridBuilder.saveOcTree("test.bt");
    }
#endif
}

void Environment::destroy() {
    if(config.world.simulator_threads > 1) {
        dThreadingImplementationShutdownProcessing(this->threading);
        dThreadingFreeThreadPool(this->pool);
        dWorldSetStepThreadingImplementation(this->world, NULL, NULL);
        dThreadingFreeImplementation(this->threading);
    }

    if(this->v) this->v->destroy();

    BOOST_FOREACH(const TriMeshMap::value_type &v, this->meshes) {
        v.second->destroy();
    }
}

void Environment::setObjectsPositions() {
    boost::property_tree::ptree scene;
    boost::property_tree::ini_parser::read_ini(CONFIG_PATH "/scene.ini", scene);

    if(this->v) {
        dVector3 pos;
        readPosition(scene, "vehicle", pos);
        this->v->setPosition(pos);
        dMatrix3 R;
        readOrientation(scene, "vehicle", R);
        this->v->setRotation(R);
    }

    size_t meshIndex = 0;
    while(true) {
        std::string k = "mesh";
        k += boost::lexical_cast<std::string>(meshIndex++);
        boost::optional<std::string> file = scene.get_optional<std::string>(k + ".file");
        if(file) {
            std::string name = scene.get<std::string>(k + ".name", k);
            TriMeshPtr m = this->meshes[name];
            TriMesh::BoundsXYZ &b = m->bounds;
            dVector3 pos = {0., 0., 0.}, c = {(b.x.max + b.x.min) / 2, (b.y.max + b.y.min) / 2, b.z.min};
            readPosition(scene, k, pos);
            for(int i = 0; i < 3; i++) pos[i] -= c[i];
            dGeomSetPosition(m->geom, pos[0], pos[1], pos[2]);
            dMatrix3 R;
            dRSetIdentity(R);
            readOrientation(scene, k, R);
            dGeomSetRotation(m->geom, R);
        }
        else break;
    }
}

std::string Environment::getGeomName(dGeomID geom) const {
    std::map<dGeomID, std::string>::const_iterator it = geomNames.find(geom);
    if(it == geomNames.end())
        return boost::lexical_cast<std::string>(reinterpret_cast<unsigned long>(geom));
    else
        return it->second;
}

void Environment::setGeomName(dGeomID geom, const std::string &name) {
    geomNames[geom] = name;
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

static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->nearCallback(o1, o2);
}

void Environment::nearCallbackDefault(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_default.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = config.contact_default.bounce;
        contact[i].surface.bounce_vel = config.contact_default.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_default.soft_cfm;
        contact[i].surface.mu = config.contact_default.mu;
        contact[i].surface.mu2 = config.contact_default.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_default.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

bool Environment::step(dReal stepSize, int simulationStepsPerFrame) {
    stepNum++;

    if(this->v) this->v->step(stepSize);
    
    this->badCollision = false;
    this->contacts.clear();
    
    for(size_t i = 0; i < simulationStepsPerFrame; i++) {

        // find collisions and add contact joints
        dSpaceCollide(this->space, this, &nearCallbackWrapper);
        // step the simulation
//        dWorldStep(this->world, stepSize / (dReal)simulationStepsPerFrame);
        dWorldQuickStep(this->world, stepSize / (dReal)simulationStepsPerFrame);

        // remove all contact joints
        dJointGroupEmpty(this->contactGroup);
    }
    
    return this->badCollision;
}

bool Environment::step() {
    return step(config.step.step_size, config.step.simulation_steps_per_frame);
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
    // flag collision if there is really a collision and is of those not allowed
    if(numc > 0 && !isValidCollision(o1, o2, contact[0])) {
        badCollision = true;
        std::cout << "collision between " << getGeomName(o1) << "[" << dClassGetName(dGeomGetClass(o1)) << "] and " << getGeomName(o2) << "[" << dClassGetName(dGeomGetClass(o2)) << "]" << std::endl;
    }
}

bool Environment::evaluateCollision() {
    this->badCollision = false;
    dSpaceCollide(this->space, this, &evaluateCollisionNearCallbackWrapper);
    return this->badCollision;
}

void Environment::draw() {
    if(this->v) this->v->draw();

    if(config.show_contact_points) {
        dsSetColor(1, 0, 1);
        dMatrix3 R; dRSetIdentity(R);
        BOOST_FOREACH(dContactGeom cg, this->contacts) {
            dsDrawSphereD(cg.pos, R, 0.05);
        }
    }
    
    for(std::vector<dGeomID>::iterator it = boxes.begin(); it != boxes.end(); it++) {
        dsSetColorAlpha(0.6, 0.6, 0.7, 0.8);
        const dReal *pos = dGeomGetPosition(*it);
        const dReal *R = dGeomGetRotation(*it);
        dReal sides[3];
        dGeomBoxGetLengths(*it, sides);
        dsDrawBoxD(pos, R, sides);
    }

    BOOST_FOREACH(const TriMeshMap::value_type &v, this->meshes) {
        v.second->draw();
    }
}

