//
//  Environment.h
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef WORLD_H_INCLUDED
#define WORLD_H_INCLUDED

#include <ode/ode.h>
#include <string>
#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "Vehicle.h"
#include "TriMesh.h"

struct ContactParams {
    int max_contacts;
    dReal bounce;
    dReal bounce_vel;
    dReal soft_cfm;
    dReal mu;
    dReal mu2;
    bool debug;
};

struct StepParams {
    dReal step_size;
    int simulation_steps_per_frame;
};

struct WorldParams {
    dReal gravity_x, gravity_y, gravity_z;
    dReal max_track_speed;
    dReal track_acceleration;
    int simulator_threads;
};

struct JoystickParams {
    unsigned short enabled;
    unsigned short device;
    dReal gain;
};

struct Config {
    StepParams step;
    WorldParams world;
    JoystickParams joystick;
    ContactParams contact_wheel_grouser;
    ContactParams contact_grouser_terrain;
    ContactParams contact_grouser_guide;
    ContactParams contact_default;
    bool show_contact_points;
};

#ifndef CONFIG_PATH
#define CONFIG_PATH "."
#endif

typedef std::map<std::string, TriMeshPtr> TriMeshMap;

struct Object {
    dGeomID geom;
    struct {
        float r, g, b, a;
    } color;
};
typedef std::map<std::string, Object> ObjectMap;

class Environment {
public:
    Config config;
    std::string datetime;

    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;
    dThreadingThreadPoolID pool;
    dThreadingImplementationID threading;
    std::map<dGeomID, std::string> geomNames;

    dGeomID planeGeom;
    Vehicle *v;
    std::vector<dGeomID> boxes;
    TriMeshMap meshes;
    ObjectMap objects;
    
    bool badCollision;
    std::vector<dContactGeom> contacts;
    size_t stepNum;
    
    Environment();
    virtual ~Environment();
    void readConfig();
    void readSceneINI(std::string file);
    void create();
    void destroy();
    std::string getGeomName(dGeomID geom) const;
    void setGeomName(dGeomID geom, const std::string &name);
    bool isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2);
    virtual bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) = 0;

    virtual void nearCallback(dGeomID o1, dGeomID o2) = 0;
    void nearCallbackDefault(dGeomID o1, dGeomID o2);
    bool step(dReal stepSize, int simulationStepsPerFrame);
    bool step();
    void evaluateCollisionNearCallback(dGeomID o1, dGeomID o2);
    bool evaluateCollision();

    virtual void draw();
};

namespace Category { enum Category {
    TERRAIN  = 1 << 0,
    TRACK_GROUSER = 1 << 1,
    TRACK_GUIDE = 1 << 2,
    TRACK_WHEEL = 1 << 3,
    OBSTACLE = 1 << 4,
    FLIPPER_GROUSER = 1 << 5,
    FLIPPER_GUIDE = 1 << 6,
    FLIPPER_WHEEL = 1 << 7,
    TRACK = 1 << 8,
    FLIPPER = 1 << 9,
    LEFT = 1 << 10,
    RIGHT = 1 << 11,
    FRONT = 1 << 12,
    REAR = 1 << 13
}; };

#endif // WORLD_H_INCLUDED
