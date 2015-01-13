//
//  simulator.cpp
//  tvs
//
//  Created by Federico Ferri on 13/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"

Environment *environment;

void start() {
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    environment->draw();
    if(!pause) environment->step();
}

void stop() {
}

void printInfo() {
    const dReal *p = environment->v->getPosition();
    const dReal *q = environment->v->getQuaternion();
    std::cout << "position: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "orientation: " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << std::endl;
}

void command(int cmd) {
    const dReal V = environment->config.world.max_track_speed;
    switch(cmd) {
        case 'd': environment->v->setTrackVelocities( V, -V); break;
        case 'a': environment->v->setTrackVelocities(-V,  V); break;
        case 'w': environment->v->setTrackVelocities(-V, -V); break;
        case 's': environment->v->setTrackVelocities( V,  V); break;
        case 'e': environment->v->setTrackVelocities(-0.25*V, -V); break;
        case 'q': environment->v->setTrackVelocities(-V, -0.25*V); break;
        case ' ': environment->v->setTrackVelocities( 0,  0); break;
        case 'p': printInfo(); break;
    }
}

int main(int argc, char **argv) {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new Environment();
    environment->create();

    // set initial robot pose:
    static dVector3 p = {2.08086,3.39581,0.102089};
    static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);

    // run simulation loop & visualization
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    // quit -> cleanup
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
