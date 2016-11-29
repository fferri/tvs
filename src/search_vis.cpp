//
//  search_vis.cpp
//  tvs
//
//  Created by Federico Ferri on 15/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"
#include <TrackedVehicleEnvironment.h>
#include <TrackedVehicle.h>
#include <SimpleTrackedVehicleEnvironment.h>
#include <SimpleTrackedVehicle.h>
#include "OMPLTVSSimpleSetup.h"
#include <boost/filesystem.hpp>
#include <fstream>

#if 1
#define VEHICLE_TYPE TrackedVehicle
#define ENVIRONMENT_TYPE TrackedVehicleEnvironment
#else
#define VEHICLE_TYPE SimpleTrackedVehicle
#define ENVIRONMENT_TYPE SimpleTrackedVehicleEnvironment
#endif

Environment *environment;
size_t old_len = 0;
int last_newline = -1;
const char *f = 0L;
std::vector<dLine> searchTree;

void start() {
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    size_t len = boost::filesystem::file_size(f);
    if(len > old_len) {
        int pos = last_newline;
        char c;
        std::ifstream fs(f);
        fs.seekg(last_newline + 1);
        std::string buf;
        std::vector<float> line;
        while(fs.get(c)) {
            //std::cout << "got char: " << c << std::endl;
            if(c == ',' || c == '\n') {
                float value = std::atof(buf.c_str());
                //std::cout << "got token: \"" << buf << "\" (" << value << ")" << std::endl;
                buf.clear();
                line.push_back(value);
            } else {
                buf += c;
            }
            if(c == '\n') {
                //std::cout << "got line:";
                //for(int i = 0; i < line.size(); i++)
                //    std::cout << " " << boost::lexical_cast<std::string>(line[i]);
                //std::cout << std::endl;
                searchTree.resize(searchTree.size() + 1);
                searchTree.back().a[0] = line[0];
                searchTree.back().a[1] = line[1];
                searchTree.back().a[2] = line[2];
                searchTree.back().b[0] = line[6];
                searchTree.back().b[1] = line[7];
                searchTree.back().b[2] = line[8];
                line.clear();
                last_newline = pos + 1;
            }
            pos++;
        }
        old_len = len;
    }
    environment->draw();
    
    // draw search tree
    const dReal w0 = 35, w1 = 21, w2 = 7;
    for(size_t i = 0; i < searchTree.size(); i++) {
        dReal r = fmax(0, i + w0 - searchTree.size()) / w0;
        dReal g = fmax(0, i + w1 - searchTree.size()) / w1;
        dReal b = fmax(0, i + w2 - searchTree.size()) / w2;
        dsSetColor(r, g, b);
        dsDrawLineD(searchTree[i].a, searchTree[i].b);
    }
}

void stop() {
}

void command(int cmd) {
}

int main(int argc, char **argv) {
    if(argc != 2) {
        std::cout << "usage: " << argv[0] << " <tree.csv>" << std::endl;
        exit(1);
    }
    
    f = argv[1];
    
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new SimpleTrackedVehicleEnvironment();
    environment->create();

    // set initial robot pose:
    static dVector3 p = {2.08086,3.39581,0.102089};
    static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
    
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
