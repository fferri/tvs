//
//  planner.cpp
//  tvs
//
//  Created by Federico Ferri on 13/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <iostream>
#include <fstream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"
#include <TrackedVehicleEnvironment.h>
#include <TrackedVehicle.h>
#include <SimpleTrackedVehicleEnvironment.h>
#include <SimpleTrackedVehicle.h>
#include "OMPLTVSSimpleSetup.h"
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>

#if 1
#define VEHICLE_TYPE TrackedVehicle
#define ENVIRONMENT_TYPE TrackedVehicleEnvironment
#else
#define VEHICLE_TYPE SimpleTrackedVehicle
#define ENVIRONMENT_TYPE SimpleTrackedVehicleEnvironment
#endif

Environment *environment;

int nstep = 0;
ompl::control::PathControl *path;
OMPLTVSStateSpace *ss;
OMPLTVSEnvironmentPtr ompl_env;

void start() {
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    environment->draw();
    
    // draw search tree
    for(std::vector<dLine>::iterator i = ompl_env->searchTree.begin(); i != ompl_env->searchTree.end(); i++) {
        dsDrawLineD(i->a, i->b);
    }
}

void stop() {
}

void setFrame(int d) {
    if(!path) return;
    int newnstep = nstep + d;
    if(newnstep < 0 || newnstep >= path->getStateCount() || newnstep == nstep)
        return;
    nstep = newnstep;
    std::cout << "STEP " << (1+nstep) << "/" << path->getStateCount() << std::endl;
    
    ompl::base::State *state = path->getState(nstep);
    ss->writeState(state);
}

void command(int cmd) {
    switch(cmd) {
        case '+':case '=': setFrame(1); break;
        case '-':case '_': setFrame(-1); break;
    }
}

int main(int argc, char **argv) {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new ENVIRONMENT_TYPE();
    environment->create();

    // set initial robot pose:
    static dVector3 p = {
        1.4554, 3.01316, 0.077984+0.0246
    };
    static dQuaternion q = {
        -0.767196, -1.83056e-06, 2.44949e-06, -0.641413
    };
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);

    // set up planning environment:
    ompl_env = OMPLTVSEnvironmentPtr(new OMPLTVSEnvironment(environment));
    ompl::base::StateSpacePtr stateSpace(ss = new OMPLTVSStateSpace(ompl_env));
    OMPLTVSSimpleSetup setup(stateSpace);
    setup.setGoalRegion(7.74, 0.95, 1.4, 0.4);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -2); bounds.setHigh(0, 8);
    bounds.setLow(1,  0); bounds.setHigh(1, 9);
    bounds.setLow(2,  -0.1); bounds.setHigh(2, 2);
    stateSpace->as<OMPLTVSStateSpace>()->setVolumeBounds(bounds);
    ompl::control::SpaceInformationPtr si = setup.getSpaceInformation();
    ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(si));
    //ompl::base::PlannerPtr planner(new ompl::control::RRT(si));
    //ompl::base::PlannerPtr planner(new ompl::control::EST(si));
    //ompl::base::PlannerPtr planner(new ompl::control::SyclopRRT(si));
    //ompl::base::PlannerPtr planner(new ompl::control::SyclopEST(si));
    //ompl::base::PlannerPtr planner(new ompl::control::PDST(si));
    setup.setPlanner(planner);
    setup.setup();
    if (setup.solve(46800)) {
        path = new ompl::control::PathControl(setup.getSolutionPath());
        std::cout << "SOLUTION LENGTH: " << path->getStateCount() << std::endl;
        setFrame(0);
        //path->printAsMatrix(std::cout);
        
        // save solution to file:
        std::ofstream solutionFile;
        solutionFile.open(("solution-" + environment->datetime + ".csv").c_str());
        for(int i = 0; i < path->getStateCount(); i++) {
            std::vector<double> row;
            ss->copyToReals(row, path->getState(i));
            for(int j = 0; j < row.size(); j++)
                solutionFile << (j > 0 ? "," : "") << row[j];
            solutionFile << std::endl;
        }
        solutionFile.close();
    } else {
        std::cout << "SOLUTION NOT FOUND" << std::endl;
        path = 0L;
    }
    
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    if(path) delete path;
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
