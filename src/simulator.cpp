//
//  simulator.cpp
//  tvs
//
//  Created by Federico Ferri on 13/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"
#include <SDL.h>
#include <SDL_joystick.h>
#include <TrackedVehicleEnvironment.h>
#include <TrackedVehicle.h>
#include <SimpleTrackedVehicleEnvironment.h>
#include <SimpleTrackedVehicle.h>

#if 1
#define VEHICLE_TYPE TrackedVehicle
#define ENVIRONMENT_TYPE TrackedVehicleEnvironment
#else
#define VEHICLE_TYPE SimpleTrackedVehicle
#define ENVIRONMENT_TYPE SimpleTrackedVehicleEnvironment
#endif

Environment *environment;
SDL_Joystick *joystick;
dReal vel_left = 0.0, vel_right = 0.0;
bool following = false;
dReal kbd_gain = 2.0;

bool flipperFLbuttonDown = false, flipperFRbuttonDown = false, flipperRLbuttonDown = false, flipperRRbuttonDown = false;
int flipperMovementSpeed = 0;
dReal linearSpeed = 0, angularSpeed = 0;


void initRobotPose() {
    static dVector3 p = {
            2.4554, 3.01316, 0.077984 + 0.0246
    };
    static dQuaternion q = {
            -0.767196, -1.83056e-06, 2.44949e-06, -0.641413
    };

    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
    vel_left = vel_right = 0.0;
    environment->v->setVelocities(0, 0);
}

void follow(dReal x, dReal y, dReal z) {
    float xyz[3], hpr[3];
    dsGetViewpoint(xyz, hpr);
    
    // ~lookAt:
    dReal dx = x - xyz[0], dy = y - xyz[1], dz = z - xyz[2];
    hpr[0] = atan2(dy, dx) * 180.0 / M_PI;
    hpr[1] = atan2(dz, hypot(dy, dx)) * 180.0 / M_PI;
    hpr[2] = 0;

    // ~followRobot:
    dReal d = hypot(dz, hypot(dx, dy));
    dReal dmin = 3.5, dmax = 4.3, k = 0.2;
    if(d > dmax) {
        dReal a = (d - dmax) * k;
        xyz[0] += dx * a;
        xyz[1] += dy * a;
        //xyz[2] += dz * a;
    } else if(d < dmin) {
        dReal a = (d - dmin) * k;
        xyz[0] += dx * a;
        xyz[1] += dy * a;
        //xyz[2] += dz * a;
    }
    
    dsSetViewpoint(xyz,hpr);
}

void start() {
    if(environment->config.joystick.enabled) {
        if(SDL_NumJoysticks() > 0) {
            std::cout << "Available joysticks:" << std::endl;
            for(int i=0; i < SDL_NumJoysticks(); i++)
                std::cout << i << ": " << SDL_JoystickNameForIndex(i) << std::endl;
            SDL_JoystickEventState(SDL_ENABLE);
            joystick = SDL_JoystickOpen(environment->config.joystick.device);
        } else {
            std::cout << "No joysticks available." << std::endl;
        }
    }
    static float xyz[3] = {9.3812,4.5702,2.8600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    if(environment->config.joystick.enabled) {
        SDL_Event event;
        dReal maxLinearSpeed;
        dReal maxAngularSpeed;
        dReal tracksDistance = 0.4;
        dReal steeringEfficiency = 0.5;

        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_JOYAXISMOTION:
                    maxLinearSpeed = environment->config.world.max_track_speed * environment->config.joystick.gain;
                    maxAngularSpeed = maxLinearSpeed * 2.0;
                    switch (event.jaxis.axis) {
                        case 0:
                            angularSpeed = maxAngularSpeed * event.jaxis.value / 32768.0;
                            break;
                        case 1:
                            linearSpeed = maxLinearSpeed * (-1 * event.jaxis.value) / 32768.0;
                            break;
                        case 2:
                            flipperRLbuttonDown = (event.jaxis.value >= -32767);
                            break;
                        case 5:
                            flipperRRbuttonDown = (event.jaxis.value >= -32767);
                            break;
                    }

                    vel_left = linearSpeed + angularSpeed * tracksDistance / 2 / steeringEfficiency;
                    vel_right = linearSpeed - angularSpeed * tracksDistance / 2 / steeringEfficiency;

                    if (fabs(vel_left) > maxLinearSpeed) {
                        vel_left = sgn(vel_left) * maxLinearSpeed;
                    }
                    if (fabs(vel_right) > maxLinearSpeed) {
                        vel_right = sgn(vel_right) * maxLinearSpeed;
                    }

//                    std::cout << (uint)event.jaxis.axis << " " << event.jaxis.value << std::endl;
                    break;
                case SDL_JOYBUTTONDOWN:
                    switch (event.jbutton.button) {
                        case 4:
                            flipperFLbuttonDown = true;
                            break;
                        case 5:
                            flipperFRbuttonDown = true;
                            break;
                    }
//                    std::cout << (uint)event.jbutton.button << " " << (uint)event.jbutton.state << std::endl;
                    break;
                case SDL_JOYBUTTONUP:
                    switch (event.jbutton.button) {
                        case 4:
                            flipperFLbuttonDown = false;
                            break;
                        case 5:
                            flipperFRbuttonDown = false;
                            break;
                    }
//                    std::cout << (uint)event.jbutton.button << " " << (uint)event.jbutton.state << std::endl;
                    break;
                case SDL_JOYHATMOTION:
                    if (event.jhat.hat == 0) {
                        switch (event.jhat.value) {
                            case 1:
                                flipperMovementSpeed = -1;
                                break;
                            case 4:
                                flipperMovementSpeed = 1;
                                break;
                            default:
                                flipperMovementSpeed = 0;
                        }
                    }
//                    std::cout << (uint)event.jhat.hat << " " << (uint)event.jhat.value << std::endl;
                    break;
            }
        }
    }
    if(following) {
        const dReal *p = environment->v->getPosition();
        follow(p[0], p[1], p[2]);
    }

    environment->v->setVelocities(kbd_gain * vel_left, kbd_gain * vel_right);

    VEHICLE_TYPE* tv = dynamic_cast<VEHICLE_TYPE*>(environment->v);
    if (tv != NULL) {
        tv->leftTrack->setFlipperAngularVelocity(0,  flipperFLbuttonDown * flipperMovementSpeed);
        tv->leftTrack->setFlipperAngularVelocity(1,  flipperRLbuttonDown * flipperMovementSpeed);
        tv->rightTrack->setFlipperAngularVelocity(0, flipperFRbuttonDown * flipperMovementSpeed);
        tv->rightTrack->setFlipperAngularVelocity(1, flipperRRbuttonDown * flipperMovementSpeed);
    }

    environment->draw();

    if(!pause) environment->step();
}

void stop() {
    if(environment->config.joystick.enabled) {
        SDL_JoystickClose(joystick);
        SDL_JoystickEventState(SDL_DISABLE);
    }
}

void printInfo() {
    const dReal *p = environment->v->getPosition();
    const dReal *q = environment->v->getQuaternion();
    const dReal *R = environment->v->getRotation();
    std::cout << "position: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "orientation: " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << std::endl;
    std::cout << "z: " << R[2] << ", " << R[6] << ", " << R[10] << std::endl;
}

void command(int cmd) {
    const dReal V = environment->config.world.max_track_speed;
    switch(cmd) {
        case 'd': vel_left = V; vel_right = -V; break;
        case 'a': vel_left = -V; vel_right = V; break;
        case 'w': vel_left = V; vel_right = V; break;
        case 's': vel_left = -V; vel_right = -V; break;
        case 'q': vel_left *= 0.33; break;
        case 'e': vel_right *= 0.33; break;
        case ' ': vel_left = 0; vel_right = 0; break;
        case 'f': following ^= 1; break;
        case 'c': environment->config.show_contact_points ^= 1; break;
        case 'p': printInfo(); break;
        case 'r': initRobotPose(); break;
    }
    if(cmd >= '1' && cmd <= '9') kbd_gain = (cmd - '1' + 1) / 5.0;
}

int main(int argc, char **argv) {
    if(SDL_Init(SDL_INIT_JOYSTICK)) {
        std::cout << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return 1;
    }
    SDL_GameControllerAddMappingsFromFile(CONFIG_PATH "/gamecontrollerdb.txt");

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new ENVIRONMENT_TYPE();
    environment->create();

    // set initial robot pose:
    initRobotPose();

    // run simulation loop & visualization
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 1920, 1000, &fn);

    // quit -> cleanup
    environment->destroy();
    delete environment;
    
    dCloseODE();
    
    SDL_Quit();

    return 0;
}
