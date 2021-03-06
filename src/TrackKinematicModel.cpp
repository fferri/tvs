//
//  TrackKinematicModel.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "TrackKinematicModel.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <drawstuff/drawstuff.h>

TrackKinematicModel::TrackKinematicModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_) {
    this->radius[0] = radius1_;
    this->radius[1] = radius2_;
    this->distance = distance_;
    this->numGrousers = numGrousers_;
    this->grouserHeight = grouserHeight_;
    this->trackDepth = trackDepth_;
    this->radiusDiff = this->radius[0] - this->radius[1];
    this->pDistance = sqrt(pow(this->distance, 2) - pow(this->radiusDiff, 2));
    this->theta = atan2(this->pDistance, this->radiusDiff);
    this->px[0] = this->radius[0] * cos(this->theta);
    this->py[0] = this->radius[0] * sin(this->theta);
    this->px[1] = this->distance + this->radius[1] * cos(this->theta);
    this->py[1] = this->radius[1] * sin(this->theta);
    this->arc1Length = this->radius[0] * 2 * (M_PI - this->theta);
    this->arc2Length = this->radius[1] * 2 * this->theta;
    this->totalLength = 0.0;
    this->dlimits[0] = this->arc1Length;
    this->dlimits[1] = this->pDistance;
    this->dlimits[2] = this->arc2Length;
    this->dlimits[3] = this->pDistance;
    for(size_t i = 0; i < 4; i++)
        this->totalLength += this->dlimits[i];
    this->grouserWidth = this->totalLength / (double)this->numGrousers;
}

TrackKinematicModel::~TrackKinematicModel() {
}

void TrackKinematicModel::getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_) {
    // compute which piece of the path u touches
    // and compute local parameter v
    size_t i, section = -1;
    dReal v = NAN;
    dReal u1 = (u - floor(u)) * this->totalLength;
    for(i = 0; i < 4; i++) {
        if(u1 < this->dlimits[i]) {
            v = u1 / this->dlimits[i];
            section = i;
            break;
        }
        u1 -= this->dlimits[i];
    }
    // compute point and angle on path
    switch(section) {
        case 0:
            *theta_ = (1 - v) * (2 * M_PI - this->theta) + v * this->theta;
            *x_ = this->radius[0] * cos(*theta_);
            *y_ = this->radius[0] * sin(*theta_);
            break;
        case 1:
            *theta_ = this->theta;
            *x_ = (1 - v) * this->px[0] + v * this->px[1];
            *y_ = (1 - v) * this->py[0] + v * this->py[1];
            break;
        case 2:
            *theta_ = (1 - v) * this->theta + v * -this->theta;
            *x_ = this->distance + this->radius[1] * cos(*theta_);
            *y_ = this->radius[1] * sin(*theta_);
            break;
        case 3:
            *theta_ = -this->theta;
            *x_ = (1 - v) * this->px[1] + v * this->px[0];
            *y_ = (1 - v) * -this->py[1] + v * -this->py[0];
            break;
    }
}

void TrackKinematicModel::computeGrouserTransform3D(size_t i, dReal *pos, dReal *R) {
    // 2d track is on XZ plane, with roller axes in Y direction
    pos[1] = 0;
    dReal theta;
    getPointOnPath(i / (dReal)this->numGrousers, &pos[0], &pos[2], &theta);
    dRFromAxisAndAngle(R, 0, 1, 0, -theta);
}
