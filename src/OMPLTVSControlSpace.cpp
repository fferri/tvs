//
//  OMPLTVSControlSpace.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSControlSpace.h"
#include <ompl/util/Exception.h>
#include <ompl/util/Console.h>

void OMPLTVSControlSampler::sample(ompl::control::Control *control) {
    const ompl::base::RealVectorBounds &bounds = space_->as<ompl::control::RealVectorControlSpace>()->getBounds();
    ompl::control::RealVectorControlSpace::ControlType *rcontrol = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    int l = 0, r = 0, d = 1;
    while(l <= 0 && r <= 0) {
        l = rng_.uniformInt(-d, d);
        r = rng_.uniformInt(-d, d);
    }
    double f = (bounds.high[0] - bounds.low[0]) / (2.0 * d);
    rcontrol->values[0] = f * l;
    rcontrol->values[1] = f * r;
}

static const OMPLTVSEnvironmentPtr& getOMPLTVSStateSpaceEnvironmentWithCheck(const ompl::base::StateSpacePtr &space) {
    if(!dynamic_cast<OMPLTVSStateSpace*>(space.get()))
        throw ompl::Exception("OMPLTVS State Space needed for creating OMPLTVS Control Space");
    return space->as<OMPLTVSStateSpace>()->getEnvironment();
}

OMPLTVSControlSpace::OMPLTVSControlSpace(const ompl::base::StateSpacePtr &stateSpace)
: RealVectorControlSpace(stateSpace, getOMPLTVSStateSpaceEnvironmentWithCheck(stateSpace)->getControlDimension()) {
    setName("OMPLTVS" + getName());
    type_ = ompl::control::CONTROL_SPACE_TYPE_COUNT + 1;
    ompl::base::RealVectorBounds bounds(dimension_);
    getEnvironment()->getControlBounds(bounds.low, bounds.high);
    setBounds(bounds);
}
