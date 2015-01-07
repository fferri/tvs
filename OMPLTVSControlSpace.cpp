/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "OMPLTVSControlSpace.h"
#include <ompl/util/Exception.h>
#include <ompl/util/Console.h>

void ompl::control::OMPLTVSControlSampler::sample(ompl::control::Control *control)
{
    const ompl::base::RealVectorBounds &bounds = space_->as<ompl::control::RealVectorControlSpace>()->getBounds();
    ompl::control::RealVectorControlSpace::ControlType *rcontrol = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    int l = 0, r = 0;
    while(l == 0 && r == 0) {
        l = rng_.uniformInt(-1, 1);
        r = rng_.uniformInt(-1, 1);
    }
    double f = (bounds.high[0] - bounds.low[0]) / 2.0;
    rcontrol->values[0] = f * l;
    rcontrol->values[1] = f * r;
    std::cout << "OMPLTVSControlSampler sampled (" << rcontrol->values[0] << ", " << rcontrol->values[1] << ")" << std::endl;
}

/// @cond IGNORE
namespace ompl
{
    const control::OMPLTVSEnvironmentPtr& getOMPLTVSStateSpaceEnvironmentWithCheck(const base::StateSpacePtr &space)
    {
        if (!dynamic_cast<control::OMPLTVSStateSpace*>(space.get()))
            throw Exception("OMPLTVS State Space needed for creating OMPLTVS Control Space");
        return space->as<control::OMPLTVSStateSpace>()->getEnvironment();
    }
}
/// @endcond

ompl::control::OMPLTVSControlSpace::OMPLTVSControlSpace(const base::StateSpacePtr &stateSpace) :
    RealVectorControlSpace(stateSpace, getOMPLTVSStateSpaceEnvironmentWithCheck(stateSpace)->getControlDimension())
{
    setName("OMPLTVS" + getName());
    type_ = CONTROL_SPACE_TYPE_COUNT + 1;
    base::RealVectorBounds bounds(dimension_);
    getEnvironment()->getControlBounds(bounds.low, bounds.high);
    setBounds(bounds);
}
