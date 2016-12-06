#ifndef TRACKED_MOTION_TRACKBASE_H
#define TRACKED_MOTION_TRACKBASE_H


#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "TrackKinematicModel.h"
#include "ODEUtils.h"
#include "Environment.h"

#define USE_GUIDE_GEOMS false
#define NUM_GUIDE_GEOMS 2

class Environment;

class TrackBase {
public:
    Environment *environment;
    std::string name;
    TrackKinematicModel *m;
    dReal density;
    dBodyID trackBody;
    dMass trackMass;
    dBodyID wheelBody[2];
    dMass wheelMass[2];
    dGeomID wheelGeom[2];
    dJointID wheelJoint[2];
#if USE_GUIDE_GEOMS
    dGeomID guideGeom[NUM_GUIDE_GEOMS];
#else
    dJointID* guideJoints;
#endif
    size_t drivingWheelIndex;
    dBodyID *linkBody;
    dGeomID *linkGeom;
    dGeomID *grouserGeom;
    dJointID *linkJoint;
    dMass *linkMass;
    size_t numGrousers;
    dReal linkThickness;
    dReal grouserHeight;
    dRigidBodyArrayID bodyArray;

    TrackBase(Environment *environment_, const std::string &name_, size_t drivingWheelIndex, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_);
    virtual ~TrackBase();
    virtual void create();
    virtual void destroy();
    virtual void draw();
    virtual void step(dReal stepSize) = 0;
    virtual void setVelocity(dReal velocity) = 0;

protected:
    virtual Category::Category getWheelCategory() = 0;
    virtual Category::Category getGuideCategory() = 0;
    virtual Category::Category getGrouserCategory() = 0;
};


#endif //TRACKED_MOTION_TRACKBASE_H
