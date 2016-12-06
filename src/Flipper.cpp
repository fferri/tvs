#include "Flipper.h"
#include "Environment.h"
#include <drawstuff/drawstuff.h>

Flipper::Flipper(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_) : TrackBase(environment_, name_, 0, radius1_, radius2_, distance_, numGrousers_, linkThickness_, grouserHeight_, trackDepth_) {

}

Flipper::~Flipper() {

}

void Flipper::create() {
    TrackBase::create();

#if NUM_FLIPPER_STRUT_GEOMS >= 1
    dReal l = this->m->distance; // distance between wheel centers
    dReal r1 = this->m->radius[0]; // radius of the larger wheel
    dReal r2 = this->m->radius[1]; // radius of the smaller wheel

    dReal flipperEdgeAngle = atan2(r1-r2, l);
    dMatrix3 flipperEdgeRotation;
    dRFromAxisAndAngle(flipperEdgeRotation, 0, 1, 0, flipperEdgeAngle);

    this->strutGeoms[0] = dCreateBox(this->environment->space, l, 0.95*this->m->trackDepth, r2);
    this->environment->setGeomName(this->strutGeoms[0], this->name + ".strut1");
    dGeomSetCategoryBits(this->strutGeoms[0], Category::FLIPPER_GUIDE);
    dGeomSetCollideBits(this->strutGeoms[0], Category::FLIPPER_GROUSER);
    dGeomSetBody(this->strutGeoms[0], this->trackBody);
    dGeomSetOffsetPosition(this->strutGeoms[0], 0.5 * this->m->distance, 0, (r1-r2)/2+r2/2);
    dGeomSetOffsetRotation(this->strutGeoms[0], flipperEdgeRotation);

#if NUM_FLIPPER_STRUT_GEOMS >= 2
    dRFromAxisAndAngle(flipperEdgeRotation, 0, 1, 0, -flipperEdgeAngle);

    this->strutGeoms[1] = dCreateBox(this->environment->space, l, 0.95*this->m->trackDepth, r2);
    this->environment->setGeomName(this->strutGeoms[1], this->name + ".strut2");
    dGeomSetCategoryBits(this->strutGeoms[1], Category::FLIPPER_GUIDE);
    dGeomSetCollideBits(this->strutGeoms[1], Category::FLIPPER_GROUSER);
    dGeomSetBody(this->strutGeoms[1], this->trackBody);
    dGeomSetOffsetPosition(this->strutGeoms[1], 0.5 * this->m->distance, 0, -(r1-r2)/2-r2/2);
    dGeomSetOffsetRotation(this->strutGeoms[1], flipperEdgeRotation);
#endif
#endif
}

void Flipper::destroy() {
    TrackBase::destroy();
}

void Flipper::step(dReal stepSize) {
    this->velocity.step(stepSize);
    dJointSetHingeParam(this->wheelJoint[this->drivingWheelIndex], dParamVel, this->velocity.get());
}

void Flipper::draw() {
    TrackBase::draw();

    dsSetColorAlpha(0, 0, 0, 1);
    for(int w = 0; w < NUM_FLIPPER_STRUT_GEOMS; w++) {
        const dReal *pos = dGeomGetPosition(this->strutGeoms[w]);
        const dReal *R = dGeomGetRotation(this->strutGeoms[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->strutGeoms[w], sides);
        dsDrawBoxD(pos, R, sides);
    }

#if USE_GUIDE_GEOMS
#define DEBUG_DRAW_GROUSER_GUIDES
#ifdef DEBUG_DRAW_GROUSER_GUIDES
    dsSetColorAlpha(0, 1, 0, 0.3);
    for(int w = 0; w < NUM_GUIDE_GEOMS; w++) {
        const dReal *pos = dGeomGetPosition(this->guideGeom[w]);
        const dReal *R = dGeomGetRotation(this->guideGeom[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->guideGeom[w], sides);
        dsDrawBoxD(pos, R, sides);
    }
#endif
#endif
}

void Flipper::setVelocity(dReal velocity) {
    this->velocity.set(velocity);
}

Category::Category Flipper::getWheelCategory() {
    return Category::FLIPPER_WHEEL;
}

Category::Category Flipper::getGuideCategory() {
    return Category::FLIPPER_GUIDE;
}

Category::Category Flipper::getGrouserCategory() {
    return Category::FLIPPER_GROUSER;
}
