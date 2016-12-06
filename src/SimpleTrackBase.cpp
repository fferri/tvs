#include "SimpleTrackBase.h"

#include "Environment.h"
#include "PlanarJoint.h"
#include <drawstuff/drawstuff.h>

SimpleTrackBase::SimpleTrackBase(Environment *environment_, const std::string &name_, dReal rearRadius, dReal frontRadius, dReal betweenWheelsDistance, dReal trackDepth, unsigned long additionalCategory) : environment(environment_), name(name_) {
    this->rearRadius = rearRadius;
    this->frontRadius = frontRadius;
    this->betweenWheelsDistance = betweenWheelsDistance;
    this->trackDepth = trackDepth;
    this->additionalCategory = additionalCategory;

    this->density = 1.0;
}

SimpleTrackBase::~SimpleTrackBase() {
}

void SimpleTrackBase::create() {
    this->trackBody = dBodyCreate(this->environment->world);
    this->bodyArray = dRigidBodyArrayCreate(this->trackBody);

    // approximate computation, but should be sufficient
    dMassSetBox(&this->trackMass,
                this->density,
                // to compute the length of a rectangle that has area equal to the rectangle with circles added
                // on both ends, we take the rectangle length + lengths corresponding to a half-area section of the
                // semi-circles; this is known as the quarter-tank problem, and the approximate solution is 0.596*radius
                this->betweenWheelsDistance + (1-0.596) * this->rearRadius + (1-0.596) * this->frontRadius,
                // it is basically a symmetric trapezoid, so the height can be approximated as the mean of both sides
                (this->rearRadius + this->frontRadius) / 2,
                this->trackDepth);
    dBodySetMass(this->trackBody, &this->trackMass);

    dReal l = this->betweenWheelsDistance; // distance between wheel centers
    dReal r1 = this->rearRadius; // radius of the larger wheel
    dReal r2 = this->frontRadius; // radius of the smaller wheel

    const dReal trackEdgeAngle = atan2(r1 - r2, l);
    for(int w = 0; w < 2; w++) {
        const int coef = (w == 0 ? 1 : -1);

        this->trackGeom[w] = dCreateBox(this->environment->space, l, this->trackDepth, r2);
        dGeomSetBody(this->trackGeom[w], this->trackBody);
        this->environment->setGeomName(this->trackGeom[w], this->name + ".trackGeom" + boost::lexical_cast<std::string>(w));

        dGeomSetCategoryBits(this->trackGeom[w], getGrouserCategory());
        dGeomSetCollideBits(this->trackGeom[w], Category::TERRAIN);

        dMatrix3 trackEdgeRotation;
        dRFromAxisAndAngle(trackEdgeRotation, 0, 1, 0, coef * trackEdgeAngle);
        dGeomSetOffsetPosition(this->trackGeom[w], 0.5 * l, 0, coef * ((r1 - r2) / 2 + r2 / 2));
        dGeomSetOffsetRotation(this->trackGeom[w], trackEdgeRotation);
    }

    const dReal radii[2] = {this->rearRadius, this->frontRadius};
    for(int w = 0; w < 2; w++) {
        this->wheelGeom[w] = dCreateCylinder(this->environment->space, radii[w], this->trackDepth);
        this->environment->setGeomName(this->wheelGeom[w], this->name + ".wheel" + boost::lexical_cast<std::string>(w));
        dGeomSetBody(this->wheelGeom[w], this->trackBody);
        dGeomSetCategoryBits(this->wheelGeom[w], getGrouserCategory());
        dGeomSetCollideBits(this->wheelGeom[w], Category::TERRAIN);

        dMatrix3 wheelRotation;
        dRFromZAxis(wheelRotation, 0, 1, 0);

        dGeomSetOffsetPosition(this->wheelGeom[w], w * this->betweenWheelsDistance, 0, 0);
        dGeomSetOffsetRotation(this->wheelGeom[w], wheelRotation);
    }
}

void SimpleTrackBase::destroy() {
    dRigidBodyArrayDestroy(this->bodyArray);
    dBodyDestroy(this->trackBody);
    for(int w = 0; w < 2; w++) {
        dGeomDestroy(this->trackGeom[w]);
        dGeomDestroy(this->wheelGeom[w]);
    }
}

void SimpleTrackBase::draw() {
    dsSetColor(1, 1, 0);

    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->trackGeom[w]);
        const dReal *R = dGeomGetRotation(this->trackGeom[w]);
        dVector3 boxSize;
        dGeomBoxGetLengths(this->trackGeom[w], boxSize);
        dsDrawBoxD(pos, R, boxSize);
    }

    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->wheelGeom[w]);
        const dReal *R = dGeomGetRotation(this->wheelGeom[w]);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheelGeom[w], &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }
}
