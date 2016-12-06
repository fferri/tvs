#include "TrackBase.h"

#include "Environment.h"
#include "PlanarJoint.h"
#include <drawstuff/drawstuff.h>

TrackBase::TrackBase(Environment *environment_, const std::string &name_, size_t drivingWheelIndex, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_) : environment(environment_), name(name_) {
    this->m = new TrackKinematicModel(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
    this->drivingWheelIndex = drivingWheelIndex;
    this->density = 1.0;
    this->linkBody = new dBodyID[numGrousers_];
    this->linkGeom = new dGeomID[numGrousers_];
    this->grouserGeom = new dGeomID[numGrousers_];
    this->linkJoint = new dJointID[numGrousers_];
    this->linkMass = new dMass[numGrousers_];
#if !USE_GUIDE_GEOMS
    this->guideJoints = new dJointID[numGrousers_];
#endif
    this->numGrousers = numGrousers_;
    this->linkThickness = linkThickness_;
    this->grouserHeight = grouserHeight_;
}

TrackBase::~TrackBase() {
    delete [] this->linkBody;
    delete [] this->linkGeom;
    delete [] this->grouserGeom;
    delete [] this->linkJoint;
    delete [] this->linkMass;
    delete this->m;
}

void TrackBase::create() {
    this->trackBody = dBodyCreate(this->environment->world);
    this->bodyArray = dRigidBodyArrayCreate(this->trackBody);
    dMassSetBox(&this->trackMass, this->density, this->m->distance, this->m->radius[1], this->m->trackDepth);
    dBodySetMass(this->trackBody, &this->trackMass);

    dVector3 trackNormal = {0, 1, 0};

    for(int w = 0; w < 2; w++) {
        this->wheelGeom[w] = dCreateCylinder(this->environment->space, this->m->radius[w], this->m->trackDepth);
        this->environment->setGeomName(this->wheelGeom[w], this->name + ".wheel" + boost::lexical_cast<std::string>(w));
        dGeomSetCategoryBits(this->wheelGeom[w], getWheelCategory());
        dGeomSetCollideBits(this->wheelGeom[w], getGrouserCategory() | Category::TERRAIN);
        dMassSetCylinder(&this->wheelMass[w], this->density, 3, this->m->radius[w], this->m->trackDepth);

        this->wheelBody[w] = dBodyCreate(this->environment->world);
        dRigidBodyArrayAdd(this->bodyArray, this->wheelBody[w]);
        dBodySetMass(this->wheelBody[w], &this->wheelMass[w]);
        dGeomSetBody(this->wheelGeom[w], this->wheelBody[w]);
        dBodySetPosition(this->wheelBody[w], w * this->m->distance, 0, 0);
        dMatrix3 wheelR;
        dRFromZAxis(wheelR, 0, 1, 0);
        dBodySetRotation(this->wheelBody[w], wheelR);

        this->wheelJoint[w] = dJointCreateHinge(this->environment->world, 0);
        dJointAttach(this->wheelJoint[w], this->trackBody, this->wheelBody[w]);
        dJointSetHingeAnchor(this->wheelJoint[w], w * this->m->distance, 0, 0);
        dJointSetHingeAxis(this->wheelJoint[w], 0, 1, 0);

#if USE_GUIDE_GEOMS
        // this guide should avoid tracks slipping out of their designed place
        dReal gh = 2 * (0.2 + std::max(this->m->radius[0], this->m->radius[1]));
        dReal gw = gh + this->m->distance;
        this->guideGeom[w] = dCreateBox(this->environment->space, gw, 0.01, gh);
        this->environment->setGeomName(this->guideGeom[w], this->name + ".grouser_guide" + boost::lexical_cast<std::string>(w));
        dGeomSetCategoryBits(this->guideGeom[w], getGuideCategory());
        dGeomSetCollideBits(this->guideGeom[w], getGrouserCategory());
        dGeomSetBody(this->guideGeom[w], this->trackBody);
        dGeomSetOffsetPosition(this->guideGeom[w], 0.5 * this->m->distance, (0.02 + this->m->trackDepth) * (w - 0.5), 0.0);
#endif
    }

    const dReal fMax = 5.0;

    dJointSetHingeParam(this->wheelJoint[this->drivingWheelIndex], dParamFMax, fMax);

    // grouser shrink/grow factor
    const dReal f = 1.03;

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        this->linkBody[i] = dBodyCreate(this->environment->world);
        dRigidBodyArrayAdd(this->bodyArray, this->linkBody[i]);
        dMassSetBox(&this->linkMass[i], this->density, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        dBodySetMass(this->linkBody[i], &this->linkMass[i]);

        dVector3 pos; dMatrix3 R;
        this->m->computeGrouserTransform3D(i, pos, R);

        this->linkGeom[i] = dCreateBox(this->environment->space, this->linkThickness, this->m->trackDepth, f * this->m->grouserWidth);
        this->environment->setGeomName(this->linkGeom[i], this->name + ".grouser" + boost::lexical_cast<std::string>(i));
        dGeomSetCategoryBits(this->linkGeom[i], getGrouserCategory());
        dGeomSetCollideBits(this->linkGeom[i], Category::TERRAIN | getWheelCategory() | Category::OBSTACLE | getGuideCategory());
        dGeomSetBody(this->linkGeom[i], this->linkBody[i]);

        this->grouserGeom[i] = dCreateBox(this->environment->space, this->m->grouserHeight, this->m->trackDepth, this->linkThickness);
        this->environment->setGeomName(this->grouserGeom[i], this->name + ".grouserTooth" + boost::lexical_cast<std::string>(i));
        dGeomSetCategoryBits(this->grouserGeom[i], getGrouserCategory());
        dGeomSetCollideBits(this->grouserGeom[i], Category::TERRAIN | getWheelCategory() | Category::OBSTACLE | getGuideCategory());
        dGeomSetBody(this->grouserGeom[i], this->linkBody[i]);
        dGeomSetOffsetPosition(this->grouserGeom[i], 0.5 * (this->linkThickness + this->m->grouserHeight), 0, 0);

        dBodySetPosition(this->linkBody[i], pos[0], pos[1], pos[2]);
        dBodySetRotation(this->linkBody[i], R);

#if !USE_GUIDE_GEOMS
        this->guideJoints[i] = dJointCreatePlanar(this->environment->world, 0);
        dJointAttach(this->guideJoints[i], this->linkBody[i], this->trackBody);
        dVector3 anchor = {0, 0, 0};
        dJointPlanarSetAnchor(this->guideJoints[i], pos);
        dJointPlanarSetPlaneNormal(this->guideJoints[i], trackNormal);
#endif
    }

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        size_t j = (i + 1) % this->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        this->m->getPointOnPath(i / (dReal)this->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2);
        dz = sin(a - M_PI_2);
        qx = px - this->m->grouserWidth * f * 0.5 * dx;
        qz = pz - this->m->grouserWidth * f * 0.5 * dz;
        px = px + this->m->grouserWidth * f * 0.5 * dx;
        pz = pz + this->m->grouserWidth * f * 0.5 * dz;
        this->linkJoint[i] = dJointCreateHinge(this->environment->world, 0);
        dJointAttach(this->linkJoint[i], this->linkBody[i], this->linkBody[j]);
        dJointSetHingeAnchor(this->linkJoint[i], px, 0, pz);
        dJointSetHingeAxis(this->linkJoint[i], 0, 1, 0);
    }
}

void TrackBase::destroy() {
    dRigidBodyArrayDestroy(this->bodyArray);
    dBodyDestroy(this->trackBody);
    for(int w = 0; w < 2; w++) {
        dBodyDestroy(this->wheelBody[w]);
        dGeomDestroy(this->wheelGeom[w]);
#if USE_GUIDE_GEOMS
        dGeomDestroy(this->guideGeom[w]);
#endif
    }
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        dBodyDestroy(this->linkBody[i]);
        dGeomDestroy(this->linkGeom[i]);
        dGeomDestroy(this->grouserGeom[i]);
        dJointDestroy(this->linkJoint[i]);
#if !USE_GUIDE_GEOMS
        dJointDestroy(this->guideJoints[i]);
#endif
    }
}

void TrackBase::draw() {
    dsSetColor(1, 1, 0);
    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->wheelGeom[w]);
        const dReal *R = dGeomGetRotation(this->wheelGeom[w]);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheelGeom[w], &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }

    dsSetColor(1, 0, 0);
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->linkGeom[i]);
        const dReal *R = dGeomGetRotation(this->linkGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->linkGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->grouserGeom[i]);
        const dReal *R = dGeomGetRotation(this->grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}
