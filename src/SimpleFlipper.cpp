#include "SimpleFlipper.h"
#include "Environment.h"
#include <drawstuff/drawstuff.h>

SimpleFlipper::SimpleFlipper(Environment *environment_, const std::string &name_, dReal radius1_, dReal radius2_, dReal distance_, dReal trackDepth_, unsigned long additionalCategory) : SimpleTrackBase(environment_, name_, radius1_, radius2_, distance_, trackDepth_, additionalCategory) {

}

SimpleFlipper::~SimpleFlipper() {

}

void SimpleFlipper::create() {
    SimpleTrackBase::create();
}

void SimpleFlipper::destroy() {
    SimpleTrackBase::destroy();
}

void SimpleFlipper::step(dReal stepSize) {
    this->velocity.step(stepSize);
}

void SimpleFlipper::draw() {
    SimpleTrackBase::draw();
}

void SimpleFlipper::setVelocity(dReal velocity) {
    this->velocity.set(velocity);
}

unsigned long SimpleFlipper::getGrouserCategory() {
    return Category::FLIPPER_GROUSER | Category::FLIPPER | this->additionalCategory;

}

dReal SimpleFlipper::getVelocity() {
    return this->velocity.get();
}
