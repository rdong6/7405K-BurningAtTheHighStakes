#include "lib/motion/NullMotion.h"
#include "lib/geometry/kinState.h"

// No motion - does nothing

NullMotion::NullMotion(bool velControl) : velocityControlled(velControl) {}

IMotion::MotorVoltages NullMotion::calculate(const kinState& state) {
	return {0.0, 0.0};
}

bool NullMotion::isVelocityControlled() const {
	return velocityControlled;
}

bool NullMotion::isSettled(const kinState& state) {
	return false;
}