#include "lib/motion/Motion.h"
#include "pros/rtos.h"

// Parent class for all motions

void IMotion::start() {
	if (startTime == 0) [[unlikely]] { startTime = pros::c::millis(); }
};

bool IMotion::isVelocityControlled() const {
	return false;
}