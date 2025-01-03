#include "lib/motion/Motion.h"
#include "main.h"

// Parent class for all motions

void IMotion::start() {
	if (startTime == 0) [[unlikely]] { startTime = pros::millis(); }
};

bool IMotion::isVelocityControlled() const {
	return false;
}