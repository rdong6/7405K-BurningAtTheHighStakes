#include "lib/motion/TimedMotion.h"
#include "lib/utils/Math.h"
#include "pros/rtos.hpp"

// Motion at a certain power for a desired amount of time

// Time is in ms
// Power is in mV
TimedMotion::TimedMotion(uint32_t time, double power) : power(power), delay(time) {}

IMotion::MotorVoltages TimedMotion::calculate(const kinState& state) {
	return {power, power};
}

// testing for now
bool TimedMotion::isVelocityControlled() const {
	return false;
}

bool TimedMotion::isSettled(const kinState& state) {
	return pros::millis() >= startTime + delay;
}