#include "lib/controller/PID.h"
#include "lib/utils/Math.h"
#include <cmath>
#include <cstdio>


PID::PID() noexcept : PID(0, 0, 0, false, 0) {}

PID::PID(double P, double I, double D, bool errorZeroFlip, double iBound) noexcept
    : kP(P), kI(I), kD(D), iBound(iBound), prevError(0), errorSum(0), errorZeroFlip(errorZeroFlip), resetFlag(true) {}

double PID::operator()(double error) {
	if (resetFlag) {
		prevError = error;
		errorSum = 0;
		resetFlag = false;
	}


	// integral windup thingamajig - 2 hacks are here
	// first one is a crappy iBound where any error below a certain threshold gets accumulated
	// and the second one is whenever we flip signs/overshoot/undershoot our target, we reset errorSum
	if (std::fabs(error) < iBound) {
		errorSum += error;
	} else {
		errorSum = 0;
	}
	if (util::sign(error) != util::sign(prevError) && errorZeroFlip) { errorSum = 0; }

	// Proportional = error * kP
	// Integral = error over time * kI
	// Derivative = change * kD

	double res = error * kP + errorSum * kI + (error - prevError) * kD;
	prevError = error;

	return res;
}

void PID::setP(double kP) {
	this->kP = kP;
}

void PID::setI(double kI) {
	this->kI = kI;
}

void PID::setD(double kD) {
	this->kD = kD;
}

void PID::reset() {
	resetFlag = true;
}