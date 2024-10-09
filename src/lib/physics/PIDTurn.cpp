#include "lib/physics/PIDTurn.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "subsystems/Drive.h"

namespace {
	int sign(double val) {
		return (val > 0) - (val < 0);
	}
}// namespace

PIDTurn::PIDTurn(double targetHeading, PID pid, bool brakeLeft, bool brakeRight, double threshold, double maxPower,
                 bool forceRight, bool forceLeft)
    : targetHeading(targetHeading), pid(pid), counter(0), threshold(threshold), brakeLeft(brakeLeft),
      brakeRight(brakeRight), initialSign(0), forceRight(forceRight), forceRightTerminate(false), forceLeft(forceLeft),
      forceLeftTerminate(false), maxPower(maxPower) {}

void PIDTurn::start() {
	if (startTime == 0) [[unlikely]] {
		auto odom = robotInstance->getSubsystem<Odometry>();

		initialSign = odom ? ::sign(util::getShortestAngle(
		                             util::toDeg(odom.value()->getCurrentState().position.getTheta()), targetHeading))
		                   : 1;

		auto drive = robotInstance->getSubsystem<Drive>();
		prevBrakeMode = drive ? drive.value()->getBrakeMode() : pros::E_MOTOR_BRAKE_BRAKE;
		IMotion::start();
	}
}

IMotion::MotorVoltages PIDTurn::calculate(const kinState state) {
	// Calculates the difference in degrees from our current heading to the target heading
	double error = util::getShortestAngle(util::toDeg(state.position.getTheta()), targetHeading);

	if (forceRight && initialSign == -1 && !forceRightTerminate) {
		if (::sign(error) != initialSign) {
			forceRightTerminate = true;
		} else {
			error += 360;
		}
	}
	if (forceLeft && initialSign == 1 && !forceLeftTerminate) {
		if (::sign(error) != initialSign) {
			forceLeftTerminate = true;
		} else {
			error += -360;
		}
	}


	// this part is for the hacky spline thing
	auto drive = robotInstance->getSubsystem<Drive>();
	if (drive) {
		if (brakeLeft) { drive.value()->setBrakeModeLeft(pros::E_MOTOR_BRAKE_HOLD); }
		if (brakeRight) { drive.value()->setBrakeModeRight(pros::E_MOTOR_BRAKE_HOLD); }
	}

	// Every time the error is within the threshold, we add to a counter (see isSettled)
	// This is so we actually settle and have reached our desired heading
	// this mitigates situations where we may be traveling too quickly, reach our target, break out of the loop, but
	// then overshoot because we are traveling too quickly
	if (std::abs(error) < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	// compute motor powers given our heading error
	double turnPwr = pid(error);

	if (fabs(turnPwr) >= maxPower) { turnPwr = util::sign(turnPwr) * maxPower; }

	// the hacky spline thing again, otherwise calculate motor power for each side
	double leftPwr = brakeLeft ? 0 : turnPwr;
	double rightPwr = brakeRight ? 0 : -turnPwr;

	//  artifact: use if you want a lower bounds on voltage - this was here from worlds, where we had a min voltage
	//  of 1.4V given to our motors

	// if ((!brakeLeft)) {
	// 	leftPwr = util::sign(leftPwr) * util::lerp(1400, 12000.0, util::clamp(0.0, 1.0, fabs(leftPwr / 12000.0)));
	// }
	// if ((!brakeRight)) {
	// 	rightPwr = util::sign(rightPwr) * util::lerp(1400, 12000.0, util::clamp(0.0, 1.0, fabs(rightPwr / 12000.0)));
	// }

	// logger->debug("Error: {:.2f} Counter: {} heading: {} Left Pwr: {} Right Pwr: {}\n", error, counter,
	//               util::toDeg(state.position.getTheta()), leftPwr, rightPwr);

	return {leftPwr, rightPwr};
}

bool PIDTurn::isSettled(const kinState state) {
	// If the counter is above 5 (meaning we have had our error below the threshold for 5 iterations) then return that
	// it is settled
	return counter > 5;
}