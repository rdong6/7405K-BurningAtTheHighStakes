#include "lib/motion/PIDTurn.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"

namespace {
	int sign(double val) {
		return (val > 0) - (val < 0);
	}
}// namespace

PIDTurn::PIDTurn(double targetHeading, PID pid, bool brakeLeft, bool brakeRight, double threshold, double maxPower,
                 bool forceRight, bool forceLeft)
    : targetHeading(targetHeading), threshold(threshold), maxPower(maxPower), pid(pid), brakeLeft(brakeLeft),
      brakeRight(brakeRight), forceRight(forceRight), forceLeft(forceLeft) {}

void PIDTurn::start() {
	if (startTime == 0) [[unlikely]] {
		printf("Turn Started\n");
		auto odom = robotInstance->getSubsystem<Odometry>();

		initialSign = odom ? ::sign(util::getShortestAngle(util::toDeg(odom.value()->getCurrentState().position.theta()),
		                                                   targetHeading))
		                   : 1;

		auto drive = robotInstance->getSubsystem<Drive>();
		prevBrakeMode = drive ? drive.value()->getBrakeMode() : pros::E_MOTOR_BRAKE_BRAKE;
		IMotion::start();
	}
}

IMotion::MotorVoltages PIDTurn::calculate(const kinState& state) {
	// Calculates the difference in degrees from our current heading to the target heading
	double error = util::getShortestAngle(util::toDeg(state.position.theta()), targetHeading);

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


	// this part is for the hacky spline turn thing
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
	double leftPwr = brakeLeft ? 0 : -turnPwr;
	double rightPwr = brakeRight ? 0 : turnPwr;


	printf("Error: %.2f Left Pwr: %.2f  Right Pwr: %.2f\n", error, leftPwr, rightPwr);
	// logger->debug("Error: {:.2f} Counter: {} heading: {} Left Pwr: {} Right Pwr: {}\n", error, counter,
	//               util::toDeg(state.position.theta()), leftPwr, rightPwr);

	return {leftPwr, rightPwr};
}

bool PIDTurn::isSettled(const kinState& state) {
	// If the counter is above 5 (meaning we have had our error below the threshold for 5 iterations) then return that
	// it is settled
	return counter > 5;
}