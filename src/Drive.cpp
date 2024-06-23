#include "Drive.h"
#include "Controller.h"
#include "Robot.h"
#include "lib/physics/NullMotion.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"
#include "lib/utils/todo.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>

void Drive::initialize() {
	leftDrive.set_gearing_all(pros::E_MOTOR_GEAR_600);
	rightDrive.set_gearing_all(pros::E_MOTOR_GEAR_600);

	leftDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	rightDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

	resetPosition();
	pros::delay(40);// delay to give time for sensor data to reset

	logger = sLogger.createSource("Drive");

	task = pros::c::task_create([](void* ign) { sDrive.runner(); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Drive");
}

void Drive::runner() {
	// variable is used when delaying task to maintain a consistent 10ms update loop
	uint32_t time = pros::millis();

	while (true) {
		// force local scope so that our guard gets destroyed before we delay
		{
			auto curMotionGuard = currentMotion.wlock();
			Motion& curMotion = *curMotionGuard;

			// marks the time in which we first start to run the motion
			curMotion->start();

			// check if motion has timed out (if it has, set current motion to nullmotion)
			if (isTimedOut.load()) {
				logger->warn("MOTION TIMED OUT\n");

				// !-------------------!
				//      VERIFY THIS
				// !-------------------!
				// This ref may be invalid?
				curMotion = NullMotion();
				isTimedOut.store(false);
			}

			// Gets new motor voltages based on calculations related to the specific motion
			auto motorVolts = curMotion->calculateVoltages(sOdom.getCurrentState());

			if (curMotion->isVelocityControlled()) {
				setVelocityLeft(motorVolts.left);
				setVelocityRight(motorVolts.right);
			} else {
				setVoltageLeft(motorVolts.left);
				setVoltageRight(motorVolts.right);
			}

			// Stores whether the motion has finished
			isSettled.store(curMotion->isSettled(sOdom.getCurrentState()));
		}

		pros::c::task_delay_until(&time, 10);
	}
}

// Waits until the bots motion has finished travelling where it wanted to
bool Drive::waitUntilSettled(uint32_t timeout) {
	auto timer = Timeout(timeout);

	// While the waiting has not timed out
	while (!timer.timedOut()) {
		// If it has settled
		if (isSettled.load()) {
			// Reset variable
			isSettled.store(false);

			// Set motion to null motion - if we have reached our setpoint
			// !-------------------!
			//      VERIFY THIS
			// !-------------------!
			// This ref may be invalid?
			currentMotion.wlock([](auto& data) { data = NullMotion(); });

			return true;
		}

		pros::delay(10);
	}

	// Timed out
	isTimedOut.store(true);

	// wait until the Drive thread realizes we timed out
	while (isTimedOut.load()) { pros::delay(10); }

	return false;
}

void Drive::waitUntilDist(double dist) {
	Pose startingPos = sOdom.getCurrentState().position;
	double distTraveled = startingPos.distanceTo(sOdom.getCurrentState().position);
	// While the waiting has not timed out
	while (distTraveled < dist) {
		// If it has settled
		pros::delay(10);
		distTraveled = startingPos.distanceTo(sOdom.getCurrentState().position);
	}
}

void Drive::setCurrentMotion(Motion motion) {
	// Tells the drive to do whatever motion is passed in

	// !-------------------!
	//      VERIFY THIS
	// !-------------------!
	// This ref may be invalid?
	currentMotion.wlock([motion](auto& data) { data = std::move(motion); });
}

void Drive::setVoltageLeft(int voltage) {
	leftDrive.move_voltage(voltage);
}

void Drive::setVoltageRight(int voltage) {
	rightDrive.move_voltage(voltage);
}

void Drive::setVelocityLeft(int velocity) {
	leftDrive.move_velocity(velocity);
}

void Drive::setVelocityRight(int velocity) {
	rightDrive.move_velocity(velocity);
}

double Drive::getLeftPosition() const {
	// return (middleLeft.get_position() + frontLeft.get_position() + backLeft.get_position()) / 3.0;
	return leftDrive.get_position();// not as good as before where we could easily specify a certain motor (we liked
	                                // middle motors)
}

double Drive::getRightPosition() const {
	// return (middleRight.get_position() + frontRight.get_position() + backRight. get_position()) / 3.0;
	return rightDrive.get_position();// not as good as before where we could easily specify a certain motor (we liked
	                                 // middle motors)
}

void Drive::resetPosition() {
	leftDrive.set_zero_position_all(0);
	rightDrive.set_zero_position_all(0);

	leftDrive.tare_position_all();
	rightDrive.tare_position_all();
}

void Drive::setBrakeMode(pros::motor_brake_mode_e_t mode) {
	leftDrive.set_brake_mode_all(mode);
	rightDrive.set_brake_mode_all(mode);
}

void Drive::setBrakeModeLeft(pros::motor_brake_mode_e_t mode) {
	leftDrive.set_brake_mode_all(mode);
}

void Drive::setBrakeModeRight(pros::motor_brake_mode_e_t mode) {
	rightDrive.set_brake_mode_all(mode);
}

pros::motor_brake_mode_e_t Drive::getBrakeMode() const {
	return static_cast<pros::motor_brake_mode_e_t>(leftDrive.get_brake_mode());
}