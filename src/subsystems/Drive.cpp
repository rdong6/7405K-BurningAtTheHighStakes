#include "subsystems/Drive.h"
#include "RobotBase.h"
#include "lib/motion/NullMotion.h"
#include "lib/motion/OpControlMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"
#include "lib/utils/todo.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "subsystems/Controller.h"
#include "subsystems/Intake.h"
#include "subsystems/Odometry.h"
#include <algorithm>
#include <stdexcept>

Drive::Drive(RobotBase* robot) : Subsystem(robot) {
	leftDrive.set_gearing_all(pros::E_MOTOR_GEAR_600);
	rightDrive.set_gearing_all(pros::E_MOTOR_GEAR_600);
	weakLeft.set_gearing(pros::E_MOTOR_GEAR_200);
	weakRight.set_gearing(pros::E_MOTOR_GEAR_200);

	leftDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	rightDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	weakLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	weakRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	switch (leftDrive.get_gearing()) {
		case pros::MotorGears::rpm_100:
			gearRatioWeak2Normal = 2;
		case pros::MotorGears::rpm_600:
			gearRatioWeak2Normal = 1.0 / 3.0;
		default:
			gearRatioWeak2Normal = 1;
	}

	resetPosition();
}

void Drive::registerTasks() {
	// Auton thread
	robot->registerTask(
	        [this]() {
		        this->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
		        return this->runner();
	        },
	        TaskType::AUTON);

	// Opctrl thread
	robot->registerTask(
	        [this]() {
		        this->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
		        this->setCurrentMotion(OpControlMotion());
		        return this->runner();
	        },
	        TaskType::OPCTRL);
}

RobotThread Drive::runner() {
	auto odom = robot->getSubsystem<Odometry>();

	while (true) {
		// marks the time in which we first start to run the motion
		curMotion->start();

		kinState curState = odom ? odom.value()->getCurrentState() : kinState();

		// Gets new motor voltages based on calculations related to the specific motion
		auto motorVolts = curMotion->calculate(curState);

		if (curMotion->isVelocityControlled()) {
			// add a 0.5 so requested velocity rounds to nearest number when casting to int
			setVelocityLeft(motorVolts.left + 0.5);
			setVelocityRight(motorVolts.right + 0.5);
		} else {
			setVoltageLeft(motorVolts.left);
			setVoltageRight(motorVolts.right);
		}

		// Stores whether the motion has finished
		isSettled = curMotion->isSettled(curState);

		co_yield util::coroutine::nextCycle();
	}
}

// Waits until the bots motion has finished travelling where it wanted to
RobotFunc Drive::waitUntilSettled(uint32_t timeout) {
	auto timer = Timeout(timeout);

	auto func = [this, timer]() -> bool {
		if (timer.timedOut()) {
			// motion timed out
			this->curMotion = NullMotion();

			return true;
		} else {
			if (isSettled) {
				// motion finished, reset drive state
				this->isSettled = false;
				this->curMotion = NullMotion();

				return true;
			}

			return false;
		}
	};

	return func;
}

RobotFunc Drive::waitUntilDist(double dist) {
	auto odom = robot->getSubsystem<Odometry>();
	Pose startingPos = odom ? odom.value()->getCurrentState().position : Pose();

	auto func = [dist, startingPos, odom]() -> bool {
		return dist <= startingPos.translation().distanceTo(
		                       (odom ? odom.value()->getCurrentState().position.translation() : Translation2D()));
	};

	return func;
}

void Drive::setCurrentMotion(Motion motion) {
	// Tells the drive to do whatever motion is passed in
	curMotion = std::move(motion);
}

void Drive::setVoltageLeft(int voltage) {
	leftDrive.move_voltage(voltage);
	weakLeft.move_voltage(voltage);
}

void Drive::setVoltageRight(int voltage) {
	rightDrive.move_voltage(voltage);
	weakRight.move_voltage(voltage);
}

void Drive::setVelocityLeft(int velocity) {
	leftDrive.move_velocity(velocity);
	weakLeft.move_velocity(velocity * gearRatioWeak2Normal);
}

void Drive::setVelocityRight(int velocity) {
	rightDrive.move_velocity(velocity);
	weakRight.move_velocity(velocity * gearRatioWeak2Normal);
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
	weakLeft.set_brake_mode(mode);
	rightDrive.set_brake_mode_all(mode);
	weakRight.set_brake_mode(mode);
}

void Drive::setBrakeModeLeft(pros::motor_brake_mode_e_t mode) {
	leftDrive.set_brake_mode_all(mode);
	weakLeft.set_brake_mode(mode);
}

void Drive::setBrakeModeRight(pros::motor_brake_mode_e_t mode) {
	rightDrive.set_brake_mode_all(mode);
	weakRight.set_brake_mode(mode);
}

pros::motor_brake_mode_e_t Drive::getBrakeMode() const {
	return static_cast<pros::motor_brake_mode_e_t>(leftDrive.get_brake_mode());
}