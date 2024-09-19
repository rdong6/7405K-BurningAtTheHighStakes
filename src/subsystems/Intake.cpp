#include "subsystems/Intake.h"
#include "Constants.h"
#include "Logger.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include <cmath>

Intake::Intake(RobotBase* robot) : Subsystem(robot, this) {
	// sets to coast to avoid motor burnout
	motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// sets the proper cart of the motor incase you ever want to use moveVel
	motors.set_gearing_all(pros::E_MOTOR_GEAR_200);

	extender.set_value(false);
}

void Intake::registerTasks() {
	robot->registerTask([this]() { return this->runner(); }, TaskType::SENTINEL);

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }

	auto controllerRef = controller.value();

	// Maps holding r1 to intake
	controllerRef->registerCallback([this]() { this->moveVoltage(12000); }, []() {}, Controller::master, Controller::r2,
	                                Controller::hold);

	// Maps holding r2 to outtake - when both r1 and r2 are held, r2 takes precedent
	controllerRef->registerCallback([this]() { this->moveVoltage(-12000); }, []() {}, Controller::master,
	                                Controller::r1, Controller::hold);

	// stops the intake when neither button is pressed
	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r1,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r2,
	                                Controller::falling);
}

// As of now, running constantly on robot no matter the competition state
RobotThread Intake::runner() {
	auto intakeFlags = robot->getFlag<flags>().value();

	while (true) {
		int32_t dist = distance.get();// in mm

		// printf("Torque: %f  Vel: %f\n", motors.get_torque(), motors.get_actual_velocity());
		if (intakeFlags->torqueStop &&
		    (motors.get_torque() > 0.97 && fabs(motors.get_actual_velocity()) >= 20) /*tune later*/) {
			printf("Stopping due to torque\n\n\n\n\n\n\n\n");
			motors.brake();
			this->setTorqueStop(false);
		}

		if (intakeFlags->distStop) {
			if (dist <= 55) {
				intakeFlags->partiallyIn = false;
				intakeFlags->fullyIn = true;
			} else if (dist <= 100) {
				if (intakeFlags->storeSecond) { intakeFlags->storeSecond = false; }
				intakeFlags->fullyIn = false;
				intakeFlags->partiallyIn = true;
			} else {
				if (intakeFlags->storeSecond) { intakeFlags->storeSecond = false; }
				intakeFlags->partiallyIn = false;
				intakeFlags->fullyIn = false;
			}

			if (!intakeFlags->storeSecond && intakeFlags->fullyIn) {
				motors.brake();
				this->setDistStop(false);
			}
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Intake::setExtender(bool extended) {
	auto intakeFlags = robot->getFlag<flags>().value();
	intakeFlags->isExtended = extended;
	extender.set_value(extended);
}

void Intake::setTorqueStop(bool val) {
	auto intakeFlags = robot->getFlag<flags>().value();
	intakeFlags->torqueStop = val;
}

void Intake::setDistStop(bool val) {
	auto intakeFlags = robot->getFlag<flags>().value();
	intakeFlags->distStop = val;
	int32_t dist = distance.get();
	if (dist < 55) { intakeFlags->storeSecond = true; }
}

void Intake::toggleExtender() {
	setExtender(!robot->getFlag<flags>().value()->isExtended);
}

void Intake::moveVoltage(int mv) {
	motors.move_voltage(mv);
}

void Intake::moveVel(int vel) {
	motors.move_velocity(vel);
}

void Intake::brake() {
	motors.brake();
}