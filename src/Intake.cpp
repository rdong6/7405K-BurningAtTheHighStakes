#include "Intake.h"
#include "Constants.h"
#include "Controller.h"
#include "Logger.h"
#include "pros/motors.h"
#include <cmath>

Intake::Intake() : motors(ports::intake) {
	// sets to coast to avoid motor burnout
	motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// sets the proper cart of the motor incase you ever want to use moveVel
	motors.set_gearing_all(pros::E_MOTOR_GEAR_200);

	logger = sLogger.createSource("Intake");
	logger->toggleLevel(LogSource::INFO);// disables it for now
}

void Intake::initialize() {
	// Maps holding r1 to intake
	sController.registerCallback([]() { sIntake.moveVoltage(12000); }, []() {}, Controller::master, Controller::r2,
	                             Controller::hold);

	// Maps holding r2 to outtake - when both r1 and r2 are held, r2 takes precedent
	sController.registerCallback([]() { sIntake.moveVoltage(-12000); }, []() {}, Controller::master, Controller::r1,
	                             Controller::hold);

	// stops the intake when neither button is pressed
	sController.registerCallback([]() { sIntake.moveVoltage(0); }, []() {}, Controller::master, Controller::r1,
	                             Controller::falling);

	sController.registerCallback([]() { sIntake.moveVoltage(0); }, []() {}, Controller::master, Controller::r2,
	                             Controller::falling);
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