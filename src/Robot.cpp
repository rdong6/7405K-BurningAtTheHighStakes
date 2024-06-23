
#include "Robot.h"
#include "Constants.h"
#include "Controller.h"
#include "Drive.h"
#include "Intake.h"
#include "Logger.h"
#include "main.h"
#include "pros/apix.h"

void Robot::initialize() {
	sLogger.initialize("test.txt");
	sOdom.initialize();
	sController.initialize();
	sDrive.initialize();
	sIntake.initialize();
}

void Robot::setOpMode(Robot::OpMode op) {
	opmode.store(op);
	// switch (op) {
	// 	case DRIVER:
	// 		sDrive.setBrakeMode(MOTOR_BRAKE_COAST);
	// 		break;
	// 	case AUTONOMOUS:
	// 		sDrive.setBrakeMode(MOTOR_BRAKE_HOLD);
	// 		break;
	// }
}

Robot::OpMode Robot::getOpMode() {
	return opmode.load();
}

Auton Robot::getAuton() {
	return auton.load();
}

void Robot::setAuton(Auton auton) {
	this->auton.store(auton);
}