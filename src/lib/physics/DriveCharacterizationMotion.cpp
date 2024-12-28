#include "lib/physics/DriveCharacterizationMotion.h"
#include "Constants.h"
#include "pros/motors.h"
#include "pros/rotation.h"
#include <numbers>

#define MV_PER_S (1000)

void DriveCharacterizationMotion::start() {
	file = fopen("/usd/driveCharacterization.txt", "w");
	// fprintf(file, "Time,Left Vol,Left Vel,Left Pos,Right Vol,Right Vel,Right Pos");
	// printf("Time,Left Vol,Left Vel,Left Pos,Right Vol,Right Vel,Right Pos");

	startLeftPos = pros::c::rotation_get_position(ports::leftRotation);
	startRightPos = pros::c::rotation_get_position(ports::rightRotation);
}

IMotion::MotorVoltages DriveCharacterizationMotion::calculate(const kinState state) {
	//// Quasi-static
	double time = (pros::millis() - startTime) / 1000.0;// time since start in seconds
	double pwr = time * MV_PER_S;

	double leftVolMV =
	        (pros::c::motor_get_voltage(ports::frontLeftMotor) + pros::c::motor_get_voltage(ports::middleLeftMotor)) / 2.0;

	double rightVolMV =
	        (pros::c::motor_get_voltage(ports::frontRightMotor) + pros::c::motor_get_voltage(ports::middleRightMotor)) / 2.0;

	double leftVel = pros::c::rotation_get_velocity(ports::leftRotation) / 100.0 * odometers::leftDeadwheelDiameter *
	                 std::numbers::pi;// deg/s
	double rightVel =
	        pros::c::rotation_get_velocity(ports::rightRotation) / 100.0 * odometers::rightDeadwheelDiameter * std::numbers::pi;

	double leftPos = (pros::c::rotation_get_position(ports::leftRotation) - startLeftPos) / 100.0;
	double rightPos = (pros::c::rotation_get_position(ports::rightRotation) - startRightPos) / 100.0;

	printf("%f,%f,%f,%f,%f,%f,%f\n", time, leftVolMV, leftVel, leftPos, rightVolMV, rightVel, rightPos);
	// fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", time, leftVolMV, leftVel, leftPos, rightVolMV, rightVel, rightPos);

	return {pwr, pwr};

	//// Max Accel Characterization:
	// return {12000, 12000};
}

bool DriveCharacterizationMotion::isSettled(const kinState state) {
	// return (pros::millis() - startTime) / 1000.0  (12000.0 / MV_PER_S);

	return false;// for max accel
}