#include "lib/physics/DriveCharacterizationMotion.h"

DriveCharacterizationMotion::DriveCharacterizationMotion() {
	//
}

void DriveCharacterizationMotion::start() {
	file = fopen("/usd/driveCharacterization.txt", "w");
	fprintf(file, "Time,Left Vol,Left Vel,Right Pos,Right Vol,Right Vel,Right Pos");
}

MotorVoltages DriveCharacterizationMotion::calculate(const kinState state) {
	//// Quasi-static
	double time = (pros::millis() - startTime) / 1000.0;// time since start in seconds
	double pwr = time * 300;                            // V/s

	//

	return {pwr, pwr};

	//// Max Accel Characterization:
	// return {12000, 12000};
}

bool DriveCharacterizationMotion::isSettled(const kinState state) {
	return false;
}