#pragma once
#include "Motion.h"
#include "lib/controller/PID.h"
#include "pros/motors.h"

// this entire motion just uses everything in degrees cause its easier
class PIDTurn : public IMotion {
private:
	double targetHeading;
	double threshold;
	double maxPower;
	PID pid;
	int counter = 0;// counter for how long we've been at a pos
	int initialSign = 0;
	pros::motor_brake_mode_e prevBrakeMode = pros::E_MOTOR_BRAKE_INVALID;

	bool brakeLeft;
	bool brakeRight;
	bool forceLeft;
	bool forceLeftTerminate = false;
	bool forceRight;
	bool forceRightTerminate = false;


public:
	PIDTurn(double targetHeading, PID pid, bool brakeLeft = false, bool brakeRight = false, double threshold = 0.5,
	        double maxPower = 12000, bool forceLeft = false, bool forceRight = false);
	void start() override;
	MotorVoltages calculate(const kinState& state) override;
	bool isSettled(const kinState& state) override;
};
