#pragma once
#include "Logger.h"
#include "Motion.h"
#include "lib/controllers/PID.h"
#include "pros/motors.h"

// this entire motion just uses everything in degrees cause its easier
class PIDTurn : public IMotion {
private:
	double targetHeading;
	double threshold;
	double degree;
	PID pid;
	int counter;// counter for how long we've been at a pos
	int initialSign;
	pros::motor_brake_mode_e prevBrakeMode;

	bool brakeLeft;
	bool brakeRight;
	bool forceRight;
	bool forceRightTerminate;
	bool forceLeft;
	bool forceLeftTerminate;

	static LoggerPtr logger;


public:
	PIDTurn(double targetHeading, PID pid, bool brakeLeft = false, bool brakeRight = false, double threshold = 0.5, bool forceRight = false, bool forceLeft = false);
	void start() override;
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};
