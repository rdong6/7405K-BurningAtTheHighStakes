#pragma once
#include "Logger.h"
#include "main.h"
#include "pros/motors.hpp"

#define sIntake Intake::getInstance()

class Intake {
private:
	pros::MotorGroup motors;
	LoggerPtr logger;

	Intake();
	Intake(const Intake&) = delete;
	Intake& operator=(const Intake&) = delete;

public:
	inline static Intake& getInstance() {
		static Intake INSTANCE;

		return INSTANCE;
	}

	void initialize();

	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();
};