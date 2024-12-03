#pragma once
#include "Constants.h"
#include "Logger.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

class Intake : public Subsystem {
private:
	enum class AntiJamState { IDLE, UNWIND };

	pros::MotorGroup motors{ports::intake};
	pros::Distance distance{ports::intakeDistance};
	pros::Optical color{ports::intakeColor};

	AntiJamState state = AntiJamState::IDLE;
	bool codeOverride = false;// code takes control of intake

	bool redRingDetector();
	bool blueRingDetector();

	bool (Intake::*blueismDetector)(void) = nullptr;

	RobotThread runner();

public:
	struct flags {
		bool antiJam{false}; // enable or disable antijam
		bool isMoving{false};// is intake commanded to move?
		bool torqueStop{false};
		bool distStop{false};

		// ring status
		bool partiallyIn{false};
		bool fullyIn{false};
		bool storeSecond{false};
	};

	explicit Intake(RobotBase* robot);

	void registerTasks() override;

	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();

	void setTorqueStop(bool val);
	void setDistStop(bool val);
};