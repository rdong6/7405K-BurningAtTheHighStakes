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
	pros::adi::DigitalOut extender{'A'};

	AntiJamState state = AntiJamState::IDLE;
	bool codeOverride = false;

	RobotThread runner();

public:
	struct flags {
		bool antiJam{true};
		bool isExtended{false};
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

	void setExtender(bool extended);
	void toggleExtender();

	void setTorqueStop(bool val);
	void setDistStop(bool val);
};