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

	pros::adi::DigitalOut extender{'F'};
	pros::MotorGroup motors{ports::intake};
	pros::Distance distance{ports::intakeDistance};
	pros::Optical color{ports::intakeColor};

	AntiJamState state = AntiJamState::IDLE;
	bool codeOverride = false;// code takes control of intake
	uint32_t timedMoveStartTime = 0;

	int lastCommandedVoltage = 0;

	bool extenderEnabled = false;

	bool (Intake::*blueismDetector)(void) = nullptr;

	bool redRingDetector();
	bool blueRingDetector();

	RobotThread runner();
	RobotThread blueismCoro();
	RobotThread ladyBrownClearanceCoro();// moves intake slightly back so lady brown clears intake as it goes to score
	RobotThread antiJamCoro();

public:
	struct flags {
		bool antiJam{false}; // enable or disable antijam
		bool isMoving{false};// is intake commanded to move?
		bool torqueStop{false};
		bool distStop{false};
		bool ladyBrownClearanceEnabled{false};// when enabled by lift, move intake back slightly

		bool colorSortResumes{false};// after color sort, we resume last commanded voltage

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

	void setExtender(bool val);
	void toggleExtender();
};