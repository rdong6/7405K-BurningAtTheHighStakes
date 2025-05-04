#pragma once
#include "Constants.h"
#include "Logger.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/filters/MA.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include <queue>
#include <sigslot/signal.h>

class Intake : public Subsystem {
private:
	enum class AntiJamState { IDLE, UNWIND };

	pros::MotorGroup motors{ports::intake};
	pros::Distance distance{ports::intakeDistance};
	pros::Distance blueismDistance{ports::blueismDistance};
	pros::Optical color{ports::intakeColor};
	pros::adi::DigitalOut extender{'E'};

	bool codeOverride = false;// intake code takes control of intake -> no other subsystem/controller can override

	AntiJamState state = AntiJamState::IDLE;

	bool isExtended = false;

	bool intakeStalled = false;

	// for blueism coro -> when we eject a ring, if enabled, we will resume last intake voltage after ejecting ring
	int lastCommandedVoltage = 0;
	MA<int> ringColorMA{3};
	std::queue<Alliance> ringsSeen{};// list of rings our intake has seen

	RobotThread runner();
	RobotThread ringDetectorCoro();// coro that detects colored rings passing through the intake
	RobotThread blueismCoro();
	RobotThread ladyBrownClearanceCoro();// moves intake slightly back so lady brown clears intake as it goes to score
	RobotThread
	ladyBrownLoadedCoro();// if intake is intaking ring into lady brown and it stalls, kill the motor -> used in autons
	RobotThread antiJamCoro();
	RobotThread stalledDetectorCoro();

public:
	struct flags {
		bool antiJam{false}; // enable or disable antijam
		bool isMoving{false};// is intake commanded to move?
		bool torqueStop{false};
		bool distStop{false};
		bool ladyBrownClearanceEnabled{false};// when enabled by lift, move intake back slightly

		bool colorSortResumes{true};// after color sort, we resume last commanded voltage

		// ring status
		bool partiallyIn{false};
		bool fullyIn{false};
		bool storeSecond{false};
	};

	sigslot::signal_st<> ladyBrownLoadedSignal;

	explicit Intake(RobotBase* robot);

	void registerTasks() override;

	// Use these funcs! Only use pros motor api calls if code is overriding other inputs
	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();

	void setTorqueStop(bool val);
	void setDistStop(bool val);

	void setExtender(bool extended);
	void toggleExtender();

	bool isStalled() const;

	int ringsInIntake() const;// returns # of rings in our intake -> only works in auton due to ringDetectorCoro

	bool ringAtDistSensor();// helper function for just 1 auton. returns whether dist sensor detects a ring
};