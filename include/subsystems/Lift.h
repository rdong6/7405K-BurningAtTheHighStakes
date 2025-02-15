#pragma once
#include "Constants.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/controller/PID.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "sigslot/signal.h"

class Lift : public Subsystem {
public:
	// IDLE = resting/driver control
	// LEVEL_1 = lower part of lady brown
	// LEVEL_2 = upper part of lady brown
	// STOW = move lift down into bot
	// HOLD = hold lift at target angle
	enum State { IDLE, LEVEL_1, LEVEL_2, STOW, HOLD };

private:
	pros::Motor motor{ports::liftMotor};
	pros::Rotation rotation{ports::liftRotation};

	RobotThread skillsWallstakeAutomationStage1();
	RobotThread updateAngle();
	RobotThread runner();

	void move(int mv);

public:
	struct flags {
		State state{State::IDLE};

		PID pid = PID(225, 10, 0, true, 10);

		double curAngle{};
		double targetAngle{5};// TBD - Angle lift should move to if code's controlling
		double errorThresh{2};

		bool liftActivatedExtender{false};// if true, lift will lower extender when lift arm is in size again -> disabled when
		                                  // any other part of code touches intake extender

		bool isMoving{false};// is the lift currently moving by code to a place

		bool kill{false};// set to true to kill entire lift subsystem
	};

	// idk any good way to expose adding connections w/o doing their entire api again so this is just gonna be public 🤷‍
	sigslot::signal_st<> liftDoneMovingSig;
	sigslot::signal_st<> liftIsIdleSig;

	explicit Lift(RobotBase* robot);

	void registerTasks() override;

	void toggleState();
	void setState(State state);
};