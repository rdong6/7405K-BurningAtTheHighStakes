#pragma once
#include "Constants.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/controller/PID.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

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
	pros::adi::DigitalOut claw{'B'};
	// PID pid = PID(220, 10, 0, true, 10);


	RobotThread updateAngle();
	RobotThread runner();

	RobotThread openLiftCoro();
	RobotThread closeLiftCoro();

	void move(int mv);

public:
	struct flags {
		State state{State::IDLE};

		PID pid = PID(400, 10, 0, true, 10);

		double curAngle{};
		double targetAngle{5};// TBD - Angle lift should move to if code's controlling
		double errorThresh{2};

		bool kill{false};// set to true to kill entire lift subsystem

		// IGNORE THESE -> will be deleted
		bool isMoving{false};       // is the lift arm open
		bool isOpen{false};         // is the lift open
		bool isMotionRunning{false};// is the lift running a motion (open/close lift coroutine)
		bool isHolding{false};
	};

	explicit Lift(RobotBase* robot);

	void registerTasks() override;

	void setState(State state);

	void setClaw(double enabled);
};