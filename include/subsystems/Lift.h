#pragma once
#include "Constants.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/controller/PID.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

class Lift : public Subsystem {
private:
	pros::Motor motor{ports::liftMotor};
	pros::Rotation rotation{ports::liftRotation};
	pros::adi::DigitalOut claw{'B'};
	// PID pid = PID(220, 10, 0, true, 10);
	PID pid = PID(400, 10, 0, true, 10);

	RobotThread updateAngle();
	RobotThread runner();

	RobotThread openLiftCoro();
	RobotThread closeLiftCoro();

	void move(int mv);
	void hold();

public:
	struct flags {
		double curAngle{};
		double targetAngle{20};
		double errorThresh{2};

		bool isMoving{false};       // is the lift arm open
		bool isOpen{false};         // is the lift open
		bool isMotionRunning{false};// is the lift running a motion (open/close lift coroutine)

		bool kill{false};// moment this is set to true kills entire subsystem
	};

	explicit Lift(RobotBase* robot);

	void registerTasks() override;

	void toggleState();
	void setState(bool open);
};