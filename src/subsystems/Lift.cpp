#include "subsystems/Lift.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"
#include <limits>

#define UPPER_BOUNDS 185
#define LOWER_BOUNDS 1.5

Lift::Lift(RobotBase* robot) : Subsystem(robot) {
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor.set_gearing(pros::E_MOTOR_GEAR_100);

	rotation.set_data_rate(5);
	pros::delay(20);// needed so get_angle() isn't 0
	int32_t curPosition = rotation.get_angle();
	rotation.set_position(curPosition);
	// rotation.set_position(curPosition < 1000 ? 36000 + curPosition : curPosition);
	pros::delay(20);
}

void Lift::registerTasks() {
	robot->registerTask([this]() { return this->updateAngle(); }, TaskType::SENTINEL);
	robot->registerTask([this]() { return this->runner(); }, TaskType::AUTON);
	robot->registerTask([this]() { return this->runner(); }, TaskType::OPCTRL);

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }
	auto controllerRef = controller.value();

	// when lift controller inputs set, stop whatever code motion is happening
	controllerRef->registerCallback(
	        [this]() {
		        Lift::State& curState = robot->getFlag<Lift>().value()->state;
		        if (curState == State::LEVEL_1) {
			        // move intake slightly back so it doesn't get caught in ring
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = true;
		        } else {
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = false;
		        }

		        curState = State::IDLE;
		        move(12000);
	        },
	        []() {}, Controller::master, Controller::l1, Controller::hold);

	controllerRef->registerCallback(
	        [this]() {
		        robot->getFlag<Lift>().value()->state = State::IDLE;
		        move(-12000);
	        },
	        []() {}, Controller::master, Controller::l2, Controller::hold);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l1, Controller::falling);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l2, Controller::falling);

	controllerRef->registerCallback(
	        [this]() {
		        setState(State::LEVEL_1);
		        // toggleState();
	        },
	        []() {}, Controller::master, Controller::right, Controller::rising);
}

RobotThread Lift::updateAngle() {
	auto liftFlags = robot->getFlag<Lift>().value();
	int counter = 0;

	while (true) {
		int32_t pos = rotation.get_position();
		double rotationVel = rotation.get_velocity() / 100.0;// deg/s

		// 1:3
		double motorVel = motor.get_actual_velocity() / 60;
		if (liftFlags->isMoving && fabs(motorVel) > 0 && util::fpEquality(fabs(rotationVel), 0.0)) {
			counter++;
		} else {
			counter = 0;
		}

		// kill lift if rotation sensor becomes unplugged (data is max) or if rotation's data freezes
		if (pos == std::numeric_limits<int32_t>::max() || counter > 20) {
			liftFlags->kill = true;
			co_return;
		}

		liftFlags->curAngle = pos / 100.0;
		co_yield util::coroutine::nextCycle();
	}
}

RobotThread Lift::runner() {
	auto liftFlags = robot->getFlag<Lift>().value();
	liftFlags->isMoving = true;
	liftFlags->pid.reset();

	while (true) {
		if (liftFlags->kill) { co_return; }

		/*if (liftFlags->curAngle <= 300) { robot->getSubsystem<Intake>().value()->setExtender(true); }

		//  lower intake extender -> lift was last to touch it and move it up
		if (liftFlags->curAngle > 300) { robot->getSubsystem<Intake>().value()->setExtender(false); }

		// test this hard clamp
		if (liftFlags->curAngle <= 210) {
		    liftFlags->targetAngle = 220;
		    liftFlags->state = Lift::HOLD;
		}*/

		// SOFT STOP
		if (liftFlags->curAngle >= 185) {
			liftFlags->targetAngle = 185;
			setState(Lift::HOLD);
		}

		if (liftFlags->state == State::IDLE) {
			liftFlags->isMoving = false;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			motor.move_velocity(0);
			co_yield util::coroutine::nextCycle();
			continue;
		}

		double error = liftFlags->targetAngle - liftFlags->curAngle;
		if (fabs(error) >= liftFlags->errorThresh) {
			liftFlags->isMoving = true;
			double pwr = liftFlags->pid(error);
			move(pwr);
			printf("Running. Error: %f  CurAngle: %f\n", error, liftFlags->curAngle);
		} else {
			liftFlags->isMoving = false;

			if (liftFlags->state == State::STOW) {
				motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				liftFlags->state = State::IDLE;
			}

			liftFlags->pid.reset();
			move(0);
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Lift::toggleState() {

	switch (robot->getFlag<Lift>().value()->state) {
		case State::LEVEL_1:
			setState(State::STOW);
			break;
		case State::LEVEL_2:
			setState(State::LEVEL_1);
			break;
		case State::STOW:
			setState(State::LEVEL_1);
			break;
		case State::IDLE:
			setState(State::LEVEL_1);
			break;
		case State::HOLD:
			setState(State::LEVEL_1);
			break;
	}
}

void Lift::setState(State state) {
	robot->getFlag<Lift>().value()->state = state;

	switch (state) {
		case State::LEVEL_1:
			robot->getFlag<Lift>().value()->targetAngle = 36.3;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::LEVEL_2:
			robot->getFlag<Lift>().value()->targetAngle = 36.3;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::STOW:
			robot->getFlag<Lift>().value()->targetAngle = 2.5;// determine
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::IDLE:
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			break;
		case State::HOLD:
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
	}

	// robot->getFlag<Lift>().value()->isOpen = open;
	// robot->getFlag<Lift>().value()->isHolding = false;

	// if (open) {
	// 	robot->registerTask([this]() { return this->openLiftCoro(); }, TaskType::SENTINEL);
	// } else {
	// 	robot->registerTask([this]() { return this->closeLiftCoro(); }, TaskType::SENTINEL);
	// }
}

void Lift::setClaw(double enabled) {
	claw.set_value(enabled);
}


void Lift::move(int mv) {
	double curAngle = robot->getFlag<Lift>().value()->curAngle;
	if (mv > 0 && curAngle > UPPER_BOUNDS) { mv = 0; }
	if (mv < 0 && curAngle < LOWER_BOUNDS) { mv = 0; }

	//// for soft stop
	if (mv > 0) { mv *= (210 - curAngle) / 210.0; }

	if (mv == 0) {
		motor.brake();
	} else {
		motor.move_voltage(mv);
	}
}