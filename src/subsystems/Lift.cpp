#include "subsystems/Lift.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"
#include <limits>

#define UPPER_BOUNDS 181
#define LOWER_BOUNDS 10

Lift::Lift(RobotBase* robot) : Subsystem(robot, this) {
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor.set_gearing(pros::E_MOTOR_GEAR_100);

	rotation.set_data_rate(5);
	pros::delay(20);// needed so get_angle() isn't 0
	int32_t curPosition = rotation.get_angle();
	rotation.set_position(curPosition > 34500 ? curPosition - 36000 : curPosition);
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
		        robot->getFlag<flags>().value()->isMotionRunning = false;
		        move(12000);
	        },
	        []() {}, Controller::master, Controller::l1, Controller::hold);

	controllerRef->registerCallback(
	        [this]() {
		        robot->getFlag<flags>().value()->isMotionRunning = false;
		        move(-12000);
	        },
	        []() {}, Controller::master, Controller::l2, Controller::hold);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l1,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l2,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { toggleState(); }, []() {}, Controller::master, Controller::right,
	                                Controller::rising);
}

RobotThread Lift::updateAngle() {
	auto liftFlags = robot->getFlag<flags>().value();
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
	auto liftFlags = robot->getFlag<flags>().value();
	liftFlags->isMoving = true;
	pid.reset();

	while (true) {
		if (liftFlags->kill) { co_return; }

		double error = liftFlags->targetAngle - liftFlags->curAngle;

		if (fabs(error) >= liftFlags->errorThresh && liftFlags->isMotionRunning) {
			liftFlags->isMoving = true;
			double pwr = pid(error);
			move(pwr);
		} else {
			liftFlags->isMoving = false;
			pid.reset();
			motor.brake();
		}

		co_yield util::coroutine::nextCycle();
	}
}

RobotThread Lift::openLiftCoro() {
	auto liftFlags = robot->getFlag<flags>().value();
	liftFlags->isMotionRunning = true;
	liftFlags->targetAngle = 110;// tune this
	liftFlags->errorThresh = 3;


	co_yield util::coroutine::nextCycle();

	// wait to move to place
	// maybe change this condition to make it faster/more fluid
	while (liftFlags->curAngle < 98 /* TBD! */ && liftFlags->isMoving) {
		if (!liftFlags->isMotionRunning) { co_return; }
		co_yield util::coroutine::nextCycle();
	}
	claw.set_value(true);
	co_yield util::coroutine::delay(100);

	pid.reset();
	liftFlags->targetAngle = 76.5;
	liftFlags->errorThresh = 1;

	co_yield util::coroutine::nextCycle();
	if (!liftFlags->isMotionRunning) { co_return; }
	co_yield [this]() -> bool { return !robot->getFlag<flags>().value()->isMoving; };
	liftFlags->isMotionRunning = false;
}

RobotThread Lift::closeLiftCoro() {
	auto liftFlags = robot->getFlag<flags>().value();

	claw.set_value(false);
	liftFlags->isMotionRunning = true;
	liftFlags->targetAngle = 105;
	liftFlags->errorThresh = 3;

	co_yield util::coroutine::nextCycle();
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	co_yield util::coroutine::delay(300);// give time for the claw to actually close

	liftFlags->targetAngle = 10;
	liftFlags->errorThresh = 2;

	co_yield util::coroutine::nextCycle();
	co_yield [this]() -> bool { return !robot->getFlag<flags>().value()->isMoving; };
	liftFlags->isMotionRunning = false;
}

void Lift::toggleState() {
	setState(!robot->getSubsystem<flags>().value()->isOpen);
}

void Lift::setState(bool open) {
	robot->getSubsystem<flags>().value()->isOpen = open;

	if (open) {
		robot->registerTask([this]() { return this->openLiftCoro(); }, TaskType::SENTINEL);
	} else {
		robot->registerTask([this]() { return this->closeLiftCoro(); }, TaskType::SENTINEL);
	}
}

void Lift::move(int mv) {
	if (mv > 0 && (rotation.get_position() / 100.0) > UPPER_BOUNDS) { mv = 0; }
	if (mv < 0 && (rotation.get_position() / 100.0) < LOWER_BOUNDS) { mv = 0; }

	if (mv == 0) {
		motor.brake();
	} else {
		motor.move_voltage(mv);
	}
}