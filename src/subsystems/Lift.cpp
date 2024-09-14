#include "subsystems/Lift.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"
#include <limits>

Lift::Lift(RobotBase* robot) : Subsystem(robot, this) {
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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

	controllerRef->registerCallback([this]() { toggleState(); }, []() {}, Controller::master, Controller::a,
	                                Controller::rising);
}

RobotThread Lift::updateAngle() {
	auto liftFlags = robot->getFlag<flags>().value();
	while (true) {
		int32_t pos = rotation.get_position();

		if (pos == std::numeric_limits<int32_t>::max()) {
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
	liftFlags->targetAngle = 84;
	// this->pid = PID(150, 5, 50);
	liftFlags->errorThresh = 3;


	co_yield util::coroutine::nextCycle();

	if (!liftFlags->isMotionRunning) { co_return; }

	// wait to move to place
	// maybe change this condition to make it faster/more fluid
	co_yield [this]() -> bool { return !robot->getFlag<flags>().value()->isMoving; };

	claw.set_value(true);
	liftFlags->targetAngle = 66;
	// this->pid = PID(50, 0, 50);
	liftFlags->errorThresh = 3.5;

	co_yield util::coroutine::nextCycle();
	if (!liftFlags->isMotionRunning) { co_return; }
	co_yield [this]() -> bool { return !robot->getFlag<flags>().value()->isMoving; };
	liftFlags->isMotionRunning = false;
}

RobotThread Lift::closeLiftCoro() {
	auto liftFlags = robot->getFlag<flags>().value();
	liftFlags->isMotionRunning = true;
	liftFlags->targetAngle = 100;
	// this->pid = PID(150, 5, 50);
	liftFlags->errorThresh = 3;

	co_yield util::coroutine::nextCycle();
	co_yield [this]() -> bool { return !robot->getFlag<flags>().value()->isMoving; };

	claw.set_value(false);
	co_yield util::coroutine::delay(100);

	liftFlags->isMotionRunning = true;
	liftFlags->targetAngle = 10;
	// this->pid = PID(100, 5, 50);
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
	if (mv > 0 && (rotation.get_position() / 100.0) > 181) { mv = 0; }
	if (mv < 0 && (rotation.get_position() / 100.0) < 10) { mv = 0; }

	motor.move_voltage(mv);
}