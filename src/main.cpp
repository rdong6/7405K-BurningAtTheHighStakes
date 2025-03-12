#include "main.h"
#include "Constants.h"
#include "Robot.h"
#include "RobotBase.h"
#include "autons.h"
#include "lib/motion/OdomCharacterizationMotion.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "lib/utils/ReferenceWrapper.h"
#include "lib/utils/Timeout.h"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"

RobotThread autonomousUser();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	// TODO: Fix logger so no unaligned accesses
	// logger_initialize("test.txt", 100);

	pros::lcd::initialize();
	robot_init();

	// robotInstance->curAlliance = Alliance::BLUE;

	// NOTE: This is bad code! There's always potential for data races because scheduler is running, so any other part of
	// codebase could register a task at the same time the initialize thread is registering the autonomousUser thread
	robotInstance->registerTask([]() { return autonomousUser(); }, TaskType::AUTON);
}

RobotThread testAuton() {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	auto drive = robotInstance->getSubsystem<Drive>().value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto intakeFlags = robotInstance->getFlag<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto pnoomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto pnoomaticFlags = robotInstance->getFlag<Pnooomatics>().value();
	auto odom = robotInstance->getSubsystem<Odometry>().value();
#pragma GCC diagnostic pop

	// working but fucked PID Constants: P = 750, I = 1, D = 5250

	// const double P = 750;
	// const double I = 1;
	// const double D = 5250;
	// const double timeoutAmt = 800;

	// 600ms pid
	const double P = 620;
	const double I = 1;
	const double D = 6500;
	const double timeoutAmt = 600;

	drive->setCurrentMotion(PIDTurn(-90, PID(615, 1, 6700, true, 10), false, false));
	// drive->setCurrentMotion(PIDTurn(120, PID(100, 1, 485, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	printf("Timed Out Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	odom->reset();

	while (!pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A)) {
		co_yield util::coroutine::nextCycle();
	}

	drive->setCurrentMotion(PIDTurn(25, PID(P, I, D, true, 10), false, false));
	// drive->setCurrentMotion(PIDTurn(120, PID(100, 1, 485, true, 10), false, false));
	co_yield drive->waitUntilSettled(timeoutAmt);
	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	odom->reset();

	while (!pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A)) {
		co_yield util::coroutine::nextCycle();
	}

	drive->setCurrentMotion(PIDTurn(45, PID(P, I, D, true, 10), false, false));
	// drive->setCurrentMotion(PIDTurn(120, PID(100, 1, 485, true, 10), false, false));
	co_yield drive->waitUntilSettled(timeoutAmt);
	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	odom->reset();

	while (!pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A)) {
		co_yield util::coroutine::nextCycle();
	}

	drive->setCurrentMotion(PIDTurn(65, PID(P, I, D, true, 10), false, false));
	co_yield drive->waitUntilSettled(timeoutAmt);

	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	odom->reset();

	while (!pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A)) {
		co_yield util::coroutine::nextCycle();
	}

	drive->setCurrentMotion(PIDTurn(90, PID(P, I, D, true, 10), false, false));
	co_yield drive->waitUntilSettled(timeoutAmt);

	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());
	odom->reset();

	while (!pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A)) {
		co_yield util::coroutine::nextCycle();
	}

	drive->setCurrentMotion(PIDTurn(130, PID(P, I, D, true, 10), false, false));
	co_yield drive->waitUntilSettled(timeoutAmt);

	co_yield util::coroutine::delay(250);
	printf("Final Heading: %f\n", odom->getCurrentState().position.rotation().degrees());


	// drive->setCurrentMotion(ProfiledMotion(15, 60, 125, 85));
	// co_yield drive->waitUntilSettled(2000);

	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	robotInstance->getSubsystem<Odometry>().value()->reset();

	// for skills

	// auto skillsCoro = skillsAuton();
	// while (skillsCoro) { co_yield skillsCoro(); }

	auto autoCoro = redRingSide();
	while (autoCoro) { co_yield autoCoro(); }

	// auto coro = redRingSide();
	// auto coro = blueMogoSide();
	// auto coro = redRingSide();
	// auto coro = redMogoSide();
	// auto coro = killsAuton();
	// auto coro = testAuton();
	// while (coro) { co_yield coro(); }

	co_yield util::coroutine::nextCycle();
}

// !!!!!!!!!!!!!!!!!!!!!
// DON'T USE THESE FUNCS
// !!!!!!!!!!!!!!!!!!!!!

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
	// static char buffer[2046];
	// static char buffer2[2046];
	// pros::delay(1000);
	// vTaskGetRunTimeStats(buffer);
	// printf("%s\n", buffer);
	// pros::delay(1000);
	// vTaskGetRunTimeStats(buffer2);
	// printf("%s\n", buffer2);
}

// conversion factor (abs time to ms): ((3/2)/1000000)