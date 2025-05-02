#include "main.h"
#include "../include/subsystems/AutonSelector.h"
#include "Constants.h"
#include "Robot.h"
#include "RobotBase.h"
#include "autons.h"
#include "lib/motion/OdomCharacterizationMotion.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/spline/Boomerang.h"
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

	robot_init();
	robotInstance->registerTask([]() { return autonomousUser(); }, TaskType::AUTON);

	// auton selector code using a pot to select
	// auto autonSelector = robotInstance->getSubsystem<AutonSelector>().value();
	// autonSelector->addAuton("Red Ring Side", redRingSide, true);
	// autonSelector->addAuton("Red Mogo Side", redMogoSide, true);
	// // autonSelector->addAuton("Red SAWP", redSAWP, true);
	//
	// autonSelector->addAuton("Blue Ring Side", blueRingSide, true);
	// autonSelector->addAuton("Blue Mogo Side", blueMogoSide, true);
	// autonSelector->addAuton("Blue SAWP", blueSAWP, true);

	// autonSelector->addAuton("Skills", skillsAuton, true);

	// everything's initialized, run the scheduler and startup the codebase
	// at this point, anything that touches our code should only come from coroutines
	// otherwise there will be potential for data races
	robotTask = pros::c::task_create(
	        [](void* robot) {
		        pros::lcd::initialize();
		        if (robot) { static_cast<decltype(robotInstance)>(robot)->run(); }
	        },
	        robotInstance, TASK_PRIORITY_DEFAULT, 0x4000, "Scheduler");
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

	// 600ms pid
	const double P = 810;
	const double I = 0;
	const double D = 6500;
	const uint32_t timeoutAmt = 600;

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

#include "FreeRTOSFuncs.h"
#include "lib/motion/BoomerangMotion.h"
RobotThread testBoomerangMotions() {
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

	// uint32_t totalTime = 0;
	// const uint32_t numItterations = 10000;
	//
	// for (int i = 0; i < numItterations; i++) {
	// 	enter_critical();
	// 	uint32_t startTime = pros::micros();
	// 	drive->setCurrentMotion(BoomerangMotion(odom->getCurrentState().position, Pose(20, 20, 1.5707963268), 0.45, 30, 40));
	// 	uint32_t endTime = pros::micros();
	// 	totalTime += (endTime - startTime);
	// 	// printf("Took %f ms\n", (endTime - startTime) / 1000.0);
	// 	exit_critical();
	// }
	//
	// printf("Took avg %f ms\n", (totalTime / 1000.0) / numItterations);

	drive->setCurrentMotion(OdomCharacterizationMotion());
	co_yield drive->waitUntilSettled(10000);

	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	robotInstance->getSubsystem<Odometry>().value()->reset();
	// auto skillsCoro = testBoomerangMotions();
	// auto skillsCoro = testAuton();
	auto skillsCoro = redSAWP();
	while (skillsCoro) { co_yield skillsCoro(); }

	// for skills

	// auto skillsCoro = skillsAuton();
	// while (skillsCoro) { co_yield skillsCoro(); }


	// auton selector code

	// if (robotInstance->autonFnPtr) {
	// 	auto autoCoro = robotInstance->autonFnPtr();
	// 	while (autoCoro) { co_yield autoCoro(); }
	// }

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