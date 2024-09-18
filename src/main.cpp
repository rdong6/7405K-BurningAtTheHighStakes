#include "main.h"
#include "FreeRTOSFuncs.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Pnooomatics.h"
#include <type_traits>

RobotThread autonomousUser();

void robot_init() {
	robotInstance = new std::decay<decltype(*robotInstance)>::type();
	robotInstance->registerTask([]() { return autonomousUser(); }, TaskType::AUTON);

	robotTask = pros::c::task_create(
	        [](void* robot) {
		        if (robot) { static_cast<decltype(robotInstance)>(robot)->run(); }
	        },
	        robotInstance, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Scheduler");
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	robot_init();
}

// USE THIS FUNC FOR AUTON CODING!
// thread runs after all other threads run
RobotThread redRingSideAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();

	drive->setCurrentMotion(ProfiledMotion(47, 50, 60, 60));
	co_yield util::coroutine::delay(1300);
	printf("Moving intake\n");
	intake->moveVoltage(-4500);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(500);
	printf("Moving forwards done\n");
	drive->setCurrentMotion(ProfiledMotion(-6, 50, 60, 60));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(PIDTurn(286, PID(150, 1, 50, true, 10), false, true));
	co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-3, 50, 20, 20));
	drive->setCurrentMotion(TimedMotion(300, -6000));
	co_yield drive->waitUntilSettled(500);
	pnooomatics->setClamp(true);
	co_yield util::coroutine::delay(300);
	robotInstance->getFlag<Intake::flags>().value()->distStop = false;
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(PIDTurn(315, PID(250, 30, 30, true, 10), false, true));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(20, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(4, PID(250, 30, 30, true, 10), true, false));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(TimedMotion(500, -6000));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::nextCycle();
}

RobotThread skillsAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	drive->setCurrentMotion(ProfiledMotion(-16, 50, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(90, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-18, 50, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);
	drive->setCurrentMotion(PIDTurn(188, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(20, 50, 50, 60));
	co_yield util::coroutine::delay(300);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1000);
	//go to wall stake
	// drive->setCurrentMotion(PIDTurn(240, PID(150, 1, 45, true, 10), false, false));
	// co_yield drive->waitUntilSettled(1000);
	// lift -> toggleState();
	// drive->setCurrentMotion(ProfiledMotion(48, 30, 50, 60));
	// co_yield drive->waitUntilSettled(1000);


	drive->setCurrentMotion(PIDTurn(270, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(24, 30, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(0, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(36, 25, 50, 50));
	co_yield drive->waitUntilSettled(2000);
	drive->setCurrentMotion(PIDTurn(236, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(14, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(138, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-8, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	pnooomatics->setClamp(false);
}

RobotThread autonomousUser() {
	auto coro = skillsAuton();
	while (coro) { co_yield coro(); }

	// auto driveOpt = robotInstance->getSubsystem<Drive>();
	// auto drive = driveOpt.value();
	// auto intake = robotInstance->getSubsystem<Intake>().value();
	// auto lift = robotInstance->getSubsystem<Lift>().value();
	// auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();

	// // move backwards to clamp mogo
	// drive->setCurrentMotion(TimedMotion(300, -6000));
	// co_yield drive->waitUntilSettled(300);
	// pnooomatics->setClamp(true);
	// co_yield util::coroutine::delay(300);// TBD

	// // turn to intake rings
	// drive->setCurrentMotion(PIDTurn(0 /* TBD */, PID(250, 30, 30, true, 10)));
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