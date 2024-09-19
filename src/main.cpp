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
	auto liftFlags = robotInstance->getFlag<Lift::flags>().value();

	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;

	// put ring on stake
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	liftFlags->pid = PID(400, 10, 0, true, 10);
	liftFlags->targetAngle = 0;

	// move to clamp first mogo
	drive->setCurrentMotion(ProfiledMotion(-10, 50, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	liftFlags->isMotionRunning = false;
	drive->setCurrentMotion(PIDTurn(88, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-18, 50, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	// mogo clamped -> turn to intake first ring
	drive->setCurrentMotion(PIDTurn(188, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(22, 50, 50, 60));
	co_yield util::coroutine::delay(300);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1250);

	// move to second ring (by wall stake) -> put ring on stake
	drive->setCurrentMotion(PIDTurn(215, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(30, 30, 50, 60));// move towards ring to place on stake
	co_yield drive->waitUntilSettled(2000);
	// turn towards wall stake
	drive->setCurrentMotion(PIDTurn(265, PID(150, 1, 45, true, 10), false, false));
	lift->toggleState();
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(18, 30, 50, 60));// move towards ring to place on stake
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(1000);// replace this with a torque stop
	intake->moveVoltage(0);

	// move lift to score the ring on wall stake
	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;
	co_yield util::coroutine::nextCycle();
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	lift->setState(false);

	// lift pid

	// move back to intake the line of 3 rings
	drive->setCurrentMotion(ProfiledMotion(-8, 30, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(356, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(59, 25, 50, 50));
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(4000);
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

RobotThread blueRingSideAuton() {
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

RobotThread blueMogoRush() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift::flags>().value();

	robotInstance->getFlag<Intake::flags>().value()->distStop = true;
	intake->moveVoltage(-12000);

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	// rush to center
	drive->setCurrentMotion(ProfiledMotion(43 , 50, 60, 60));
	co_yield drive->waitUntilDist(37);
	lift->setClaw(true);
	pnooomatics->setHammer(true);// set hammer down
	co_yield drive->waitUntilSettled(2000);

	// move back and turn towards second mogo
	drive->setCurrentMotion(ProfiledMotion(-10.5, 40, 30, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(110, PID(250, 1, 45, true, 10), false, false, 0.5, 5500));
	co_yield drive->waitUntilSettled(1800);
	pnooomatics->setHammer(false);// set hammer up
	co_yield util::coroutine::delay(200);
	robotInstance->getFlag<Intake::flags>().value()->distStop = false;

	// get the second mogo
	drive->setCurrentMotion(ProfiledMotion(-19, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);
	intake->moveVoltage(-12000);

	// move to alliance stake
	// drive->setCurrentMotion(PIDTurn(245, PID(150, 1, 45, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// intake->setExtender(true);
	// robotInstance->getFlag<Intake::flags>().value()->distStop = true;
	// // intake->moveVoltage(-12000);

	// drive->setCurrentMotion(ProfiledMotion(30, 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);


	// drive->setCurrentMotion(ProfiledMotion(-3, 20, 30, 30));
	// intake->setExtender(false);
	// co_yield drive->waitUntilSettled(1500);
	
	// // co_yield []() -> bool { return !robotInstance->getFlag<Intake::flags>().value()->distStop; };

	// drive->setCurrentMotion(ProfiledMotion(2, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	// drive->setCurrentMotion(PIDTurn(210, PID(150, 1, 45, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(12, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	// auto coro = redRingSideAuton();
	// while (coro) { co_yield coro(); }

	// auto coro = skillsAuton();
	// while (coro) { co_yield coro(); }

	auto coro = blueMogoRush();
	while (coro) { co_yield coro(); }

	// auto driveOpt = robotInstance->getSubsystem<Drive>();
	// auto drive = driveOpt.value();
	// auto intake = robotInstance->getSubsystem<Intake>().value();
	// auto lift = robotInstance->getSubsystem<Lift>().value();
	// auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();

	// co_yield util::coroutine::nextCycle();

	// drive->setCurrentMotion(ProfiledMotion(-45, 50, 60, 60));
	//

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