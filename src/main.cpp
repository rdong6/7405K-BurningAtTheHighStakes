#include "main.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/controller/RAMSETE.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/DelayedBool.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"
#include <type_traits>


#include "Logger.h"
#include "lib/geometry/Rotation2D.h"
#include "lib/geometry/Transform2D.h"
#include "lib/geometry/Translation2D.h"
#include "lib/geometry/Twist2D.h"

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
	logger_initialize("test.txt", 100);
	robot_init();
	pros::delay(250);
	robotInstance->getSubsystem<Odometry>().value()->reset();
}

RobotThread testAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto pnoomaticsFlags = robotInstance->getFlag<Pnooomatics>().value();

	//
	pnoomaticsFlags->clampMogo = true;
	drive->setCurrentMotion(ProfiledMotion(24, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(90, PID(24, 50, 60)));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(30, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(PIDTurn(0, PID(24, 50, 60)));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(20, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(PIDTurn(270, PID(24, 50, 60)));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);
}

// USE THIS FUNC FOR AUTON CODING!
// thread runs after all other threads run
RobotThread redRingSideAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	drive->setCurrentMotion(ProfiledMotion(34.25, 50, 60, 60));
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);
	lift->setClaw(true);

	drive->setCurrentMotion(PIDTurn(314, PID(190, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(-16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	drive->setCurrentMotion(PIDTurn(30, PID(190, 1, 50, true, 10), false, true));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(10, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-50, 50, 60, 60));
	co_yield drive->waitUntilSettled(3000);
	drive->setCurrentMotion(PIDTurn(120, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// intake->setExtender(true);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(12, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(false);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(137, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(18.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(217, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// drive->setCurrentMotion(ProfiledMotion(3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;

	// put ring on stake
	lift->setClaw(false);
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	liftFlags->pid = PID(400, 10, 0, true, 10);
	liftFlags->targetAngle = 0;

	co_yield util::coroutine::delay(500);
	liftFlags->isMotionRunning = false;

	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread skillsAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto odom = robotInstance->getSubsystem<Odometry>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

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
	co_yield drive->waitUntilSettled(700);
	drive->setCurrentMotion(ProfiledMotion(-18.5, 50, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	// mogo clamped -> turn to intake first ring
	drive->setCurrentMotion(PIDTurn(188, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(700);
	drive->setCurrentMotion(ProfiledMotion(22, 50, 50, 60));
	co_yield util::coroutine::delay(300);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1250);

	// move to second ring (by wall stake) -> put ring on stake
	drive->setCurrentMotion(PIDTurn(212, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(30, 30, 50, 60));// move towards ring to place on stake
	co_yield util::coroutine::delay(300);
	// lift->toggleState();
	co_yield drive->waitUntilSettled(2000);
	// turn towards wall stake
	drive->setCurrentMotion(PIDTurn(265.5, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(18.5, 30, 50, 60));// move towards ring to place on stake
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(1000);// replace this with a torque stop
	intake->moveVoltage(0);

	// move lift to score the ring on wall stake
	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;
	co_yield util::coroutine::nextCycle();
	Timeout liftScoreTimeout(2000);
	co_yield [=]() -> bool { return !liftFlags->isMoving || liftScoreTimeout.timedOut(); };
	// lift->setState(false);

	// lift pid

	// move back to intake the line of 3 rings
	drive->setCurrentMotion(ProfiledMotion(-9, 30, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(353, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(54, 23, 50, 50));
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(4000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(PIDTurn(240, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(11, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(-3, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);

	// dump mogo
	drive->setCurrentMotion(PIDTurn(135, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-8, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	pnooomatics->setClamp(false);
	intake->moveVoltage(0);

	// move to center
	drive->setCurrentMotion(ProfiledMotion(78, 50, 50, 60));
	co_yield util::coroutine::delay(1800);
	intake->setDistStop(true);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(3000);

	// turn to second corner
	drive->setCurrentMotion(PIDTurn(50, PID(153, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	// ring and setup for mogo
	drive->setCurrentMotion(ProfiledMotion(45, 50, 50, 60));
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(-5000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);

	// grab mogo
	drive->setCurrentMotion(PIDTurn(150, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-15.5, 50, 50, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	// rings
	drive->setCurrentMotion(PIDTurn(123, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(30.5, 50, 50, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(0, PID(190, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(32, 23, 50, 60));
	co_yield drive->waitUntilSettled(3000);

	drive->setCurrentMotion(PIDTurn(120, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(13, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(-3, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);

	// dump mogo
	drive->setCurrentMotion(PIDTurn(218, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-8, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	pnooomatics->setClamp(false);

	// go to second wall stake
	drive->setCurrentMotion(ProfiledMotion(6, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(167, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(50, 40, 50, 60));
	// lift->toggleState();
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(3, 40, 50, 60));
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(1500);

	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;
	co_yield util::coroutine::nextCycle();
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	// lift->setState(false);

	/*intake->moveVoltage(12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(PIDTurn(200, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(24, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(270, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(20, 40, 50, 60));
	co_yield drive->waitUntilSettled(1500); */
}

RobotThread blueRingSideAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	drive->setCurrentMotion(ProfiledMotion(34, 50, 60, 60));
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);
	lift->setClaw(true);

	drive->setCurrentMotion(PIDTurn(46, PID(190, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(-16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	drive->setCurrentMotion(PIDTurn(330, PID(190, 1, 50, true, 10), true, false));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(350, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-50, 50, 60, 60));
	co_yield drive->waitUntilSettled(3000);
	drive->setCurrentMotion(PIDTurn(215, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// intake->setExtender(true);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(13, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(false);
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(234, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(18.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(140, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	lift->setClaw(false);
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	liftFlags->pid = PID(400, 10, 0, true, 10);
	liftFlags->targetAngle = 0;

	co_yield util::coroutine::delay(500);
	liftFlags->isMotionRunning = false;


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread blueMogoRush() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	robotInstance->getFlag<Intake>().value()->distStop = true;
	intake->moveVoltage(-12000);

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	// rush to center
	drive->setCurrentMotion(ProfiledMotion(43, 50, 60, 60));
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
	robotInstance->getFlag<Intake>().value()->distStop = false;

	// get the second mogo
	drive->setCurrentMotion(ProfiledMotion(-19, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);
	intake->moveVoltage(-12000);

	// move to clear corner
	drive->setCurrentMotion(PIDTurn(155, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1800);
	pnooomatics->setHammer(false);

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

RobotThread redMogoRush() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	robotInstance->getFlag<Intake>().value()->distStop = true;
	intake->moveVoltage(-12000);

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	// rush to center
	drive->setCurrentMotion(ProfiledMotion(43, 50, 60, 60));
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
	robotInstance->getFlag<Intake>().value()->distStop = false;

	// get the second mogo
	drive->setCurrentMotion(ProfiledMotion(-19, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);
	intake->moveVoltage(-12000);

	// move to alliance stake
	drive->setCurrentMotion(PIDTurn(240, PID(150, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	// intake->setExtender(true);
	robotInstance->getFlag<Intake>().value()->distStop = true;
	// intake->moveVoltage(-12000);

	drive->setCurrentMotion(ProfiledMotion(30, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);


	drive->setCurrentMotion(ProfiledMotion(-3, 20, 30, 30));
	// intake->setExtender(false);
	co_yield drive->waitUntilSettled(1500);

	// // co_yield []() -> bool { return !robotInstance->getFlag<Intake::flags>().value()->distStop; };

	// drive->setCurrentMotion(ProfiledMotion(2, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	// drive->setCurrentMotion(PIDTurn(210, PID(150, 1, 45, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(12, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	auto coro = redRingSideAuton();
	while (coro) { co_yield coro(); }

	// auto coro = skillsAuton();
	// while (coro) { co_yield coro(); }

	// auto coro = redMogoRush();
	// while (coro) { co_yield coro(); }

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