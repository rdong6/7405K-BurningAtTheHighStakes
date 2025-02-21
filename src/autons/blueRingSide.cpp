#include "Robot.h"
#include "autons.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/motion/TimedMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Timeout.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"


RobotThread blueRingSide() {
	robotInstance->curAlliance = Alliance::BLUE;

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
	
	drive->setCurrentMotion(PIDTurn(315, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1000);

	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(PIDTurn(340, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-34.5, 50, 60, 22));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	// drive->setCurrentMotion(PIDTurn(230, PID(200, 1, 75, true, 10), false, false, 0.5)); // original
	drive->setCurrentMotion(PIDTurn(130, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	// tune
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(16, 50, 60, 50));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(90, PID(150, 1, 75, true, 10), false, true, 0.5));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 50));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(200);
	drive->setCurrentMotion(PIDTurn(352, PID(150, 1, 125, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(16, 50, 60, 50));
	co_yield drive->waitUntilSettled(1500);
    co_yield util::coroutine::delay(200);
	
	// move to get last ring
	drive->setCurrentMotion(PIDTurn(283, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(45, 50, 60, 50));
	co_yield drive->waitUntilSettled(2500);
    drive->setCurrentMotion(ProfiledMotion(13, 30, 60, 30));
	co_yield drive->waitUntilSettled(2500);

// turn to hit ladder
co_yield util::coroutine::delay(100);
	drive->setCurrentMotion(PIDTurn(175, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	// drive->setCurrentMotion(TimedMotion(1500, 8000));
	liftFlags->targetAngle = 160;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(12, 20, 40, 30));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	co_yield drive->waitUntilSettled(1500);

	

	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);

}