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

RobotThread redSAWP() {
	robotInstance->curAlliance = Alliance::RED;

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

	drive->setCurrentMotion(PIDTurn(40, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);


	lift->setState(Lift::STOW);
	liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};

	drive->setCurrentMotion(PIDTurn(20, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-34, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	drive->setCurrentMotion(PIDTurn(230, PID(200, 1, 75, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	//tune
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(17, 50, 60, 30));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(270, PID(200, 1, 45, true, 10), true, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 30));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(0, PID(200, 1, 75, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(34, 50, 60, 50));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(90, PID(200, 1, 75, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(48, 50, 60, 30));
	co_yield util::coroutine::delay(500);
	pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(2000);
	intake->setDistStop(true);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(PIDTurn(330, PID(200, 1, 75, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-30, 50, 60, 30));
	co_yield drive->waitUntilSettled(1000);
	pnoomatics->setClamp(false);
	intake->moveVoltage(12000);
	drive->setCurrentMotion(PIDTurn(270, PID(200, 1, 75, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 30));
	co_yield drive->waitUntilSettled(1000);
	liftFlags->targetAngle = 210;
	lift->setState(Lift::IDLE);
	co_yield util::coroutine::nextCycle();



	//ending statement
	co_yield util::coroutine::delay(5000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}