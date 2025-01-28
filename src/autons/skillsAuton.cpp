#include "Robot.h"
#include "autons.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/DriveCharacterizationMotion.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/motion/RAMSETEMotion.h"
#include "lib/motion/TimedMotion.h"
#include "lib/trajectory/constraints/CentripetalAccelerationConstraint.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "lib/utils/ReferenceWrapper.h"
#include "lib/utils/Timeout.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"

RobotThread realSkillsAuton() {
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


	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::STOW);

	drive->setCurrentMotion(PIDTurn(270, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-17, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);

	intake->moveVoltage(12000);

	drive->setCurrentMotion(PIDTurn(167, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(18, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(157, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(58, 50, 60, 60));
	co_yield util::coroutine::delay(750);
	lift->setState(Lift::LEVEL_1);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(1250);
	drive->setCurrentMotion(PIDTurn(172, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);


	drive->setCurrentMotion(ProfiledMotion(-22, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 50;
	intakeFlags->ladyBrownClearanceEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(22, 30, 40, 30));
	co_yield util::coroutine::delay(250);// delay needed as ladyBrownClearanceEnabled stops intake after 60ms
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(2000);

	liftFlags->targetAngle = 150;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	drive->setCurrentMotion(ProfiledMotion(-10, 30, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(0, PID(750, 0, 5500, true, 10), false, false, 0.5));
	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(58, 25, 40, 30));
	co_yield drive->waitUntilSettled(5000);

	drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(15, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(ProfiledMotion(-5, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(225, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-5, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(false);
	// first corner is done

	/*drive->setCurrentMotion(ProfiledMotion(12, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(70, 50, 50, 30));
	co_yield drive->waitUntilSettled(4000);

	pnoomatics->setClamp(true);

	intake->moveVoltage(12000);

	drive->setCurrentMotion(PIDTurn(185, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(20, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(210, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(50, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	lift->setState(Lift::LEVEL_1);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(-25, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(270, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 50;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(25, 30, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	liftFlags->targetAngle = 150;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};

	drive->setCurrentMotion(ProfiledMotion(-10, 30, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(0, PID(750, 0, 5500, true, 10), false, false, 0.5));
	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(55, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(225, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(15, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-5, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);*/
}