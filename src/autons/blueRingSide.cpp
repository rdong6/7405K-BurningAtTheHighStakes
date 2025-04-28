#include "Robot.h"
#include "autons.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/motion/RAMSETEMotion.h"
#include "lib/motion/TimedMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"

RobotThread blueRingSide() {
	robotInstance->curAlliance = Alliance::BLUE;
	uint32_t startTime = pros::millis();

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

	drive->setCurrentMotion(ProfiledMotion(3, 60, 100, 60));
	co_yield util::coroutine::delay(200);
	// score on alliance stake w/ preload
	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	uint32_t start = pros::millis();
	co_yield drive->waitUntilSettled(2000);
	co_yield util::coroutine::delay(100 - (pros::millis() - start));

	Pose curPose = odom->getCurrentState().position;
	Pose mogo1(-43.2, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo1.translation()), 80, 120, 35));
	co_yield util::coroutine::delay(200);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(50);

	Pose centerRings(-51, 6.0);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRings).degrees(), PID(530, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRings.translation())-0.5, 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);
	pnoomatics->setLeftHammer(true);

	drive->setCurrentMotion(ProfiledMotion(4, 60, 100, 60));
	co_yield drive->waitUntilSettled(500);

	Pose temp1(-36.6, -1.1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(temp1.translation()), 60, 70, 60));
	co_yield drive->waitUntilSettled(2500);

	// // turn to heading 113 setup ring
	drive->setCurrentMotion(PIDTurn(87, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	pnoomatics->setLeftHammer(false);

	co_yield util::coroutine::delay(100);


	Pose ring2(-40.1, 19.4);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ring2).degrees(), PID(600, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring2.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	Pose ring3(-60.4, 20.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
		PIDTurn(curPose.headingTo(ring3).degrees(), PID(810, 1, 6000), false, false, 0.5, 12000, false, false));
	// drive->setCurrentMotion(
	// 		PIDTurn(190, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	Pose cornerSetup(-29.6, 19.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(cornerSetup.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	Pose corner(-10.7, 50.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
		PIDTurn(curPose.headingTo(corner).degrees(), PID(690, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 50, 100, 65));
	co_yield util::coroutine::delay(600);
	intake->moveVoltage(0);
	co_yield util::coroutine::delay(850);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(950);

	drive->setCurrentMotion(TimedMotion(400, 12000));
	co_yield drive->waitUntilSettled(400);

	drive->setCurrentMotion(ProfiledMotion(-36, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(24, 40, 100, 85));
	co_yield drive->waitUntilSettled(900);

}