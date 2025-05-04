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

namespace {
	// macro to help with the corner
	// loads first ring in corner onto mogo
	// sets dist stop to true and moves lady brown into load position
	// loads second ring onto lady brown
	RobotThread cornerLoadingMacro() {
		auto intake = robotInstance->getSubsystem<Intake>().value();
		auto intakeFlags = robotInstance->getFlag<Intake>().value();
		auto lift = robotInstance->getSubsystem<Lift>().value();
		auto liftFlags = robotInstance->getFlag<Lift>().value();

		intake->moveVoltage(12000);
		while (!intake->ringAtDistSensor()) { co_yield util::coroutine::nextCycle(); }// see the first ring pass by dist sensor
		co_yield util::coroutine::delay(400);
		while (!intake->ringAtDistSensor()) { co_yield util::coroutine::nextCycle(); }// see the first ring pass by dist sensor
		co_yield util::coroutine::delay(400);
		while (!intake->ringAtDistSensor()) { co_yield util::coroutine::nextCycle(); }// see the first ring pass by dist sensor
		intake->setDistStop(true);
		co_yield util::coroutine::delay(75);// ensure ring has passed by dist sensor
		lift->setState(Lift::LEVEL_1);

		Timeout liftTimeout = Timeout(200);
		co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

		intake->setDistStop(false);
		intake->moveVoltage(12000);

		// co_yield [&]() { return intake->isStalled(); };
		// co_yield util::coroutine::delay(50);
		//
		// intakeFlags->ladyBrownClearanceEnabled = true;
		// liftFlags->targetAngle = 100;
		// lift->setState(Lift::HOLD);
		// liftTimeout = Timeout(200);
		// co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
		// intake->moveVoltage(12000);
	}
}

RobotThread blueMogoRush() {
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
	uint32_t startTime = pros::millis();

	liftFlags->targetAngle = 160;
	lift->setState(Lift::HOLD);
	liftFlags->pid = PID(100, 0, 50, true, 10);

	printf("Mogo rush\n");
	Pose mogoRush(41.5, 0);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(mogoRush.translation()), 70, 120, 80));
	co_yield util::coroutine::delay(500);
	intake->setDistStop(true);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);

	liftFlags->targetAngle = 240;
	lift->setState(Lift::HOLD);
	liftFlags->pid = PID(1000, 0, 50);
	Timeout liftTimeout = Timeout(200);
	co_yield [=]() { return liftFlags->curAngle >= 230 || liftTimeout.timedOut(); };

	// 25.6, 22.1
	printf("Mogo 1\n");
	Pose mogo1(26.3, -20.3);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(180 + curPose.headingTo(mogo1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(560);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield util::coroutine::delay(350);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1100);
	pnoomatics->setClamp(true);
	intake->setDistStop(false);
	co_yield util::coroutine::delay(100);

	printf("Pre corner\n");
	Pose preCorner(13, 12);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(preCorner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	robotInstance->registerTask([]() { return ::cornerLoadingMacro(); },
								TaskType::AUTON);// registers additional thread to just help w/ this corner clearing macro
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCorner.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);

	printf("Corner\n");
	Pose corner(-6.2, 29.9);
	// for corner, intake first ring onto mogo
	// intake 2nd ring onto lady brown -> do this by waiting for first ring to be in intake, enable dist stop. wait until first
	// ring is laoded. then move lady brown into loading position. then run intake

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 20, 60, 100, 50));
	co_yield drive->waitUntilSettled(2500);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(20, 40, 60, 85));
	co_yield drive->waitUntilSettled(900);

	// CORNER CLEAR WOOOO
	co_yield util::coroutine::delay(750);

	pnoomatics->setLeftHammer(true);
	co_yield util::coroutine::delay(250);

	drive->setCurrentMotion(
			PIDTurn(330, PID(810, 1, 6500), false, false, 0.5, 12000, false, true));
	co_yield drive->waitUntilSettled(550);

	pnoomatics->setLeftHammer(false);


	printf("Pre wallstake\n");
	Pose preWallstake(10.6, 6.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(180 + curPose.headingTo(preWallstake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(preWallstake.translation()), 60, 100, 70));
	co_yield drive->waitUntilSettled(1500);

	printf("Wallstake\n");
	Pose wallstake(43.6, 25);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(wallstake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallstake.translation()), 60, 100, 70));
	co_yield drive->waitUntilSettled(1750);

	drive->setCurrentMotion(PIDTurn(42, PID(720, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	liftFlags->targetAngle = 155;
	lift->setState(Lift::HOLD);

	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);
}