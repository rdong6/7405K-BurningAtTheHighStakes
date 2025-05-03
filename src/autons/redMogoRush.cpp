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

RobotThread redMogoRush() {
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
	Pose mogo1(26.3, 20.3);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(180 + curPose.headingTo(mogo1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	lift->setState(Lift::LEVEL_1);
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	intake->setDistStop(false);
	co_yield util::coroutine::delay(100);

	printf("Pre corner\n");
	Pose preCorner(2.3, 2.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(preCorner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	lift->setState(Lift::LEVEL_1);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCorner.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);

	printf("Corner\n");
	Pose corner(-6.4, -22.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	// this is to give more time for lady brown to get to position before loading ring into it
	liftTimeout = Timeout(150);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(550);

	// move to corner. while moving to corner, lift lady brown up so it clears rings in intake
	uint32_t cornerStartTime = pros::millis();
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 15, 60, 100, 50));

	Timeout intakeStallTimeout = Timeout(500); // timeout for how long we wait for intake to stall/ring to load into lady brown
	co_yield [=]() { return intakeStallTimeout.timedOut() || intake->isStalled(); };
	liftFlags->targetAngle = 100;
	intakeFlags->ladyBrownClearanceEnabled = true;
	lift->setState(Lift::HOLD);
	liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(2500 - (pros::millis() - cornerStartTime)); // motion to corner takes 2.5s max. lift timeout stuff now won't affect it
	intake->moveVoltage(12000);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(12, 40, 60, 85));
	co_yield drive->waitUntilSettled(900);

	// CORNER CLEAR WOOOO
	co_yield util::coroutine::delay(750);

	pnoomatics->setRightHammer(true);
	co_yield util::coroutine::delay(250);

	drive->setCurrentMotion(
			PIDTurn(30, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	pnoomatics->setRightHammer(false);


	Pose preWallstake(5.6, -6.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(180 + curPose.headingTo(preWallstake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(preWallstake.translation()), 60, 100, 70));
	co_yield drive->waitUntilSettled(1500);

	Pose wallstake(49.5, -13.9);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
			PIDTurn(curPose.headingTo(wallstake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallstake.translation()), 60, 100, 70));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(
			PIDTurn(322.6, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	liftFlags->targetAngle = 155;
	lift->setState(Lift::HOLD);*/


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);
}