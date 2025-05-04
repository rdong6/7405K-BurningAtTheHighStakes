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

RobotThread redMogoFake() {
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

	printf("Mogo rush\n");
	Pose mogoRush(37.5, 0);
	Pose curPose = odom->getCurrentState().position;
	intake->setDistStop(true);
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(mogoRush.translation()), 70, 120, 80));
	co_yield drive->waitUntilSettled(1500);

	// 25.6, 22.1
	printf("Mogo 1\n");
	Pose mogo1(26.3, 20.3);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	intake->moveVoltage(2000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield util::coroutine::delay(350);
	co_yield drive->waitUntilSettled(1100);
	pnoomatics->setClamp(true);
	intake->setDistStop(false);
	co_yield util::coroutine::delay(250);
	intake->moveVoltage(12000);

	// 40.3, 37.4, angle should be around 59
	printf("Center ring 1\n");
	Pose centerRing1(40.3, 40);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRing1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	pnoomatics->setRightHammer(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRing1.translation()), 60, 100, 75));
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(1500);

	printf("Center ring 2\n");
	// turn to 35.6 and move forwards 2 inches
	drive->setCurrentMotion(PIDTurn(35.6, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(400);
	drive->setCurrentMotion(ProfiledMotion(3, 60, 100, 75));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setLeftHammer(true);


	Pose dragBack(22.1, 25.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(dragBack).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(dragBack.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(281.1, PID(700, 1, 6500), false, false, 1, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	pnoomatics->setRightHammer(false);
	pnoomatics->setLeftHammer(false);
	intake->moveVoltage(12000);

	printf("Pre corner\n");
	Pose rings1(29.2, 4.0);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rings1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rings1.translation()) + 5, 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);

	Pose rings2(5.1, 15.0);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rings2).degrees(), PID(560, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rings2.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);


	printf("Corner\n");
	Pose corner(0.5, -24.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 15, 60, 100, 50));
	co_yield drive->waitUntilSettled(2500);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(15, 40, 60, 85));
	co_yield drive->waitUntilSettled(900);

	// CORNER CLEAR WOOOO
	co_yield util::coroutine::delay(750);

	pnoomatics->setRightHammer(true);
	co_yield util::coroutine::delay(250);

	drive->setCurrentMotion(PIDTurn(30, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	pnoomatics->setRightHammer(false);


	/*Pose preWallstake(5.6, -6.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(preWallstake).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
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
	co_yield drive->waitUntilSettled(1750);

	drive->setCurrentMotion(PIDTurn(323, PID(700, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	liftFlags->targetAngle = 155;
	lift->setState(Lift::HOLD);*/


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);
}