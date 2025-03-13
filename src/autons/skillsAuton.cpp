#include "Robot.h"
#include "autons.h"
#include "lib/geometry/kinState.h"
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

RobotThread skillsAuton() {
	// TODO: REENABLE THIS!
	// robotInstance->curAlliance = Alliance::RED;

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

	// TODO: UNCOMMENT POST ADI's HOUSE
	liftFlags->targetAngle = 220;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	co_yield util::coroutine::delay(250);
	drive->setCurrentMotion(ProfiledMotion(-7, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);


	printf("First Mogo\n");
	Pose firstMogo(-7.6, 19.3);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(firstMogo).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(firstMogo.translation()), 75, 120, 30));
	co_yield drive->waitUntilSettled(1000);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	printf("M1 ring1\n");
	Pose firstRing(-32.0, 19);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(firstRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(firstRing.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("M1 ring2\n");
	Pose secondRing(-76.3, 46.0);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(secondRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(280);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondRing.translation()) + 10, 75, 120, 45));
	Timeout timeout = Timeout(1100);
	co_yield
	        [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() >= 1 || timeout.timedOut(); };
	co_yield
	        [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() == 0 || timeout.timedOut(); };
	lift->setState(Lift::LEVEL_1);
	co_yield drive->waitUntilSettled(5000);

	Pose thirdRing(-98.0, 64.5);

	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(thirdRing.translation()) + 3,75, 120, 40));
	// co_yield drive->waitUntilSettled(5000);
	// // NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	// lift->setState(Lift::LEVEL_1);

	printf("Lineup Wallstake 1\n");
	Pose lineUpWallStake1(-58.5, 44.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake1).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(300);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake1.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 50;
	intakeFlags->ladyBrownClearanceEnabled = true;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);

	printf("Wallstake 1\n");
	Pose wallStake1(-51.2, 55.7);
	drive->setCurrentMotion(PIDTurn(90, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake1.translation()) + 7.5, 75, 120, 60));
	co_yield drive->waitUntilSettled(2000);


	// score first ring
	liftFlags->targetAngle = 210;
	liftFlags->slewRate = 1000;
	// liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(800);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;

	// move lift down and load second ring into intake
	lift->setState(Lift::STOW);

	printf("Move back wallstake 1\n");
	// move back after scoring on wallstake to align with the row of rings
	Pose moveBack1(-55.6, 50.05);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(moveBack1).degrees(), PID(620, 1, 6500), false, false, 2, 12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack1.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(12000);

	printf("Row of rings 1\n");
	Pose rowOfRings(6.92, 52.5);

	curPose = odom->getCurrentState().position;
	// bumped up D from 6500
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rowOfRings).degrees(), PID(620, 1, 6550), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings.translation()), 35, 90, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Align Corner 1\n");
	Pose alignCornerPoint(-9.6, 63.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(alignCornerPoint).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignCornerPoint.translation()) + 3, 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Corner 1\n");
	Pose cornerPoint(0, 64);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(cornerPoint).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(cornerPoint.translation()), 40, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);
	co_yield util::coroutine::delay(100);

	printf("Align mogo 2\n");
	Pose alignMogo2(-9.2, 56.8);

	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(
	//         PIDTurn(curPose.headingTo(alignMogo2).degrees(), PID(600, 1, 6800), false, false, 0.5, 12000, false, false));
	// co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignMogo2.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	intake->moveVoltage(0);

	printf("Moving to mogo2\n\n");
	Pose mogo2Midpoint(-7.8, -4);
	Pose mogo2(-7, -20.7);


	// do a 2 part movement to mogo
	co_yield util::coroutine::delay(250);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(mogo2Midpoint).degrees(), PID(620, 1, 6700), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(400);

	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2Midpoint.translation()), 80, 85, 85));
	co_yield drive->waitUntilSettled(2000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(620, 1, 6500), false, false, 2.5, 12000, false, false));
	co_yield drive->waitUntilSettled(300);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 40, 100, 35));
	co_yield drive->waitUntilSettled(5000);
	pnoomatics->setClamp(true);

	printf("M2 ring1\n");
	Pose fourthRing(-25.7, -24.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(fourthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(450);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fourthRing.translation()) + 3, 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("M2 ring2\n");
	Pose fifthRing(-75, -50);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(fifthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(450);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fifthRing.translation()) + 6.5, 75, 120, 40));
	timeout = Timeout(1100);
	co_yield
	        [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() >= 1 || timeout.timedOut(); };
	co_yield
	        [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() == 0 || timeout.timedOut(); };
	lift->setState(Lift::LEVEL_1);
	co_yield drive->waitUntilSettled(2500);

	// Pose sixthRing(-98.8, -58.5);
	//
	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(
	//         PIDTurn(curPose.headingTo(sixthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	// co_yield drive->waitUntilSettled(500);
	//
	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(sixthRing.translation()) + 3,75, 120, 40));
	// co_yield drive->waitUntilSettled(5000);
	// // NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	// lift->setState(Lift::LEVEL_1);


	printf("Lineup wallstake 2\n");
	Pose lineUpWallStake2(-54, -44.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake2).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake2.translation()), 60, 100, 50));
	co_yield drive->waitUntilSettled(5000);


	printf("Wallstake 2\n");
	Pose wallStake2(-50.8, -66.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(270, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	intakeFlags->ladyBrownClearanceEnabled = true;
	liftFlags->targetAngle = 50;
	intakeFlags->ladyBrownClearanceEnabled = true;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake2.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(2000);

	// score the ring onto wallstake
	liftFlags->targetAngle = 210;
	// liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(750);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;

	lift->setState(Lift::STOW);

	printf("Move back wallstake 2\n");
	Pose backWallStake(-53.0, -52.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(backWallStake).degrees(), PID(620, 1, 6500), false, false, 2, 12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(backWallStake.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Row of rings 2\n");
	Pose rowOfRings2(9.6, -51.5);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rowOfRings2).degrees(), PID(620, 1, 6520), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(700);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings2.translation()), 30, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	co_yield util::coroutine::delay(100);

	printf("Setup corner 2\n");
	Pose setUpCorner2(-11, -65.1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(setUpCorner2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(470);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpCorner2.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Corner 2\n");
	Pose corner2(2, -64.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(corner2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner2.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);


	printf("Pre seventh ring\n");
	Pose setUpRingSeventhRing(-58.2, -42.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(setUpRingSeventhRing).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpRingSeventhRing.translation()), 75, 120, 80));
	co_yield drive->waitUntilSettled(5000);

	printf("Seventh ring\n");
	Pose seventhRing(-78.7, -25.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(seventhRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	lift->setState(Lift::LEVEL_1);
	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(seventhRing.translation()) + 5, 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Mogo 3\n");
	Pose mogo3(-104.2, 3.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo3).degrees(), PID(470, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(1045);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo3.translation()), 75, 120, 40));
	co_yield drive->waitUntilSettled(1000);

	pnoomatics->setClamp(true);

	printf("Alliance stake\n");
	Pose allianceStake(-118, -4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceStake).degrees(), PID(450, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(650);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceStake.translation()) + 4, 75, 120, 60));
	intakeFlags->ladyBrownClearanceEnabled = true;
	co_yield drive->waitUntilSettled(5000);
	// TUNE THIS DISTANCE
	drive->setCurrentMotion(ProfiledMotion(-8, 75, 120, 60));
	co_yield drive->waitUntilSettled(1500);

	liftFlags->targetAngle = 220;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	pros::delay(250);
	drive->setCurrentMotion(ProfiledMotion(-9, 50, 75, 45));
	co_yield drive->waitUntilSettled(1500);

	/*printf("M3 ring1\n");
	Pose m3RingOne(-106.4, 45.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingOne).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingOne.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("M3 ring2\n");
	Pose m3RingTwo(-112.3, 45.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingTwo).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(450);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingTwo.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingThree(-103, 58);
	printf("M3 ring3\n");
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingThree).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(450);

	curPose = odom->getCurrentState().position;
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingThree.translation()), 75, 120, 60));
	co_yield util::coroutine::delay(100);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(5000);

	printf("Pre center ring\n");
	Pose preCenterRing(-75.7, 25.0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preCenterRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(700);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCenterRing.translation()) + 4, 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Corner 3 Dump\n");
	Pose corner3Dump(-115.0, 53.5);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(corner3Dump).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(300);
	drive->setCurrentMotion(ProfiledMotion(-15, 75, 120, 70));
	co_yield drive->waitUntilSettled(5000);
	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);
	co_yield util::coroutine::delay(150);
	intake->moveVoltage(0);
	drive->setCurrentMotion(TimedMotion(800, -10500));
	co_yield drive->waitUntilSettled(800);

	printf("Pre Mogo Jam\n");
	Pose preMogoJam(-93.7, 35.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preMogoJam).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(450);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preMogoJam.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Setup Mogo Jam\n");
	Pose setupMogoJam(-111.6, -10.0);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(setupMogoJam).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(450);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(setupMogoJam.translation()), 75, 120, 80));
	co_yield drive->waitUntilSettled(1000);

	pnoomatics->setClamp(true);
	drive->setCurrentMotion(TimedMotion(900, -10000));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(TimedMotion(250, 6000));
	co_yield drive->waitUntilSettled(250);

	printf("Hanging\n");
	Pose hangPose(-70.3, -11.87);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(hangPose).degrees(), PID(620, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(650);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(hangPose.translation()) - 15, 75, 120, 55));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(TimedMotion(250, 10000));
	co_yield drive->waitUntilSettled(250);*/


	/*printf("Center ring\n");
	Pose centerRing(-53.6, -1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRing.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("M3 ring3\n");
	Pose m3RingThree(-100.2, -50.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingThree).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingThree.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("M3 ring4\n");
	Pose m3RingFour(-105, -50.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingFour).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingFour.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	// ref heading: 47.2
	printf("Corner 3 dump\n");
	Pose corner3Dump(-110.5, -55.3);

	curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(
	//         PIDTurn(180 + curPose.headingTo(corner3Dump).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false,
	//         false));
	drive->setCurrentMotion(PIDTurn(60, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	// move forwards, clamp, ram mogo into corner
	drive->setCurrentMotion(ProfiledMotion(8, 75, 60, 60));
	co_yield util::coroutine::delay(250);
	pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1000);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(100);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner3Dump.translation()), 75, 120, 60));
	pnoomatics->setRightHammer(false);
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);

	printf("Pre mogo 4\n");
	Pose preMogo4(-96.4, -32.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preMogo4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(250);

	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preMogo4.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	printf("Mogo 4\n");
	Pose mogo4(-113.8, 20.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	pnoomatics->setClamp(true);
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo4.translation()), 75, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	*/


	/*drive->setCurrentMotion(TimedMotion(1000, -12000));
	co_yield drive->waitUntilSettled(1000);

	// pnoomatics->setClamp(true);

	Pose corner4(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	pnoomatics->setRightHammer(true);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner4.translation()),75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	// turn to dump

	Pose preHang(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preHang).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	pnoomatics->setRightHammer(false);
	pnoomatics->setClamp(false);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-10,75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preHang).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preHang.translation()),75, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 180;
	lift->setState(Lift::HOLD);


	Pose Hang(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(Hang).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false,
	false)); co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(Hang.translation()),75, 120, 30));
	co_yield drive->waitUntilSettled(5000);
	*/
	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
	co_yield util::coroutine::nextCycle();
}