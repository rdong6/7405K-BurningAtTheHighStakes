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


	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	// TODO: TESTING
	pros::delay(250);
	drive->setCurrentMotion(ProfiledMotion(-7, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);



	Pose firstMogo(-7.6, 19.3);
	Pose curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(firstMogo).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(firstMogo.translation()), 70, 120, 30));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	Pose firstRing(-32.0, 21.3);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(firstRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(firstRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose secondRing(-76.3, 46.0);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(secondRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose thirdRing(-98.0, 64.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(thirdRing.translation()) + 3, 70, 120, 40));
	co_yield drive->waitUntilSettled(5000);
	// NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	lift->setState(Lift::LEVEL_1);


	Pose lineUpWallStake1(-58.5, 44.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake1).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake1.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	intakeFlags->ladyBrownClearanceEnabled = true;
	liftFlags->slewEnabled = true;
	// liftFlags->slewRate = 250;
	lift->setState(Lift::HOLD);

	Pose wallStake1(-51.2, 55.7);

	curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(
	//         PIDTurn(curPose.headingTo(wallStake1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	drive->setCurrentMotion(PIDTurn(90, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake1.translation()) + 10.5, 70, 120, 60));
	co_yield drive->waitUntilSettled(2000);


	// score first ring
	liftFlags->targetAngle = 190;
	liftFlags->slewRate = 1000;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(800);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;

	// move lift down and load second ring into intake
	lift->setState(Lift::LEVEL_1);

	intake->setDistStop(false);
	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);
	printf("Running intake to load second ring\n");

	Timeout timeout = Timeout(1250);
	co_yield [=]() -> bool { return intakeFlags->ladyBrownClearanceEnabled || timeout.timedOut(); };
	// delays 110 ms because lady brown stall detection coro enables lady brown clearance coro through the flag. during this
	// time, intake has complete control and anything we do to intake will be ignored until lady brown clearance code is done
	// running. that takes about 100-110 ms. pretty sure its 100ms if i counted correctly
	co_yield util::coroutine::delay(200);


	// score second run
	liftFlags->targetAngle = 190;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(800);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;
	lift->setState(Lift::STOW);

	// move back after scoring on wallstake to align with the row of rings
	Pose moveBack1(-55.6, 50.05);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(moveBack1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack1.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(12000);

	// original x: 2.6
	Pose rowOfRings(6.92, 54.0);

	curPose = odom->getCurrentState().position;
	// bumped up D from 6500
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rowOfRings).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings.translation()), 35, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	// original x: -6.7 y: 58.4
	Pose alignCornerPoint(-9.6, 63.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(alignCornerPoint).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignCornerPoint.translation()) + 3, 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	// original: 60.3
	Pose cornerPoint(0, 61);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(cornerPoint).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(cornerPoint.translation()), 40, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);
	co_yield util::coroutine::delay(100);

	Pose alignMogo2(-9.2, 56.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(alignMogo2).degrees(), PID(600, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignMogo2.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	intake->moveVoltage(0);

	printf("Moving to mogo2\n\n");
	Pose mogo2Midpoint(-7.8, -11);
	Pose mogo2(-7.8, -20.7);


	// do a 2 part movement to mogo
	co_yield util::coroutine::delay(250);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(mogo2Midpoint).degrees(), PID(615, 1, 6700), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2Midpoint.translation()), 70, 100, 60));
	co_yield drive->waitUntilSettled(2000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 40, 100, 35));
	co_yield drive->waitUntilSettled(5000);
	pnoomatics->setClamp(true);

	Pose fourthRing(-25.7, -21.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(fourthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fourthRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);


	Pose fifthRing(-73.4, -49.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(fifthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fifthRing.translation()), 70, 120, 40));

	Pose sixthRing(-98.8, -58.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(sixthRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(sixthRing.translation()) + 3, 70, 120, 40));
	co_yield drive->waitUntilSettled(5000);
	// NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	lift->setState(Lift::LEVEL_1);

	// // wait until we intake first ring, wait for it to leave intake, then set lady brown down to load the 2nd ring
	// timeout = Timeout(2500);
	// co_yield
	//         [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() >= 1 || timeout.timedOut();
	//         };
	//
	// timeout = Timeout(500);
	// co_yield
	//         [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() == 0 || timeout.timedOut();
	//         };
	//
	// timeout = Timeout(1750);
	// co_yield
	//         [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() >= 1 || timeout.timedOut();
	//         };
	//
	// printf("First ring intaked. Setting dist stop to true\n");
	//
	// intake->setDistStop(true);
	//
	// timeout = Timeout(500);
	// co_yield
	//         [=]() -> bool { return robotInstance->getSubsystem<Intake>().value()->ringsInIntake() == 0 || timeout.timedOut();
	//         };
	// printf("First ring left intake, setting lady brown to level 1\n");
	//
	// lift->setState(Lift::LEVEL_1);
	// intake->setDistStop(false);
	// // TODO: MAKE SURE THIS STILL HAPPENS WITHIN MOTION, OTHERWISE START MOVING BACKWARDS AND WAIT FOR LADYBROWN UP
	// timeout = Timeout(600);
	// co_yield [=]() { return timeout.timedOut() || !robotInstance->getFlag<Lift>().value()->isMoving; };
	// intake->moveVoltage(12000);
	//
	// co_yield drive->waitUntilSettled(5000);


	Pose lineUpWallStake2(-51.4, -47.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake2).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake2.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose wallStake2(-50.8, -66.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(270, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	intakeFlags->ladyBrownClearanceEnabled = true;
	co_yield drive->waitUntilSettled(500);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake2.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(2000);

	liftFlags->targetAngle = 190;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;

	lift->setState(Lift::LEVEL_1);

	intake->setDistStop(false);
	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);

	co_yield [&]() -> bool { return intakeFlags->ladyBrownClearanceEnabled; };
	co_yield util::coroutine::delay(200);

	liftFlags->targetAngle = 190;
	liftFlags->slewEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	liftFlags->slewEnabled = false;
	lift->setState(Lift::STOW);

	Pose backWallStake(-51.0, -52.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(backWallStake).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(backWallStake.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose rowOfRings2(9.6, -49.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(rowOfRings2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings2.translation()), 30, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose setUpCorner2(-9.4, -61.1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(setUpCorner2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpCorner2.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose corner2(2, -61.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(corner2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner2.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);


	Pose setUpRingSeventhRing(-58.2, -42.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(setUpRingSeventhRing).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpRingSeventhRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose seventhRing(-78.7, -25.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(seventhRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	lift->setState(Lift::LEVEL_1);
	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(seventhRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose mogo3(-107.2, -1.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo3.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);

	Pose allianceStake(-124, -6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceStake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceStake.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	// TUNE THIS DISTANCE
	drive->setCurrentMotion(ProfiledMotion(-8, 70, 120, 60));
	co_yield drive->waitUntilSettled(1500);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	pros::delay(250);
	drive->setCurrentMotion(ProfiledMotion(-6, 50, 70, 45));
	co_yield drive->waitUntilSettled(1500);

	Pose m3RingOne(-106.4, 45.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingOne).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingOne.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingTwo(-117.3, 45.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingTwo).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingTwo.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	drive->setCurrentMotion(ProfiledMotion(-7, 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose preCenterRing(-81.7, 24.0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preCenterRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCenterRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	/*Pose centerRing(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRing).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRing.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingThree(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingThree).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingThree.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingFour(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m3RingFour).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingFour.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose corner3Dump(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(corner3Dump).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner3Dump.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);

	Pose preMogo4(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preMogo4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preMogo4.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	Pose mogo4(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo4.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);

	Pose corner4(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner4).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	pnoomatics->setRightHammer(true);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner4.translation()), 70, 120, 60));
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
	drive->setCurrentMotion(ProfiledMotion(-10, 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preHang).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preHang.translation()), 70, 120, 60));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 180;
	lift->setState(Lift::HOLD);


	Pose Hang(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(Hang).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(Hang.translation()), 70, 120, 30));
	co_yield drive->waitUntilSettled(5000);
	*/

	co_yield util::coroutine::nextCycle();
}