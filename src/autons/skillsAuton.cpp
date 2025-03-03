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


	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::STOW);


	Pose firstMogo(-8.3, 19.6);
	Pose curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(firstMogo).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(firstMogo.translation()), 60, 120, 80));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	Pose firstRing(-27.1, 23.9);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(firstRing).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(firstRing.translation()), 60, 120, 80));
	co_yield drive->waitUntilSettled(5000);

	Pose secondRing(-77.2, 45.2);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(secondRing).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondRing.translation()), 60, 120, 100));
	co_yield drive->waitUntilSettled(5000);

	Pose thirdRing(-100.2, 56.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(thirdRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	//NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	lift->setState(Lift::LEVEL_1);


	Pose lineUpWallStake1(-56.9, 40.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake1).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose wallStake1(-56.9, 62.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(wallStake1).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::LEVEL_1);

	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);

	co_yield [&]() -> bool { return intakeFlags->ladyBrownClearanceEnabled; };
	co_yield util::coroutine::delay(110);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::STOW);

	// move back after scoring on wallstake to align with the row of rings
	Pose moveBack1(-56.5, 48.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(moveBack1).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(12000);

	Pose rowOfRings(2.6, 50.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(rowOfRings).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings.translation()), 30, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose alignCornerPoint(-7.5, 60.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(alignCornerPoint).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignCornerPoint.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose cornerPoint(-3.4, 62.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(cornerPoint).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(cornerPoint.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	
	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);
	co_yield util::coroutine::delay(100);

	Pose alignMogo2(-10.9, 58.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignMogo2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	intake->moveVoltage(0);

	Pose mogo2(-3.2, -18.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 60, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	pnoomatics->setClamp(true);

	Pose fourthRing(-23.3, -24.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(fourthRing).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fourthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	

	Pose fifthRing(-69.9, -53.1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(fifthRing).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fifthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose sixthRing(-92.0, -67.0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(sixthRing).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(sixthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	
	lift->setState(Lift::LEVEL_1);

	Pose lineUpWallStake2(-48.6, -48.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake2).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose wallStake2(-45.6, -64.0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(wallStake2).degrees(), PID(), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::LEVEL_1);

	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);

	co_yield [&]() -> bool { return intakeFlags->ladyBrownClearanceEnabled; };
	co_yield util::coroutine::delay(110);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::STOW);

	Pose backWallStake(-46.9, -50.7);


	/*drive->setCurrentMotion(PIDTurn(265, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-18.5, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);

	intake->moveVoltage(12000);

	drive->setCurrentMotion(PIDTurn(172, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(650);
	drive->setCurrentMotion(ProfiledMotion(18, 50, 80, 80));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(157, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);
	drive->setCurrentMotion(ProfiledMotion(60, 50, 80, 60));
	co_yield util::coroutine::delay(750);
	lift->setState(Lift::LEVEL_1);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(1250);
	drive->setCurrentMotion(PIDTurn(167, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);


	drive->setCurrentMotion(ProfiledMotion(-24.5, 50, 80, 60));
	co_yield util::coroutine::delay(750);
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 65;
	intakeFlags->ladyBrownClearanceEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(17.5, 30, 60, 30));
	co_yield util::coroutine::delay(250);// delay needed as ladyBrownClearanceEnabled stops intake after 60ms
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(2000);

	liftFlags->targetAngle = 150;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	drive->setCurrentMotion(ProfiledMotion(-13, 30, 70, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(2.5, PID(750, 0, 5500, true, 10), false, false, 0.5));
	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(700);

	drive->setCurrentMotion(ProfiledMotion(62, 25, 40, 30));
	co_yield drive->waitUntilSettled(7000);
	co_yield util::coroutine::delay(500);

	drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(15, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(ProfiledMotion(-9.5, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(225, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(-8, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(false);
	intake->moveVoltage(-12000);
	// first corner is done

	drive->setCurrentMotion(ProfiledMotion(12, 50, 60, 30));
	co_yield util::coroutine::delay(250);
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90.5, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

// original: -71 in
	drive->setCurrentMotion(ProfiledMotion(-65, 50, 80, 50));
	co_yield drive->waitUntilSettled(5000);

	// grab 2nd mogo
	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);


	drive->setCurrentMotion(PIDTurn(189, PID(750, 0, 5500, true, 10), false, false, 0.5)); // original
	co_yield drive->waitUntilSettled(700);
	drive->setCurrentMotion(ProfiledMotion(18, 50, 80, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(205, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(60, 50, 80, 60));
	co_yield util::coroutine::delay(750);
	lift->setState(Lift::LEVEL_1);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(1250);
	drive->setCurrentMotion(PIDTurn(191, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);


// oriignal: 23
	drive->setCurrentMotion(ProfiledMotion(-27, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(270, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);

	liftFlags->targetAngle = 65;
	intakeFlags->ladyBrownClearanceEnabled = true;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(ProfiledMotion(20, 30, 40, 30));

	co_yield util::coroutine::delay(250);// delay needed as ladyBrownClearanceEnabled stops intake after 60ms
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(2000);

	liftFlags->targetAngle = 150;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	drive->setCurrentMotion(ProfiledMotion(-13, 30, 40, 30));
	co_yield drive->waitUntilSettled(2000);

// original: 1deg
	drive->setCurrentMotion(PIDTurn(0, PID(750, 0, 5500, true, 10), false, false, 0.5));
	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();
	co_yield drive->waitUntilSettled(700);
	intake->moveVoltage(12000);

	drive->setCurrentMotion(ProfiledMotion(62, 25, 40, 30));
	co_yield drive->waitUntilSettled(7000);

	drive->setCurrentMotion(PIDTurn(225, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);

	drive->setCurrentMotion(ProfiledMotion(15, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(ProfiledMotion(-6, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	// original: 8
	drive->setCurrentMotion(ProfiledMotion(-9.5, 50, 80, 60));
	co_yield drive->waitUntilSettled(2000);

	// drop 2nd mogo into corner
	pnoomatics->setClamp(false);
	intake->moveVoltage(-12000);


	// original: 8
	// drive->setCurrentMotion(ProfiledMotion(9.5, 25, 40, 30));
	// co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);
	intake->moveVoltage(0);
	drive->setCurrentMotion(ProfiledMotion(73, 50, 60, 60));
	co_yield util::coroutine::delay(1800);
	intake->setDistStop(true);
	intake->moveVoltage(10000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(ProfiledMotion(40, 50, 60, 60));
	co_yield util::coroutine::delay(700);
	intake->setDistStop(true);
	intake->moveVoltage(9000);
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(45, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);
	drive->setCurrentMotion(ProfiledMotion(-34, 50, 60, 30));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);
	intake->setDistStop(false);

	drive->setCurrentMotion(PIDTurn(92, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(40, 50, 80, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(ProfiledMotion(-2, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(90, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(21, 25, 60, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(215, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(12, 25, 40, 30));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(310, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-25, 50, 100, 60));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1000);
	pnoomatics->setClamp(false);

	drive->setCurrentMotion(PIDTurn(310, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(30, 50, 100, 100));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(70, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(TimedMotion(10000,-12000));
	co_yield drive->waitUntilSettled(10000);*/


	// lift->setState(Lift::LEVEL_1);
	// co_yield util::coroutine::nextCycle();

	// drive->setCurrentMotion(ProfiledMotion(-25, 50, 60, 60));
	// co_yield drive->waitUntilSettled(2000);

	// drive->setCurrentMotion(PIDTurn(270, PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// liftFlags->targetAngle = 50;
	// lift->setState(Lift::HOLD);
	// co_yield util::coroutine::nextCycle();

	// drive->setCurrentMotion(ProfiledMotion(25, 30, 40, 30));
	// co_yield drive->waitUntilSettled(2000);

	// liftFlags->targetAngle = 150;
	// lift->setState(Lift::HOLD);
	// co_yield util::coroutine::nextCycle();
	// liftTimeout = Timeout(700);
	// co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};

	// drive->setCurrentMotion(ProfiledMotion(-10, 30, 40, 30));
	// co_yield drive->waitUntilSettled(2000);

	// drive->setCurrentMotion(PIDTurn(0, PID(750, 0, 5500, true, 10), false, false, 0.5));
	// lift->setState(Lift::STOW);
	// co_yield util::coroutine::nextCycle();
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(55, 25, 40, 30));
	// co_yield drive->waitUntilSettled(2000);

	// drive->setCurrentMotion(PIDTurn(225, PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(15, 25, 40, 30));
	// co_yield drive->waitUntilSettled(2000);

	// drive->setCurrentMotion(PIDTurn(135, PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(-5, 25, 40, 30));
	// co_yield drive->waitUntilSettled(2000);
}