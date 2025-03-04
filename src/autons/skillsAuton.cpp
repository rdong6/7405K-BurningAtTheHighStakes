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
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::STOW);


	Pose firstMogo(-7.6, 19.3);
	Pose curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(firstMogo).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(firstMogo.translation()), 60, 120, 80));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	Pose firstRing(-28.2, 23.6);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(firstRing).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(firstRing.translation()), 60, 120, 80));
	co_yield drive->waitUntilSettled(5000);

	Pose secondRing(-79.0, 47.4);
	curPose = odom->getCurrentState().position;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(secondRing).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondRing.translation()), 60, 120, 100));
	co_yield drive->waitUntilSettled(5000);

	Pose thirdRing(-102.6, 59.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(thirdRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	//NEEDS TUNING FOR DELAY AND WHEN WE SET UP LB
	lift->setState(Lift::LEVEL_1);


	Pose lineUpWallStake1(-58.7, 46.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake1).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose wallStake1(-57.2, 63.1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(wallStake1).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::LEVEL_1);

	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);

	co_yield [&]() -> bool { return intakeFlags->ladyBrownClearanceEnabled; };
	// delays 110 ms because lady brown stall detection coro enables lady brown clearance coro through the flag. during this
	// time, intake has complete control and anything we do to intake will be ignored until lady brown clearance code is done
	// running. that takes about 100-110 ms. pretty sure its 100ms if i counted correctly
	co_yield util::coroutine::delay(110);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::STOW);

	// move back after scoring on wallstake to align with the row of rings
	Pose moveBack1(-57.0, 48.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(moveBack1).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack1.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(12000);

	Pose rowOfRings(2.5, 50.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(rowOfRings).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings.translation()), 30, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose alignCornerPoint(-7.6, 60.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(alignCornerPoint).degrees(), PID(100, 1, 485, true, 10), false, false,
	                                0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignCornerPoint.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose cornerPoint(-2.0, 63.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(cornerPoint).degrees(), PID(100, 1, 485, true, 10), false, false,
	                                0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(cornerPoint.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);
	co_yield util::coroutine::delay(100);

	Pose alignMogo2(-9.1, 59.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alignMogo2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	intake->moveVoltage(0);

	Pose mogo2(-3.6, -17.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 60, 120, 60));
	co_yield drive->waitUntilSettled(5000);
	pnoomatics->setClamp(true);

	Pose fourthRing(-24, -23.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(fourthRing).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fourthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);


	Pose fifthRing(-70.8, -40.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(fifthRing).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(fifthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose sixthRing(-94.8, -62.0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(sixthRing).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(sixthRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	lift->setState(Lift::LEVEL_1);

	Pose lineUpWallStake2(-52.1, -45.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(lineUpWallStake2).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(lineUpWallStake2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose wallStake2(-49.6, -62.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(wallStake2).degrees(), PID(100, 1, 485, true, 10), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->setDistStop(true);
	intake->moveVoltage(10000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(wallStake2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::LEVEL_1);

	co_yield util::coroutine::delay(200);
	intake->moveVoltage(12000);

	co_yield [&]() -> bool { return intakeFlags->ladyBrownClearanceEnabled; };
	co_yield util::coroutine::delay(110);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	liftTimeout = Timeout(400);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	lift->setState(Lift::STOW);

	Pose backWallStake(-46.9, -50.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(backWallStake).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(backWallStake.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose rowOfRings2(0,0);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(rowOfRing2).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rowOfRings2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose setUpCorner2(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(setUpCorner2).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpCorner2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	
	Pose corner2(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(corner2).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner2.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);


	

	Pose setUpRingSeventhRing(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(setUpRingSeventhRing).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setUpRingSeventhRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose seventhRing(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(seventhRing).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	lift->setState(Lift::LEVEL_1);
	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(seventhRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose preASMogoPush(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preASMogoPush).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preASMogoPush.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose ASMogoPush(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(ASMogoPush).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(ASMogoPush.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose allianceStake(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(allianceStake).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceStake.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
//TUNE THIS DISTANCE
	drive->setCurrentMotion(ProfiledMotion(-8, 60, 120, 85));

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(600);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);

	Pose mogo3(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(mogo3).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo3.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);

	Pose m3RingOne(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(m3RingOne).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingOne.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);
	
	Pose m3RingTwo(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(m3RingTwo).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingTwo.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	drive->setCurrentMotion(ProfiledMotion(-7, 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose preCenterRing(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preCenterRing).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCenterRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose centerRing(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(centerRing).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRing.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingThree(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(m3RingThree).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingThree.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose m3RingFour(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(m3RingFour).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m3RingFour.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose corner3Dump(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(corner3Dump).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(corner3Dump.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	intake->moveVoltage(-12000);
	pnoomatics->setClamp(false);

	co_yield util::coroutine::delay(100);

	Pose preMogo4(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preMogo4).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);
	
	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preMogo4.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	Pose mogo4(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(mogo4).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo4.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	pnoomatics->setClamp(true);

	Pose corner4(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner4).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	pnoomatics->setRightHammer(true);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner4.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	//turn to dump

	Pose preHang(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preHang).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	pnoomatics->setRightHammer(false);
	pnoomatics->setClamp(false);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-10, 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preHang).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preHang.translation()), 60, 120, 85));
	co_yield drive->waitUntilSettled(5000);

	liftFlags->targetAngle = 180;
	lift->setState(Lift::HOLD);


	Pose Hang(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(Hang).degrees(), PID(100, 1, 485, true, 10), false,
	                                false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(Hang.translation()), 60, 120, 30));
	co_yield drive->waitUntilSettled(5000);





}