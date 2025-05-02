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
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"


RobotThread redRingSideTest() {
	robotInstance->curAlliance = Alliance::RED;
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

	drive->setCurrentMotion(ProfiledMotion(3.2, 60, 100, 60));
	co_yield util::coroutine::delay(200);
	// score on alliance stake w/ preload
	liftFlags->targetAngle = 225;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	uint32_t start = pros::millis();
	co_yield drive->waitUntilSettled(2000);
	co_yield util::coroutine::delay(150 - (pros::millis() - start));

	printf("Mogo 1\n");
	Pose mogo1(-35.7, 0);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(100);

	printf("Center rings\n");
	Pose centerRings(-53.7, -5);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRings).degrees() + 4, PID(530, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRings.translation())+1, 60, 100, 60));
	pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(2500);

	printf("Setup ring intake\n");
	Pose setupRingIntake(-32.4, 5.4);

	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(setupRingIntake).degrees() - 170, PID(810, 1, 6500), false, false, 0.5,
	//                                 12000, false, false));
	// co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(setupRingIntake.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	printf("Turn to drag ring\n");
	drive->setCurrentMotion(PIDTurn(218, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(250);
	pnoomatics->setLeftHammer(false);
	co_yield util::coroutine::delay(100);

	printf("Ring intake\n");
	Pose ringIntake(-46.0, -15.6);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ringIntake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(300);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation()), 25, 60, 70));
	co_yield drive->waitUntilSettled(2000);

	printf("Corner\n");
	Pose corner(-23.8, -52.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(700);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 10, 60, 100, 65));
	co_yield drive->waitUntilSettled(1600);

	// drive->setCurrentMotion(TimedMotion(400, 12000));
	// co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(12, 40, 60, 85));
	co_yield drive->waitUntilSettled(900);

	printf("alliance ring\n");
	Pose allianceRing(4.8, 3.4);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 80, 65));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceRing).degrees(), PID(530, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(700);

	intake->moveVoltage(12000);
	intake->setExtender(true);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceRing.translation()), 60, 95, 70));
	co_yield drive->waitUntilSettled(2000);
	intake->setExtender(false);


	Pose ladder(-14.4, 16.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 50, 100, 65));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	co_yield util::coroutine::delay(500);
	liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(1500);


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}

// TODO: change all the headings
RobotThread redRingSide() {
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

	drive->setCurrentMotion(ProfiledMotion(3, 60, 60, 60));
	/*liftFlags->targetAngle = 50;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(800);// TUNE THIS
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };*/

	Pose curPose = odom->getCurrentState().position;
	Pose mogo1(-43.2, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo1.translation()), 80, 120, 35));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(50);

	Pose centerRings(-50.2, 11.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRings).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRings.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);
	pnoomatics->setLeftHammer(true);

	// move forwards 3 inches after
	drive->setCurrentMotion(ProfiledMotion(3, 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	Pose temp1(-43.9, 6.3);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp1).degrees() + 180, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(temp1.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	// turn to heading 113
	drive->setCurrentMotion(PIDTurn(113, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	// lift hammer
	pnoomatics->setLeftHammer(false);

	Pose temp2(-59.6, 21.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp2).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(temp2.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);


	Pose temp3(-47.7, 9.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp3).degrees() + 180, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(temp3.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);


	Pose temp4(-41.5, 27.3);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp4).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(temp4.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);


	Pose temp5(-13.6, 52.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp5).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(temp5.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);


	// move back 24in
	drive->setCurrentMotion(ProfiledMotion(-24, 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	// move fwd 12in
	drive->setCurrentMotion(ProfiledMotion(12, 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	drive->setCurrentMotion(ProfiledMotion(-30, 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);

	Pose temp6(-43.8, -2.6);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(temp6).degrees(), PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(temp6.translation()), 60, 100, 60));
	co_yield drive->waitUntilSettled(2500);
}