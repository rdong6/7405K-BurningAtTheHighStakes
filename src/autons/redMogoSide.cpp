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

RobotThread redMogoSide() {
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

	// co_yield util::coroutine::delay(900);
	
	uint32_t startTime = pros::millis();

	drive->setCurrentMotion(ProfiledMotion(2.5, 60, 100, 60));
	co_yield util::coroutine::delay(200);
	// score on alliance stake w/ preload
	liftFlags->targetAngle = 225;
	liftFlags->pid = PID(1000, 0, 0);
	liftFlags->state = Lift::HOLD;
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	// lift->setState(Lift::STOW);
	// uint32_t start = pros::millis();
	co_yield drive->waitUntilSettled(2000);
	// co_yield util::coroutine::delay(150 - (pros::millis() - start));
	// move back first to get mogo

	printf("Mogo 1\n");
	Pose mogo1(-35.7,0);

	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield util::coroutine::delay(400);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	co_yield util::coroutine::delay(100);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.rotation().degrees() + 5, PID(800, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(150);

	printf("Ring 1\n");
	Pose ring1(-37.5, -17);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ring1).degrees(), PID(554, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation())+0.5, 60, 100, 85));
	pnoomatics->setRightHammer(true);
	co_yield drive->waitUntilSettled(1250);

	printf("Setup ring intake\n");
	Pose setupRingIntake(-25.9, 1);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(setupRingIntake).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(setupRingIntake.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(PIDTurn(158, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	pnoomatics->setRightHammer(false);
	co_yield util::coroutine::delay(100);

	printf("Ring intake\n");
	Pose ringIntake(-42.5, 24.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ringIntake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(300);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation())+3, 45, 40, 70));
	co_yield drive->waitUntilSettled(2000);

	printf("Corner\n");
	Pose corner(-20, 56.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 15, 60, 100, 45));
	co_yield drive->waitUntilSettled(2000);

	// drive->setCurrentMotion(TimedMotion(400, 12000));
	// co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	pnoomatics->setLeftHammer(true);

	drive->setCurrentMotion(ProfiledMotion(12, 40, 50, 50));
	co_yield drive->waitUntilSettled(900);

	printf("Ladder\n");
	Pose ladder(-41.6, -4.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees() + 20, PID(810, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(1000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 50, 100, 65));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	// intake->moveVoltage(0);
	pnoomatics->setLeftHammer(false);
	co_yield util::coroutine::delay(500);
	liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);
	// pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1500);

	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}