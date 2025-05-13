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


RobotThread redRingSide() {
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
	liftFlags->pid = PID(1000, 0, 0);
	liftFlags->state = Lift::HOLD;
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(350);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	// lift->setState(Lift::STOW);
	// uint32_t start = pros::millis();
	co_yield drive->waitUntilSettled(2000);
	// co_yield util::coroutine::delay(150 - (pros::millis() - start));

	printf("Mogo 1\n");
	Pose mogo1(-35.7, 0);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield util::coroutine::delay(400);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(800);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(100);

	printf("Center rings\n");
	Pose centerRings(-54.6, -4);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(centerRings).degrees() + 4, PID(550, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerRings.translation()), 60, 100, 60));
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

	printf("Alliance ring\n");
	Pose allianceRing(1.8, 6.4);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 80, 65));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceRing).degrees(), PID(530, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(700);
	liftFlags->targetAngle = 175;

	intake->moveVoltage(12000);
	intake->setExtender(true);
	lift->setState(Lift::HOLD);
	liftFlags->targetAngle = 100;


	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceRing.translation()), 60, 95, 70));
	co_yield drive->waitUntilSettled(2000);
	intake->setExtender(false);


	Pose ladder(-19.4, 15.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(750, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()) + 3, 50, 100, 65));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	liftFlags->targetAngle = 180;
	co_yield drive->waitUntilSettled(1500);


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}