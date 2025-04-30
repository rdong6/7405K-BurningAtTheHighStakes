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

RobotThread redMogoSideFake() {
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

    printf("Ring 1\n");
	Pose ring1(0,0);

	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 60, 100, 35));
	co_yield util::coroutine::delay(800);
    intake->setDistStop(true);
    intake->moveVoltage(12000);
    co_yield drive->waitUntilSettled(1500);

    printf("Mogo 1\n");
	Pose mogo1(0,0);

    curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo1).degrees(), PID(554, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);
    intake->moveVoltage(12000);

	co_yield util::coroutine::delay(100);

	printf("Ring 2\n");
	Pose ring2(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ring2).degrees(), PID(554, 1, 6800), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 60, 100, 85));
	pnoomatics->setRightHammer(true);
	co_yield drive->waitUntilSettled(1250);

    curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(0, PID(554, 1, 6800), false, true, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
    pnoomatics->setLeftHammer(true);


	printf("Setup ring intake\n");
	Pose setupRingIntake(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180 + curPose.headingTo(setupRingIntake).degrees(), PID(620, 1, 6500), false, false, 0.5,
	                                12000, false, false));
	co_yield drive->waitUntilSettled(400);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(setupRingIntake.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

    pnoomatics->setRightHammer(false);

	drive->setCurrentMotion(PIDTurn(000, PID(620, 1, 6500), true, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

    pnoomatics->setLeftHammer(false);

	printf("Ring intake\n");
	Pose ringIntake(0, 0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ringIntake).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation()), 20, 40, 70));
	co_yield drive->waitUntilSettled(2000);

	printf("Corner\n");
	Pose corner(-20, 53.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

    liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 50, 100, 65));
	// co_yield util::coroutine::delay(500);
	// intake->moveVoltage(0);
	// co_yield util::coroutine::delay(700);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(TimedMotion(400, 12000));
	co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-28, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	pnoomatics->setLeftHammer(true);

	drive->setCurrentMotion(ProfiledMotion(16, 40, 60, 85));
	co_yield drive->waitUntilSettled(900);

	printf("Ladder\n");
	Pose ladder(-41.6, -4.3);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(525, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(1000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 50, 100, 65));
    intake->moveVoltage(-12000);
    co_yield util::coroutine::delay(100);
    intake->moveVoltage(12000);
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	// intake->moveVoltage(0);
	pnoomatics->setLeftHammer(false);
	co_yield util::coroutine::delay(500);
	// pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1500);

	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}