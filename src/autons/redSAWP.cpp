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

RobotThread redSAWP() {
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

	drive->setCurrentMotion(ProfiledMotion(3.5, 60, 60, 60));
	co_yield util::coroutine::delay(200);
	// score on alliance stake w/ preload
	liftFlags->targetAngle = 220;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	lift->setState(Lift::STOW);
	uint32_t start = pros::millis();
	co_yield drive->waitUntilSettled(2000);
	co_yield util::coroutine::delay(100 - (pros::millis() - start));


	printf("Mogo 1\n");
	Pose mogo1(-35.7,0);
	// move back first to get mogo
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 120, 45));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	printf("M1 ring1\n");
	Pose ring1(-56.0, -7.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ring1).degrees(), PID(330, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(12000);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	printf("Intake rings\n");
	Pose intakeRings(-61.9, -14.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(intakeRings).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intakeRings.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	printf("Pre Ring 3\n");
	Pose preRing3(-45.4, -4.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(preRing3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(preRing3.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	printf("Ring 3\n");
	Pose ring3(-44.4, -19.95);
	// pre ring stack

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ring3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);


	printf("Move back\n");
	Pose moveBack(14.6, 46.9);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(moveBack).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);
	pnoomatics->setClamp(false);

	printf("M2 ring1\n");
	Pose m2Ring1(-0.7, 62);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m2Ring1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(550);

	intake->setDistStop(true);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m2Ring1.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	printf("Mogo 2\n");
	Pose mogo2(-12.0, 40.7);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 60, 100, 40));
	curPose = odom->getCurrentState().position;
	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	co_yield drive->waitUntilSettled(2000);
	intake->setDistStop(false);
	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	printf("Ladder\n");
	Pose ladder(-39.1, 33.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(450, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 60, 100, 85));
	// co_yield drive->waitUntilSettled(2000);*/

	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);

	co_yield util::coroutine::nextCycle();
}