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
	// co_yield util::coroutine::delay(150 - (pros::millis() - start));

	printf("Mogo 1\n");
	Pose mogo1(-35.7, 0);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 35));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(100);

	printf("Ring intake\n");
	Pose ringIntake(-42.0, -18.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ringIntake).degrees(), PID(580, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(520);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation()), 50, 85, 70));
	co_yield drive->waitUntilSettled(2000);

	printf("Corner\n");
	Pose corner(-23.8, -50.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 10, 60, 100, 65));
	co_yield drive->waitUntilSettled(1350);

	// drive->setCurrentMotion(TimedMotion(400, 12000));
	// co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 85));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(12, 50, 80, 85));
	co_yield drive->waitUntilSettled(900);

	printf("Alliance ring\n");
	Pose allianceRing(3, 4.6);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 80, 85));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceRing).degrees(), PID(550, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	pnoomatics->setRightHammer(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceRing.translation())+27, 60, 95, 50));
	co_yield drive->waitUntilSettled(2000);

	printf("m2 ring1\n");
	Pose m2ring1(6.2, 65.4);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m2ring1).degrees(), PID(620, 1, 6300), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	pnoomatics->setRightHammer(false);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m2ring1.translation()), 60, 95, 50));
	co_yield util::coroutine::delay(600);
	pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1800);

	printf("Mogo 2\n");
	Pose mogo2(-6.3, 45.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo2.translation()) - 3, 60, 95, 70));
	co_yield drive->waitUntilSettled(2000);
	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);	

	printf("ladder\n");
	Pose ladder(-10.1, 45.8);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(540, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()) + 3, 60, 95, 50));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(1500);


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}