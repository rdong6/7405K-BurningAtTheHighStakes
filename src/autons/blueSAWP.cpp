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

RobotThread blueSAWP() {
	robotInstance->curAlliance = Alliance::BLUE;
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

	drive->setCurrentMotion(ProfiledMotion(2.7, 60, 100, 60));
	co_yield util::coroutine::delay(200);
	// score on alliance stake w/ preload
	liftFlags->targetAngle = 225;
	liftFlags->pid = PID(1000, 0, 0);
	liftFlags->state = Lift::HOLD;
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
	Pose ringIntake(-42.0, 18.9);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ringIntake).degrees(), PID(580, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(520);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation()), 50, 85, 70));
	co_yield drive->waitUntilSettled(2000);

	printf("Corner\n");
	Pose corner(-23.8, 50.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 15, 60, 110, 65));
	co_yield drive->waitUntilSettled(1400);

	// drive->setCurrentMotion(TimedMotion(400, 12000));
	// co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-22, 50, 80, 85));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(12, 50, 80, 85));
	co_yield drive->waitUntilSettled(900);

	printf("Alliance ring\n");
	Pose allianceRing(9, -6);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 80, 85));
	co_yield drive->waitUntilSettled(500);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(allianceRing).degrees(), PID(550, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	pnoomatics->setLeftHammer(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceRing.translation()) + 27, 60, 95, 50));
	co_yield drive->waitUntilSettled(2000);

	printf("m2 ring1\n");
	Pose m2ring1(-4, -58);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(m2ring1).degrees(), PID(620, 1, 6300), false, false, 0.5, 12000, false, false));
	co_yield util::coroutine::delay(50);
	pnoomatics->setLeftHammer(false);
	co_yield drive->waitUntilSettled(450);// total timeout is 500ms

	curPose = odom->getCurrentState().position;
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m2ring1.translation()), 60, 95, 50));
	co_yield util::coroutine::delay(600);
	pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1800);

	printf("Mogo 2\n");
	Pose mogo2(-14, -41.2);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo2).degrees(), PID(630, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-1 * curPose.translation().distanceTo(mogo2.translation()) - 3, 60, 95, 70));
	co_yield drive->waitUntilSettled(2000);
	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);
	co_yield util::coroutine::delay(100);

	printf("ladder\n");
	Pose ladder(-20.3, -35.7);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(540, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()) + 5, 60, 95, 50));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	liftFlags->targetAngle = 175;
	lift->setState(Lift::IDLE);
	liftFlags->state = Lift::HOLD;
	co_yield drive->waitUntilSettled(1500);


	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
}

/*RobotThread blueSAWP() {
    robotInstance->curAlliance = Alliance::BLUE;

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
    Pose ring1(-53.0, 7.4);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(curPose.headingTo(ring1).degrees(), PID(330, 1, 6700), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(800);
    intake->moveVoltage(12000);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);

    printf("Intake rings\n");
    Pose intakeRings(-61.9, 14.7);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(curPose.headingTo(intakeRings).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(550);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intakeRings.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);

    printf("Pre Ring 3\n");
    Pose preRing3(-45.4, 4.1);
    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(180 + curPose.headingTo(preRing3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(500);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(preRing3.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);

    printf("Ring 3\n");
    Pose ring3(-44.4, 19.95);
    // pre ring stack

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(curPose.headingTo(ring3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(550);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);


    printf("Move back\n");
    Pose moveBack(14.6, -46.9);
    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(180 + curPose.headingTo(moveBack).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(550);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(moveBack.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);
    pnoomatics->setClamp(false);

    printf("M2 ring1\n");
    Pose m2Ring1(-2.7, -62);
    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(curPose.headingTo(m2Ring1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(550);

    intake->setDistStop(true);
    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m2Ring1.translation()), 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);

    printf("Mogo 2\n");
    Pose mogo2(-12.0, -40.7);
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
    Pose ladder(-39.1, -33.7);

    curPose = odom->getCurrentState().position;
    drive->setCurrentMotion(
            PIDTurn(curPose.headingTo(ladder).degrees(), PID(450, 1, 6500), false, false, 0.5, 12000, false, false));
    co_yield drive->waitUntilSettled(600);

    liftFlags->targetAngle = 170;
    lift->setState(Lift::HOLD);
    drive->setCurrentMotion(ProfiledMotion(4, 60, 100, 85));
    co_yield drive->waitUntilSettled(2000);

    printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);

    co_yield util::coroutine::nextCycle();
}*/