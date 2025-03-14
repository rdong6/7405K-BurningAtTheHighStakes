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

	uint32_t startTime = pros::millis();

	printf("Rush rings\n");
	Pose rush(47.5, 0);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rush.translation()), 60, 120, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
	co_yield util::coroutine::delay(650);
	pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(2000);

	printf("Mogo 1\n");
	Pose mogo1(27.1, -13.1);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(180 + curPose.headingTo(mogo1).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 40));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(true);
	intake->setDistStop(false);


	printf("Intake 3 rings\n");
	drive->setCurrentMotion(PIDTurn(54.8, PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	pnoomatics->setLeftHammer(false);
	co_yield util::coroutine::delay(125);
	Pose intakeR1R2R3(36.0, 12.4);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(intakeR1R2R3).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intakeR1R2R3.translation()), 40, 60, 50));
	co_yield drive->waitUntilSettled(2000);


	// co_yield util::coroutine::delay(300);

	printf("Corner point\n");
	Pose corner(0.0, 42.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(corner).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 50, 100, 65));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(0);
	co_yield util::coroutine::delay(700);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(950);

	drive->setCurrentMotion(TimedMotion(400, 12000));
	co_yield drive->waitUntilSettled(400);

	drive->setCurrentMotion(ProfiledMotion(-36, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(24, 40, 100, 85));
	co_yield drive->waitUntilSettled(900);

	intake->moveVoltage(12000);

	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 65));
	co_yield drive->waitUntilSettled(900);

	printf("Preload\n");
	Pose preload(0, -12.0);

	// quals
	//     TODO: Tune this PID
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preload).degrees(), PID(450, 1, 6700), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(800);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preload.translation()) + 5, 60, 100, 85));
	co_yield util::coroutine::delay(500);
	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(2000);


	Pose ladder(14.8, -32.5);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(ladder).degrees(), PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 60, 100, 85));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	co_yield drive->waitUntilSettled(2000);

	printf("Took %f seconds\n", (pros::millis() - startTime) / 1000.0);
	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);
}