#include "main.h"
#include "AutonSelector.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/DriveCharacterizationMotion.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/motion/TimedMotion.h"
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
#include <type_traits>


#include "Logger.h"
#include "lib/geometry/Rotation2D.h"
#include "lib/geometry/Transform2D.h"
#include "lib/geometry/Translation2D.h"
#include "lib/geometry/Twist2D.h"

RobotThread autonomousUser();

void robot_init() {
	robotInstance = new std::decay<decltype(*robotInstance)>::type();
	robotInstance->registerTask([]() { return autonomousUser(); }, TaskType::AUTON);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	AutonSelector autonSelector;// also initalizes pros::lcd
	robot_init();
	// autonSelector.run();


	pros::lcd::print(6, "Alliance: %d", robotInstance->curAlliance);
	pros::lcd::print(7, "Auton: %d", robotInstance->curAuton);

	// TODO: Fix logger so no unaligned accesses
	// logger_initialize("test.txt", 100);

	robotTask = pros::c::task_create(
	        [](void* robot) {
		        if (robot) { static_cast<decltype(robotInstance)>(robot)->run(); }
	        },
	        robotInstance, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Scheduler");
}

RobotThread blueMogoSide() {
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

	drive->setCurrentMotion(ProfiledMotion(-26, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(true);

	intake->moveVoltage(-12000);

	drive->setCurrentMotion(PIDTurn(-90, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(29, 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(-5, 35, 40, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(-10, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(44, 35, 40, 60));
	co_yield drive->waitUntilSettled(2000);

	co_yield util::coroutine::delay(5000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}


RobotThread redMogoSide() {
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

	drive->setCurrentMotion(ProfiledMotion(-26, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(true);

	intake->moveVoltage(-12000);

	drive->setCurrentMotion(PIDTurn(-90, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(29, 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(-5, 35, 40, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(-10, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(44, 35, 40, 60));
	co_yield drive->waitUntilSettled(2000);

	co_yield util::coroutine::delay(5000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}


RobotThread blueSAWPAuton() {
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


	/*pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(250);
	drive->setCurrentMotion(PIDTurn(55, PID(750, 0, 5500, true, 10)));// for up to 90 deg
	co_yield drive->waitUntilSettled(750);
	printf("heading: %f\n", util::toDeg(odom->getCurrentState().position.theta()));
	co_yield util::coroutine::delay(250);
	printf("heading: %f\n", util::toDeg(odom->getCurrentState().position.theta()));

	drive->setCurrentMotion(PIDTurn(130, PID(650, 0, 5750, true, 10)));


	    // drive->setCurrentMotion(PIDTurn(-90, PID(150, 1, 1000, true, 10))); // itteration 1
	    // drive->setCurrentMotion(PIDTurn(-90, PID(190, 0, 1250, true, 10))); // itteration 2

	    // drive->setCurrentMotion(PIDTurn(45, PID(210, 0, 1250, true, 10)));

	    // drive->setCurrentMotion(PIDTurn(5, PID(750, 0, 5000, true, 10))); // for up to 90 deg
	    co_yield drive->waitUntilSettled(750);

	    co_yield util::coroutine::delay(250);
	    printf("heading: %f\n", util::toDeg(odom->getCurrentState().position.theta()));*/

	// move to alliance stake
	Pose curPose = odom->getCurrentState().position;
	Pose allianceStake(6.67, 0);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(allianceStake.translation()), 20, 40, 40));
	co_yield drive->waitUntilSettled(1500);

	// score ring onto alliance stake
	// co_yield util::coroutine::delay(500);
	liftFlags->targetAngle = 198;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(750);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };


	// stow lift and move to clamp mogo
	co_yield util::coroutine::delay(100);
	lift->setState(Lift::STOW);

	Pose mogoPose(-28.4, 18.1);
	curPose = odom->getCurrentState().position;
	double targetHeading = curPose.headingTo(mogoPose).degrees() + 180;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogoPose).degrees() + 180, PID(750, 0, 5000, true, 10), true, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!
	curPose = odom->getCurrentState().position;

	pros::lcd::print(4, "Err: %f", util::getShortestAngle(curPose.rotation().degrees(), targetHeading));

	pnoomaticFlags->clampMogo = true;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogoPose.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);
	co_yield util::coroutine::delay(250);

	// move to intake middle ring
	Pose middleRing(-48, 21.5);
	curPose = odom->getCurrentState().position;
	targetHeading = curPose.headingTo(middleRing).degrees();
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(middleRing).degrees(), PID(750, 0, 5500, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!

	curPose = odom->getCurrentState().position;
	pros::lcd::print(5, "Err1: %f", util::getShortestAngle(curPose.rotation().degrees(), targetHeading));
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(middleRing.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);

	// move back before second ring
	drive->setCurrentMotion(ProfiledMotion(-10, 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);

	// move to intake second ring
	// Pose secondIntakeRing(-44.9, 5.6);
	Pose secondIntakeRing(-42.8, 10.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(secondIntakeRing).degrees(), PID(750, 0, 5500, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(150);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!
	curPose = odom->getCurrentState().position;
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondIntakeRing.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);

	// Pose intermediatePose(11.5, 22.2);
	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(intermediatePose).degrees(), PID(750, 0, 500, true, 10), false,
	// false)); co_yield drive->waitUntilSettled(1000); intake->moveVoltage(0);

	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intermediatePose.translation()), 60, 60, 60));
	// co_yield drive->waitUntilSettled(2500);
	// pnoomatics->setClamp(false);

	//

	// move to be right before rings
	/*	Pose preIntake1(-35.4, 12.1);
	    curPose = odom->getCurrentState().position;
	    drive->setCurrentMotion(PIDTurn(curPose.headingTo(preIntake1).degrees(), PID(175, 1, 50, true, 10), false, false));
	    co_yield drive->waitUntilSettled(1000);
	    curPose = odom->getCurrentState().position;
	    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preIntake1.translation()), 60, 80, 80));
	    intake->moveVoltage(-12000);
	    co_yield drive->waitUntilSettled(2500);

	    // move to intake rings
	    Pose postIntake1(-49.9, 2.6);
	    curPose = odom->getCurrentState().position;
	    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(postIntake1.translation()), 40, 80, 80));
	    co_yield drive->waitUntilSettled(1000);*/
}

RobotThread skillsAuton() {
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


	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(5.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(0);

	Pose curPose = odom->getCurrentState().position;
	Pose mogo1 = Pose(13, -17.2);

	drive->setCurrentMotion(PIDTurn(90, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-17, 40, 30, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(100);


	curPose = odom->getCurrentState().position;
	Pose ring1m1 = Pose(35.5, -22.6);

	intake->moveVoltage(-12000);
	//>100
	drive->setCurrentMotion(PIDTurn(360, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(22, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	Pose ring2m1 = Pose(54, -48);

	drive->setCurrentMotion(PIDTurn(305, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(28, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);

	drive->setCurrentMotion(ProfiledMotion(-4, 50, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose prering3m1 = Pose(52.1, -44.4);

	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(prering3m1.translation()), 30, 60, 60));
	// co_yield drive->waitUntilSettled(1500);
	// co_yield util::coroutine::delay(500);

	curPose = odom->getCurrentState().position;
	Pose ring5m1 = Pose(2, -48);

	//>100
	drive->setCurrentMotion(PIDTurn(180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(52, 15, 60, 60));
	co_yield drive->waitUntilSettled(3000);

	drive->setCurrentMotion(ProfiledMotion(-5, 50, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring6m1 = Pose(8, -56.2);

	//>100
	drive->setCurrentMotion(PIDTurn(290, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);

	curPose = odom->getCurrentState().position;
	Pose dump1 = Pose(6, -62);

	drive->setCurrentMotion(PIDTurn(30, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(false);

	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(2.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose centerringm2 = Pose(54, -2);

	intake->moveVoltage(-8000);
	intake->setDistStop(true);

	// small
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(centerringm2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerringm2.translation()), 45, 60, 25));
	co_yield drive->waitUntilSettled(2000);
	co_yield util::coroutine::delay(500);

	curPose = odom->getCurrentState().position;
	Pose ring2m2 = Pose(34, 25.4);

	// no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring2m2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring2m2.translation()), 50, 60, 60));
	co_yield util::coroutine::delay(400);
	intake->moveVoltage(-8000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);

	pnoomaticFlags->clampMogo = true;
	curPose = odom->getCurrentState().position;
	Pose mogo2 = Pose(13, 23.6);

	//>100 no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);

	intake->moveVoltage(-12000);

	curPose = odom->getCurrentState().position;
	Pose ring3m2 = Pose(31.8, 46.4);


	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring3m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3m2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	Pose ring5m2 = Pose(1.6, 48);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring5m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring5m2.translation()), 35, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	Pose ring6m2 = Pose(3.0, 54.5);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring6m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring6m2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(250);

	curPose = odom->getCurrentState().position;
	Pose dump2 = Pose(-4.1, 55.6);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump2).degrees() + 180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(false);

	// drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// lift->setState(Lift::LEVEL_2);

	// curPose = odom->getCurrentState().position;
	// Pose ring1ws1 = Pose(50.8, 61.8);

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws1).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws1.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// Pose ws1 = Pose(53.7, 62.7);

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ws1).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(true);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ws1.translation()), 20, 30, 30));
	// co_yield drive->waitUntilSettled(700);

	// intake->moveVoltage(0);

	// liftFlags->targetAngle = 198;
	// lift->setState(Lift::HOLD);
	// co_yield util::coroutine::nextCycle();
	// Timeout liftTimeout = Timeout(1000);
	// co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};

	// lift->setState(Lift::STOW);

	// drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	// intake->setExtender(false);
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose ring1mogo3 = Pose(0, 0);

	// //>100 no mogo
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1mogo3).degrees(), PID(750, 0, 5000, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1mogo3.translation()), 50, 60, 60));
	// intake->moveVoltage(-3000);
	// intake->setDistStop(true);
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose ring2mogo3 = Pose(0, 0);

	// // no mogo
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1mogo3).degrees(), PID(750, 0, 5000, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1mogo3.translation()), 50, 60, 60));
	// intake->moveVoltage(-3000);
	// intake->setDistStop(true);
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose mogo3 = Pose(0, 0);

	// //>100 no mogo
	// pnoomaticFlags->clampMogo = true;
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo3).degrees() + 180, PID(750, 0, 5000, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo3.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// pnoomatics->setClamp(true);

	// intake->moveVoltage(-12000);

	// curPose = odom->getCurrentState().position;
	// Pose ring3m3 = Pose(0, 0);

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring3m3).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3m3.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);
	// drive->setCurrentMotion(ProfiledMotion(-2.5, 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose ring4m3 = Pose(0, 0);

	// // small
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring4m3).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(500);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring4m3.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose dump3 = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump3).degrees() + 180, PID(750, 0, 5000, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump3.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// pnoomatics->setClamp(false);

	// curPose = odom->getCurrentState().position;
	// Pose ring1AS2 = Pose(0, 0);

	// intake->moveVoltage(-3000);
	// intake->setDistStop(true);
	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1AS2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1AS2.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose pre1AS2 = Pose(0, 0);

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(pre1AS2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(pre1AS2.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose pre2AS2 = Pose(0, 0);

	// drive->setCurrentMotion(
	//         PIDTurn(curPose.headingTo(pre2AS2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(pre1AS2.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose AS2 = Pose(0, 0);

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(AS2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// intake->moveVoltage(-12000);
	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(pre1AS2.translation()), 25, 30, 30));
	// co_yield drive->waitUntilSettled(1500);


	// drive->setCurrentMotion(ProfiledMotion(3, 25, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose mogo4 = Pose(0, 0);

	// pnoomaticFlags->clampMogo=true;

	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo4).degrees() + 180, PID(750, 0, 5000, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo4.translation()), 25, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	// pnoomatics->setClamp(true);

	// lift->setState(Lift::LEVEL_2);

	// curPose = odom->getCurrentState().position;
	// Pose ring1ws2 = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws2.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose ring2ws2 = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// intake->moveVoltage(0);
	// lift->setState(Lift::LEVEL_2);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws2.translation()), 50, 60, 60));
	// co_yield util::coroutine::delay(300);
	// intake->moveVoltage(-12000);
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose prews2 = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(prews2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// intake->moveVoltage(0);
	// lift->setState(Lift::LEVEL_2);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(prews2.translation()), 50, 60, 60));
	// co_yield util::coroutine::delay(300);
	// intake->moveVoltage(-12000);
	// co_yield drive->waitUntilSettled(1500);

	// curPose = odom->getCurrentState().position;
	// Pose ws2 = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(true);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ws2.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);

	// intake->moveVoltage(0);

	// liftFlags->targetAngle = 198;
	// lift->setState(Lift::HOLD);
	// co_yield util::coroutine::nextCycle();
	// liftTimeout = Timeout(1000);
	// co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};

	// lift->setState(Lift::STOW);

	// curPose = odom->getCurrentState().position;
	// Pose finalcorner = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(finalcorner).degrees(), PID(750, 0, 5500, true, 10), false, false,
	// 0.5)); co_yield drive->waitUntilSettled(1000); pnoomatics->setHammer(true);
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(finalcorner.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(2000);

	// curPose = odom->getCurrentState().position;
	// Pose center = Pose(0, 0);

	// //>100
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(center).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	// pnoomatics->setHammer(false);
	// drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	// co_yield drive->waitUntilSettled(2000);


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread testAuton() {
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


	Pose curPose = odom->getCurrentState().position;
	Pose targetPose(0, -9);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(targetPose).degrees(), PID(150, 1, 50, true, 10), false, false));
	drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(targetPose.translation()), 60, 60, 60));
	// drive->waitUntilSettled(2000);

	co_yield util::coroutine::nextCycle();

	// Pose targetPose(24, 0);
	// drive->setCurrentMotion(ProfiledMotion(origin.translation().distanceTo(targetPose.translation()), 60, 60, 60));
	// drive->waitUntilSettled(1000);


	// Robot starts at origin (0,0,0)
	/*Pose origin;// starting pose
	Pose targetPose(0.0, 1.0);

	// turn to face target pose
	drive->setCurrentMotion(PIDTurn(origin.headingTo(targetPose).degrees(), PID(0, 0, 0)));
	drive->waitUntilSettled(1000);

	// move to target pose (calculates dist it needs to travel using Translation class)
	drive->setCurrentMotion(ProfiledMotion(origin.translation().distanceTo(targetPose.translation()), 60, 60, 60));
	drive->waitUntilSettled(1000);*/
}

RobotThread redMogoRush() {
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

	// drive fwd
	drive->setCurrentMotion(ProfiledMotion(-9, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	// turn and move to mogo
}

// just a simple mogo rush made by lexi + richard + jason for sugar
RobotThread redMogoRushSafe() {
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
	drive->setCurrentMotion(ProfiledMotion(-9.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	// drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));
	drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));// tuning pid so we don't undershoot
	co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-5.5, 50, 60, 60));
	drive->setCurrentMotion(ProfiledMotion(-5.3, 50, 60, 60));// decrease backup a bit
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(PIDTurn(226, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);


	// new stuff not from blueQualRingSide()
	Pose ring1(26.76, 27.25);
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1).degrees(), PID(750, 1, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);

	intake->moveVoltage(-12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	Pose corner(39.14, -6.8);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner).degrees(), PID(750, 1, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()) + 4, 50, 60, 60));
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(500);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	// drop hammer and clear corner
	pnoomatics->setHammer(true);
	co_yield util::coroutine::delay(350);
	drive->setCurrentMotion(PIDTurn(10, PID(750, 1, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	pnoomatics->setHammer(false);
}

// Tuned by lexie + richard for Sugar
RobotThread blueQualRingSideAuton() {
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

	// co_yield util::coroutine::delay(1000);
	// odom->reset();

	// co_yield util::coroutine::delay(4500);

	drive->setCurrentMotion(ProfiledMotion(-9.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	// drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));
	drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));// tuning pid so we don't undershoot
	co_yield drive->waitUntilSettled(1000);
	// drive->setCurrentMotion(ProfiledMotion(-5.5, 50, 60, 60));
	drive->setCurrentMotion(ProfiledMotion(-5.3, 50, 60, 60));// decrease backup a bit
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(PIDTurn(226, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);


	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);

	drive->setCurrentMotion(PIDTurn(40, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	intake->moveVoltage(-12000);
	// drive->setCurrentMotion(ProfiledMotion(15.5, 50, 60, 60));
	drive->setCurrentMotion(ProfiledMotion(14.1, 50, 60, 60));// reduce so we don't cross the line
	co_yield drive->waitUntilSettled(1500);

	// drive->setCurrentMotion(ProfiledMotion(-9.2, 50, 60, 60)); // lexie's
	drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	// drive->setCurrentMotion(PIDTurn(-18, PID(750, 1, 5000, true, 10), false, false)); // lexie's
	drive->setCurrentMotion(PIDTurn(-14, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	// drive->setCurrentMotion(ProfiledMotion(12, 80, 60, 80)); // lexie's
	drive->setCurrentMotion(ProfiledMotion(12, 80, 60, 80));
	co_yield drive->waitUntilSettled(1500);


	// drive->setCurrentMotion(PIDTurn(-70, PID(750, 1, 5000, true, 10), false, false)); // lexie's stuff
	drive->setCurrentMotion(PIDTurn(-73, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(900);

	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 25));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(2000);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(250);

	// turn and hit the ladder
	drive->setCurrentMotion(PIDTurn(-34.8, PID(1100, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-45, 50, 60, 60));
	co_yield util::coroutine::delay(1200);
	drive->setCurrentMotion(PIDTurn(135, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);


	/*intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1500);


	drive->setCurrentMotion(PIDTurn(315, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(-40, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(180, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(15, 50, 25, 60));
	co_yield drive->waitUntilSettled(1500);


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
	*/
}

// USE THIS FUNC FOR AUTON CODING!
// thread runs after all other threads run
RobotThread blueElimRingSideAuton() {
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

	// co_yield util::coroutine::delay(1000);
	// odom->reset();

	drive->setCurrentMotion(ProfiledMotion(-10.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-4.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	drive->setCurrentMotion(PIDTurn(226, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);


	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);

	drive->setCurrentMotion(PIDTurn(40, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(350, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(ProfiledMotion(12, 80, 60, 80));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(300, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(ProfiledMotion(-6, 30, 25, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(7, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-6, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(7, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(1000);


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread redElimRingSideAuton() {
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

	drive->setCurrentMotion(ProfiledMotion(33.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	Pose curPose = odom->getCurrentState().position;
	Pose ring1 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 35, 40, 60));
	intake->moveVoltage(-4000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose mogo1 = Pose(0, 0);

	pnoomaticFlags->clampMogo = true;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo1).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	intake->moveVoltage(-12000);

	curPose = odom->getCurrentState().position;
	Pose ring2 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring2).degrees() + 180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring2.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose corner = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);

	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 40, 40, 25));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose preAllianceStake = Pose(0, 0);

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(preAllianceStake).degrees() + 180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(preAllianceStake.translation()), 40, 40, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(false);

	curPose = odom->getCurrentState().position;
	Pose allianceStake = Pose(0, 0);

	drive->setCurrentMotion(
	        PIDTurn(-curPose.headingTo(allianceStake).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(allianceStake.translation()), 20, 25, 25));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);

	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread redQualRingSideAuton() {
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

	// co_yield util::coroutine::delay(1000);
	// odom->reset();

	// score onto wall stake
	drive->setCurrentMotion(ProfiledMotion(-11.3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(0.5, 30, 20, 20));
	co_yield drive->waitUntilSettled(750);
	drive->setCurrentMotion(PIDTurn(270, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-4.7, 50, 60, 60));// up from -4.5
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	// turn and clamp mogo
	drive->setCurrentMotion(PIDTurn(134, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);

	// turn to middle of field and intake ring
	drive->setCurrentMotion(PIDTurn(320, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	intake->moveVoltage(-12000);
	// drive->setCurrentMotion(ProfiledMotion(16, 50, 60, 60)); // initial terrence one
	drive->setCurrentMotion(ProfiledMotion(15.7, 50, 60, 60));// decreasing so it doesn't cross the line
	co_yield drive->waitUntilSettled(1500);

	// move back and get 2nd pair of rings
	drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(10, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(ProfiledMotion(12, 80, 60, 80));
	co_yield drive->waitUntilSettled(1500);

	// our version of getting the corner
	drive->setCurrentMotion(PIDTurn(69.5, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(40, 50, 60, 60));
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);

	// 40.2
	drive->setCurrentMotion(PIDTurn(45, PID(1100, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-48, 50, 60, 60));
	co_yield util::coroutine::delay(1200);
	drive->setCurrentMotion(PIDTurn(140, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	// terrence's ending to get to the middle
	/*drive->setCurrentMotion(PIDTurn(60, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(45, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(-40, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(180, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(15, 50, 25, 60));
	co_yield drive->waitUntilSettled(1500);*/


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread blueLexieTunedQualRingSideAuton() {
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

	// co_yield util::coroutine::delay(1000);
	// odom->reset();

	drive->setCurrentMotion(ProfiledMotion(-13, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(PIDTurn(90, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-4.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1700);
	intake->moveVoltage(0);


	drive->setCurrentMotion(PIDTurn(-136, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);


	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);
	/*

	drive->setCurrentMotion(PIDTurn(320, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(10, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(ProfiledMotion(12, 80, 60, 80));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(60, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(45, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(-40, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(180, PID(750, 0, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	drive->setCurrentMotion(ProfiledMotion(15, 50, 25, 60));
	co_yield drive->waitUntilSettled(1500);*/


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread blueMogoRush() {
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

	co_yield util::coroutine::delay(4500);// delay for other team

	drive->setCurrentMotion(ProfiledMotion(-11.3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(0.5, 30, 20, 20));
	co_yield drive->waitUntilSettled(750);
	drive->setCurrentMotion(PIDTurn(270, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-4.7, 50, 60, 60));// up from -4.5
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(500);
	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	// intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(0);

	// turn and clamp mogo
	drive->setCurrentMotion(PIDTurn(134, PID(750, 1, 5000, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-33, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
	co_yield util::coroutine::delay(300);


	drive->setCurrentMotion(PIDTurn(7, PID(750, 1, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(20, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	// Pose ring1(-28.2, 8.7);
	// Pose curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1).degrees(), PID(750, 1, 5000, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(800);

	// intake->moveVoltage(-12000);
	// curPose = odom->getCurrentState().position;
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);


	/*robotInstance->getFlag<Intake>().value()->distStop = true;
	intake->moveVoltage(-12000);

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	// rush to center
	drive->setCurrentMotion(ProfiledMotion(43, 50, 60, 60));
	co_yield drive->waitUntilDist(37);
	lift->setClaw(true);
	pnoomatics->setHammer(true);// set hammer down
	co_yield drive->waitUntilSettled(2000);

	// move back and turn towards second mogo
	drive->setCurrentMotion(ProfiledMotion(-10.5, 40, 30, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(110, PID(250, 1, 45, true, 10), false, false, 0.5, 5500));
	co_yield drive->waitUntilSettled(1800);
	pnoomatics->setHammer(false);// set hammer up
	co_yield util::coroutine::delay(200);
	robotInstance->getFlag<Intake>().value()->distStop = false;

	// get the second mogo
	drive->setCurrentMotion(ProfiledMotion(-19, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnoomatics->setClamp(true);
	intake->moveVoltage(-12000);

	// move to clear corner
	drive->setCurrentMotion(PIDTurn(155, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1800);
	pnoomatics->setHammer(false);*/

	// move to alliance stake
	// drive->setCurrentMotion(PIDTurn(245, PID(150, 1, 45, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// intake->setExtender(true);
	// robotInstance->getFlag<Intake::flags>().value()->distStop = true;
	// // intake->moveVoltage(-12000);

	// drive->setCurrentMotion(ProfiledMotion(30, 50, 60, 60));
	// co_yield drive->waitUntilSettled(1500);


	// drive->setCurrentMotion(ProfiledMotion(-3, 20, 30, 30));
	// intake->setExtender(false);
	// co_yield drive->waitUntilSettled(1500);

	// // co_yield []() -> bool { return !robotInstance->getFlag<Intake::flags>().value()->distStop; };

	// drive->setCurrentMotion(ProfiledMotion(2, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	// drive->setCurrentMotion(PIDTurn(210, PID(150, 1, 45, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);

	// drive->setCurrentMotion(ProfiledMotion(12, 20, 30, 30));
	// co_yield drive->waitUntilSettled(1500);

	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	robotInstance->getSubsystem<Odometry>().value()->reset();
	auto coro = blueQualRingSideAuton();
	// auto coro = redQualRingSideAuton();
	// auto coro = redMogoRushSafe();
	// auto coro = blueMogoRush();
	while (coro) { co_yield coro(); }

	// if (robotInstance->curAlliance == Alliance::BLUE) {
	// 	switch (robotInstance->curAuton) {
	// 		case Auton::ELIM:
	// 			{auto coro = blueMogoSide();
	// 			while (coro) { co_yield coro(); }}
	// 			break;
	// 		case Auton::QUAL:
	// 			{auto coro = blueQualRingSideAuton();
	// 			while (coro) { co_yield coro(); }}
	// 			break;
	// 	}
	// }

	// if (robotInstance->curAlliance == Alliance::RED) {
	// 	switch (robotInstance->curAuton) {
	// 		case Auton::ELIM:
	// 			{auto coro = redMogoSide();
	// 			while (coro) { co_yield coro(); }}
	// 			break;
	// 		case Auton::QUAL:
	// 			{
	// 			auto coro = redQualRingSideAuton();
	// 			while (coro) { co_yield coro(); }}
	// 			break;
	// 	}
	// }

	co_yield util::coroutine::nextCycle();
}

// !!!!!!!!!!!!!!!!!!!!!
// DON'T USE THESE FUNCS
// !!!!!!!!!!!!!!!!!!!!!

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
	// while (true) {
	// 	printf("Target Velocity: %d\n", pros::c::motor_get_target_velocity(1));
	// 	pros::c::motor_move_velocity(1, 50);
	// 	// pros::c::motor_move_voltage(1, 5000);
	// 	// printf("OPCONTROL RUNNING\n");
	// 	// pros::delay(50);
	// 	// printCompFlagBits();
	// 	pros::delay(10);
	// }
	// static char buffer[2046];
	// static char buffer2[2046];
	// pros::delay(1000);
	// vTaskGetRunTimeStats(buffer);
	// printf("%s\n", buffer);
	// pros::delay(1000);
	// vTaskGetRunTimeStats(buffer2);
	// printf("%s\n", buffer2);
}

// conversion factor (abs time to ms): ((3/2)/1000000)