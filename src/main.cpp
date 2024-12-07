#include "main.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/controller/RAMSETE.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/DelayedBool.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
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

	robotTask = pros::c::task_create(
	        [](void* robot) {
		        if (robot) { static_cast<decltype(robotInstance)>(robot)->run(); }
	        },
	        robotInstance, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Scheduler");
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// auto drive = GET_SUBSYSTEM(Drive);

	pros::lcd::initialize();
	logger_initialize("test.txt", 100);

	Pose origin;
	// Pose target(0, 1, 0); // 90 deg
	// Pose target(0, -1, 0);// -90 deg

	Pose target(-1, -1, 0);
	double error = util::getShortestAngle(util::toDeg(origin.theta()), origin.headingTo(target).degrees());
	printf("Heading to: %f  Error: %f\n", origin.headingTo(target).degrees(), error);
	printf("Dist: %f\n", origin.translation().distanceTo(target.translation()));

	// exit(0);

	robot_init();
	// pros::delay(250);
	// robotInstance->getSubsystem<Odometry>().value()->reset();
}

RobotThread redMogoRush() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto odom = robotInstance->getSubsystem<Odometry>().value();
	auto pnooomaticFlags = robotInstance->getFlag<Pnooomatics>().value();

	drive->setCurrentMotion(ProfiledMotion(-28, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	Pose curPose = odom->getCurrentState().position;
	Pose mogo1 = Pose(0, 0);

	pnooomaticFlags->clampMogo = true;

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo1).degrees() + 180, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);
	co_yield util::coroutine::delay(100);

	bool firstMogoClamped = !pnooomaticFlags->clampMogo;
	if (firstMogoClamped) {
		intake->moveVoltage(-12000);
	} else {
		pnooomaticFlags->clampMogo = false;
	}

	drive->setCurrentMotion(PIDTurn(317, PID(200, 1, 45, true, 10), false, true, 0.5));
	co_yield drive->waitUntilSettled(700);

	if (!firstMogoClamped) {
		intake->moveVoltage(-3000);
		intake->setDistStop(true);
	}

	drive->setCurrentMotion(ProfiledMotion(3, 60, 60, 40));
	co_yield util::coroutine::delay(200);
	pnooomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose mogo2 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo2).degrees() + 180, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield util::coroutine::delay(200);
	intake->moveVoltage(0);
	co_yield drive->waitUntilSettled(1500);

	pnooomaticFlags->clampMogo = true;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 35, 45, 30));
	co_yield drive->waitUntilSettled(1500);
	intake->moveVoltage(-12000);

	curPose = odom->getCurrentState().position;
	Pose preCorner = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preCorner).degrees(), PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preCorner.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose corner = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner).degrees(), PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	pnooomatics->setHammer(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	// corner clear turn
	drive->setCurrentMotion(PIDTurn(170, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(800);
	pnooomatics->setHammer(false);

	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(15, 15, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose dump = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump).degrees() + 180, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ladder = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ladder).degrees() + 180, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(ladder.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(2000);

	// maybe we touch cone maybe we raise lady brown


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}


RobotThread blueSAWPAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto pnoomaticsFlags = robotInstance->getFlag<Pnooomatics>().value();
	auto odometry = robotInstance->getSubsystem<Odometry>().value();

	/*pnooomatics->setClamp(true);
	co_yield util::coroutine::delay(250);
	drive->setCurrentMotion(PIDTurn(55, PID(750, 0, 5500, true, 10)));// for up to 90 deg
	co_yield drive->waitUntilSettled(750);
	printf("heading: %f\n", util::toDeg(odometry->getCurrentState().position.theta()));
	co_yield util::coroutine::delay(250);
	printf("heading: %f\n", util::toDeg(odometry->getCurrentState().position.theta()));

	drive->setCurrentMotion(PIDTurn(130, PID(650, 0, 5750, true, 10)));


	    // drive->setCurrentMotion(PIDTurn(-90, PID(150, 1, 1000, true, 10))); // itteration 1
	    // drive->setCurrentMotion(PIDTurn(-90, PID(190, 0, 1250, true, 10))); // itteration 2

	    // drive->setCurrentMotion(PIDTurn(45, PID(210, 0, 1250, true, 10)));

	    // drive->setCurrentMotion(PIDTurn(5, PID(750, 0, 5000, true, 10))); // for up to 90 deg
	    co_yield drive->waitUntilSettled(750);

	    co_yield util::coroutine::delay(250);
	    printf("heading: %f\n", util::toDeg(odometry->getCurrentState().position.theta()));*/

	// move to alliance stake
	Pose curPose = odometry->getCurrentState().position;
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
	curPose = odometry->getCurrentState().position;
	double targetHeading = curPose.headingTo(mogoPose).degrees() + 180;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogoPose).degrees() + 180, PID(750, 0, 5000, true, 10), true, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!
	curPose = odometry->getCurrentState().position;

	pros::lcd::print(4, "Err: %f", util::getShortestAngle(curPose.rotation().degrees(), targetHeading));

	pnoomaticsFlags->clampMogo = true;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogoPose.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);
	co_yield util::coroutine::delay(250);

	// move to intake middle ring
	Pose middleRing(-48, 21.5);
	curPose = odometry->getCurrentState().position;
	targetHeading = curPose.headingTo(middleRing).degrees();
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(middleRing).degrees(), PID(750, 0, 5500, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!

	curPose = odometry->getCurrentState().position;
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
	curPose = odometry->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(secondIntakeRing).degrees(), PID(750, 0, 5500, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);
	co_yield util::coroutine::delay(150);
	co_yield util::coroutine::delay(500);// TEMPORARY!!!
	curPose = odometry->getCurrentState().position;
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(secondIntakeRing.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(2500);

	// Pose intermediatePose(11.5, 22.2);
	// curPose = odometry->getCurrentState().position;
	// drive->setCurrentMotion(PIDTurn(curPose.headingTo(intermediatePose).degrees(), PID(750, 0, 500, true, 10), false,
	// false)); co_yield drive->waitUntilSettled(1000); intake->moveVoltage(0);

	// curPose = odometry->getCurrentState().position;
	// drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intermediatePose.translation()), 60, 60, 60));
	// co_yield drive->waitUntilSettled(2500);
	// pnooomatics->setClamp(false);

	//

	// move to be right before rings
	/*	Pose preIntake1(-35.4, 12.1);
	    curPose = odometry->getCurrentState().position;
	    drive->setCurrentMotion(PIDTurn(curPose.headingTo(preIntake1).degrees(), PID(175, 1, 50, true, 10), false, false));
	    co_yield drive->waitUntilSettled(1000);
	    curPose = odometry->getCurrentState().position;
	    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preIntake1.translation()), 60, 80, 80));
	    intake->moveVoltage(-12000);
	    co_yield drive->waitUntilSettled(2500);

	    // move to intake rings
	    Pose postIntake1(-49.9, 2.6);
	    curPose = odometry->getCurrentState().position;
	    drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(postIntake1.translation()), 40, 80, 80));
	    co_yield drive->waitUntilSettled(1000);*/
}

RobotThread skillsAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto odom = robotInstance->getSubsystem<Odometry>().value();
	auto pnooomaticFlags = robotInstance->getFlag<Pnooomatics>().value();

	intake->moveVoltage(-12000);
	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	drive->setCurrentMotion(ProfiledMotion(13, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);

	Pose curPose = odom->getCurrentState().position;
	Pose mogo1 = Pose(13, -17.2);

	pnooomaticsFlags->clampMogo = true;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo1).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 40, 30, 60));
	co_yield drive->waitUntilSettled(1500);

	pnooomatics->setClamp(true);

	curPose = odom->getCurrentState().position;
	Pose ring1m1 = Pose(37.5, -24.6);

	intake->moveVoltage(-12000);
	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1m1).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1m1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring2m1 = Pose(58.7, -53.5);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring2m1).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring2m1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose prering3m1 = Pose(52.1, -44.4);

	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(prering3m1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring5m1 = Pose(9.3, -48.2);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring5m1).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring5m1.translation()), 35, 40, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring6m1 = Pose(12.3, -56.2);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring6m1).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring6m1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose dump1 = Pose(5.2, -60.2);


	drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump1).degrees() + 180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnooomatic->setClamp(false);

	drive->setCurrentMotion(ProfiledMotion(2.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose centerringm2 = Pose(55.9, 0);

	intake->moveVoltage(-3000);
	intake->setDistStop(true);

	// small
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(centerringm2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(centerringm2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring2m2 = Pose(32.5, 25.4);

	// no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring2m2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring2m2.translation()), 50, 60, 60));
	co_yield util::coroutine::delay(400);
	intake->moveVoltage(-3000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);

	pnooomaticsFlags->clampMogo = true;
	curPose = odom->getCurrentState().position;
	Pose mogo2 = Pose(14, 23.6);

	//>100 no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	intake->moveVoltage(-12000);

	curPose = odom->getCurrentState().position;
	Pose ring3m2 = Pose(31.8, 46.4);


	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring3m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3m2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring5m2 = Pose(1.6, 45.2);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring5m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring5m2.translation()), 35, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring6m2 = Pose(3.0, 52.5);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring6m2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring6m2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose dump2 = Pose(-4.1, 55.6);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump2).degrees() + 180, PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnooomatics->setClamp(false);

	drive->setCurrentMotion(ProfiledMotion(5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::LEVEL_2);

	curPose = odom->getCurrentState().position;
	Pose ring1ws1 = Pose(50.8, 61.8);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws1).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws1.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	Pose ws1 = Pose(53.7, 62.7);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ws1).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->setExtender(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ws1.translation()), 20, 30, 30));
	co_yield drive->waitUntilSettled(700);

	intake->moveVoltage(0);

	liftFlags->targetAngle = 198;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(1000);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut() };

	lift->setState(Lift::STOW);

	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	intake->setExtender(false);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring1mogo3 = Pose(0, 0);

	//>100 no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1mogo3).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1mogo3.translation()), 50, 60, 60));
	intake->moveVoltage(-3000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring2mogo3 = Pose(0, 0);

	// no mogo
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1mogo3).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1mogo3.translation()), 50, 60, 60));
	intake->moveVoltage(-3000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose mogo3 = Pose(0, 0);

	//>100 no mogo
	pnooomaticFlags->clampMogo = true;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo3).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo3.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnooomatics->setClamp(true);

	intake->moveVoltage(-12000);

	curPose = odom->getCurrentState().position;
	Pose ring3m3 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring3m3).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring4m3.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	drive->setCurrentMotion(ProfiledMotion(-2.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring4m3 = Pose(0, 0);

	// small
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring4m3).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring4m3.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose dump3 = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(dump3).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(dump3.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	pnooomatics->setClamp(false);

	curPose = odom->getCurrentState().position;
	Pose ring1AS2 = Pose(0, 0);

	intake->moveVoltage(-3000);
	intake->setDistStop(true);
	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1AS2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1AS2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose pre1AS2 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(pre1AS2).degrees(), PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(pre1AS2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose pre2AS2 = Pose(0, 0);

	drive->setCurrentMotion(
	        PIDTurn(curPose.headingTo(pre2AS2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(pre1AS2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose AS2 = Pose(0, 0);

	drive->setCurrentMotion(PIDTurn(curPose.headingTo(AS2).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(pre1AS2.translation()), 25, 30, 30));
	co_yield drive->waitUntilSettled(1500);


	drive->setCurrentMotion(ProfiledMotion(3, 25, 30, 30));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose mogo4 = Pose(0, 0);

	pnooomaticFlags
	        ->set

	                drive->setCurrentMotion(
	                        PIDTurn(curPose.headingTo(mogo4).degrees() + 180, PID(750, 0, 5000, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo4.translation()), 25, 30, 30));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::LEVEL_2);

	curPose = odom->getCurrentState().position;
	Pose ring1ws2 = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ring2ws2 = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(0);
	lift->setState(Lift::LEVEL_2);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1ws2.translation()), 50, 60, 60));
	co_yield util::coroutine::delay(300);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose prews2 = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(prews2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->moveVoltage(0);
	lift->setState(Lift::LEVEL_2);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(prews2.translation()), 50, 60, 60));
	co_yield util::coroutine::delay(300);
	intake->moveVoltage(-12000);
	co_yield drive->waitUntilSettled(1500);

	curPose = odom->getCurrentState().position;
	Pose ws2 = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ws2).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	intake->setExtender(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ws2.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	intake->moveVoltage(0);

	liftFlags->targetAngle = 198;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(1000);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut() };

	lift->setState(Lift::STOW);

	curPose = odom->getCurrentState().position;
	Pose finalcorner = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(finalcorner).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setHammer(true);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(finalcorner.translation()), 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	curPose = odom->getCurrentState().position;
	Pose center = Pose(0, 0);

	//>100
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(center).degrees(), PID(750, 0, 5500, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setHammer(false);
	drive->setCurrentMotion(-ProfiledMotion(-10, 50, 60, 60));
	co_yield drive->waitUntilSettled(2000);


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread testAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();
	auto pnoomaticsFlags = robotInstance->getFlag<Pnooomatics>().value();
	auto odometry = robotInstance->getSubsystem<Odometry>().value();

	Pose curPose = odometry->getCurrentState().position;
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

// USE THIS FUNC FOR AUTON CODING!
// thread runs after all other threads run
RobotThread redRingSideAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	drive->setCurrentMotion(ProfiledMotion(34.25, 50, 60, 60));
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);
	lift->setClaw(true);

	drive->setCurrentMotion(PIDTurn(314, PID(190, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(-16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	drive->setCurrentMotion(PIDTurn(30, PID(190, 1, 50, true, 10), false, true));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(10, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-50, 50, 60, 60));
	co_yield drive->waitUntilSettled(3000);
	drive->setCurrentMotion(PIDTurn(120, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// intake->setExtender(true);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(12, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(false);
	drive->setCurrentMotion(ProfiledMotion(-8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(137, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(18.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(217, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// drive->setCurrentMotion(ProfiledMotion(3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	liftFlags->targetAngle = 190;
	liftFlags->pid = PID(1000, 10, 0, true, 10);
	liftFlags->isMotionRunning = true;

	// put ring on stake
	lift->setClaw(false);
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	liftFlags->pid = PID(400, 10, 0, true, 10);
	liftFlags->targetAngle = 0;

	co_yield util::coroutine::delay(500);
	liftFlags->isMotionRunning = false;

	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread blueRingSideAuton() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	drive->setCurrentMotion(ProfiledMotion(34, 50, 60, 60));
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	co_yield drive->waitUntilSettled(1500);
	lift->setClaw(true);

	drive->setCurrentMotion(PIDTurn(46, PID(190, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(-16, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);

	drive->setCurrentMotion(PIDTurn(330, PID(190, 1, 50, true, 10), true, false));
	co_yield drive->waitUntilSettled(800);
	intake->moveVoltage(-12000);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(8, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-3, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(350, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(400);
	intake->moveVoltage(-12000);
	drive->setCurrentMotion(ProfiledMotion(10, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(-50, 50, 60, 60));
	co_yield drive->waitUntilSettled(3000);
	drive->setCurrentMotion(PIDTurn(215, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);
	// intake->setExtender(true);
	intake->setDistStop(true);
	drive->setCurrentMotion(ProfiledMotion(13, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	// intake->setExtender(false);
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(234, PID(250, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(18.5, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(PIDTurn(140, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(800);

	lift->setClaw(false);
	co_yield [=]() -> bool { return !liftFlags->isMoving; };
	liftFlags->pid = PID(400, 10, 0, true, 10);
	liftFlags->targetAngle = 0;

	co_yield util::coroutine::delay(500);
	liftFlags->isMotionRunning = false;


	co_yield util::coroutine::delay(1000);
	intake->moveVoltage(0);
	co_yield util::coroutine::nextCycle();
}

RobotThread blueMogoRush() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();
	auto drive = driveOpt.value();
	auto intake = robotInstance->getSubsystem<Intake>().value();
	auto lift = robotInstance->getSubsystem<Lift>().value();
	auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();
	auto liftFlags = robotInstance->getFlag<Lift>().value();

	robotInstance->getFlag<Intake>().value()->distStop = true;
	intake->moveVoltage(-12000);

	liftFlags->targetAngle = 107;
	liftFlags->isHolding = true;

	// rush to center
	drive->setCurrentMotion(ProfiledMotion(43, 50, 60, 60));
	co_yield drive->waitUntilDist(37);
	lift->setClaw(true);
	pnooomatics->setHammer(true);// set hammer down
	co_yield drive->waitUntilSettled(2000);

	// move back and turn towards second mogo
	drive->setCurrentMotion(ProfiledMotion(-10.5, 40, 30, 60));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(PIDTurn(110, PID(250, 1, 45, true, 10), false, false, 0.5, 5500));
	co_yield drive->waitUntilSettled(1800);
	pnooomatics->setHammer(false);// set hammer up
	co_yield util::coroutine::delay(200);
	robotInstance->getFlag<Intake>().value()->distStop = false;

	// get the second mogo
	drive->setCurrentMotion(ProfiledMotion(-19, 50, 60, 60));
	co_yield drive->waitUntilSettled(1000);
	pnooomatics->setClamp(true);
	intake->moveVoltage(-12000);

	// move to clear corner
	drive->setCurrentMotion(PIDTurn(155, PID(150, 1, 45, true, 10), false, false));
	co_yield drive->waitUntilSettled(1800);
	pnooomatics->setHammer(false);

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
	auto coro = blueSAWPAuton();
	while (coro) { co_yield coro(); }

	// auto coro = skillsAuton();
	// while (coro) { co_yield coro(); }

	// auto coro = redMogoRush();
	// while (coro) { co_yield coro(); }

	// auto driveOpt = robotInstance->getSubsystem<Drive>();
	// auto drive = driveOpt.value();
	// auto intake = robotInstance->getSubsystem<Intake>().value();
	// auto lift = robotInstance->getSubsystem<Lift>().value();
	// auto pnooomatics = robotInstance->getSubsystem<Pnooomatics>().value();

	// co_yield util::coroutine::nextCycle();

	// drive->setCurrentMotion(ProfiledMotion(-45, 50, 60, 60));
	//

	// // move backwards to clamp mogo
	// drive->setCurrentMotion(TimedMotion(300, -6000));
	// co_yield drive->waitUntilSettled(300);
	// pnooomatics->setClamp(true);
	// co_yield util::coroutine::delay(300);// TBD

	// // turn to intake rings
	// drive->setCurrentMotion(PIDTurn(0 /* TBD */, PID(250, 30, 30, true, 10)));
}


// !!!!!!!!!!!!!!!!!!!!!
// DON'T USE THESE FUNCS
// !!!!!!!!!!!!!!!!!!!!!

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
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