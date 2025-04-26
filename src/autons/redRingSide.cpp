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

	drive->setCurrentMotion(ProfiledMotion(3, 60, 60, 60));
	/*liftFlags->targetAngle = 50;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(800);// TUNE THIS
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); }*/
	;

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