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

	Pose rush(49,0);

	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(rush.translation()), 60, 120, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
    co_yield util::coroutine::delay(600);
    pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(2000);

	Pose mogo1(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(mogo1).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setClamp(true);
    intake->setDistStop(false);

	Pose intakeR1R2R3(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(intakeR1R2R3).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	
	co_yield util::coroutine::delay(190);
    pnoomatics->setLeftHammer(false);
	co_yield drive->waitUntilSettled(400);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intakeR1R2R3.translation()), 40, 60, 50));
	co_yield drive->waitUntilSettled(2000);

	
	// co_yield util::coroutine::delay(300);

	Pose corner(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 50, 100, 85));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(0);
	co_yield util::coroutine::delay(400);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(TimedMotion(200, 12000));
	co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-20, 50, 100, 100));
	co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(12, 50, 100, 100));
	co_yield drive->waitUntilSettled(500);

    intake->moveVoltage(12000);

	Pose preload(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(preload).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(preload.translation()), 60, 100, 85));
	lift->setState(Lift::LEVEL_1);
	co_yield drive->waitUntilSettled(2000);
	co_yield util::coroutine::delay(300);

	Pose ringStack(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ringStack).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(300);
	

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringStack.translation()), 60, 100, 85));
	intake->setExtender(true);
	co_yield util::coroutine::delay(300);
	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::delay(100);
	co_yield drive->waitUntilSettled(2000);
	
	intake->moveVoltage(12000);
	intake->setExtender(false);
	drive->setCurrentMotion(ProfiledMotion(-10, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);

	Pose allianceStake(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(allianceStake).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	
	drive->setCurrentMotion(ProfiledMotion(-8, 60, 100, 85));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(600);

	drive->setCurrentMotion(ProfiledMotion(-10, 60, 100, 85));
	co_yield drive->waitUntilSettled(1000);
	liftFlags->targetAngle = 80;
	lift->setState(Lift::HOLD);

	Pose ladder(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ladder).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	liftFlags->targetAngle = 175;
	lift->setState(Lift::HOLD);
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 60, 100, 85));
	drive->setBreakMode(pros::E_MOTOR_BRAKE_COAST);
	co_yield drive->waitUntilSettled(2000);


	// elims with no ring ah
	// drive->setCurrentMotion(PIDTurn(235, PID(120, 1, 170, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(1000);
	//
	// pnoomatics->setRightHammer(true);
	// intake->moveVoltage(-12000);
	//
	// drive->setCurrentMotion(ProfiledMotion(30, 50, 100, 120));
	// co_yield drive->waitUntilSettled(2500);


	//    drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 100));
	//	co_yield drive->waitUntilSettled(1500);
	//
	//    co_yield util::coroutine::delay(100);
	//
	//    drive->setCurrentMotion(ProfiledMotion(18, 50, 60, 60));
	//	co_yield drive->waitUntilSettled(1500);
	//     co_yield util::coroutine::delay(300);

	//    drive->setCurrentMotion(PIDTurn(0, PID(150, 1, 170, true, 10), false, false, 0.5));
	//	co_yield drive->waitUntilSettled(600);
	//
	//    liftFlags->targetAngle = 170;
	//	lift->setState(Lift::HOLD);
	//	co_yield drive->waitUntilSettled(300);
	//
	//	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	//	drive->setCurrentMotion(ProfiledMotion(19, 50, 100, 60));
	//	co_yield drive->waitUntilSettled(1500);

	co_yield util::coroutine::delay(2000);
	intake->moveVoltage(0);
}