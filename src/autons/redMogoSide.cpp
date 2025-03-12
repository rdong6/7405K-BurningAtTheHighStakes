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

RobotThread redMogoSide() {
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
	drive->setCurrentMotion(ProfiledMotion(2.5, 60, 100, 60));
	co_yield drive->waitUntilSettled(2000);

	// score on alliance stake w/ preload
	liftFlags->targetAngle = 220;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(255);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	// move back first to get mogo

	Pose mogo1(-35.7,0);

	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 100, 85));
	co_yield util::coroutine::delay(600);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	Pose ring1(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring1).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring1.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	pnoomatics->setRightHammer(true);
	co_yield util::coroutine::delay(150);

	Pose setupRingIntake(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(setupRingIntake).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(setupRingIntake.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose ringIntake(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ringIntake).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield util::coroutine::delay(120);
	pnoomatics->setRightHammer(false);
	co_yield drive->waitUntilSettled(480);

	intake->moveVoltage(12000);
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringIntake.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose corner(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(corner).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(corner.translation()), 50, 100, 65));
	co_yield util::coroutine::delay(500);
	intake->moveVoltage(0);
	co_yield util::coroutine::delay(400);
	intake->moveVoltage(12000);
	co_yield drive->waitUntilSettled(800);

	drive->setCurrentMotion(TimedMotion(400, 12000));
	co_yield drive->waitUntilSettled(195);

	drive->setCurrentMotion(ProfiledMotion(-25, 50, 80, 65));
	co_yield drive->waitUntilSettled(900);

	pnoomatics->setLeftHammer(true);

	drive->setCurrentMotion(ProfiledMotion(17, 50, 100, 85));
	co_yield drive->waitUntilSettled(900);

	Pose ladder(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ladder).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ladder.translation()), 50, 100, 65));
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	intake->moveVoltage(0);
	pnoomatics->setLeftHammer(false);
	co_yield util::coroutine::delay(500);
	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	pnoomatics->setClamp(false);
	co_yield drive->waitUntilSettled(1500);
}