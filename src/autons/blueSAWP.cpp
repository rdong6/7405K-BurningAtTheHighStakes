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

	drive->setCurrentMotion(ProfiledMotion(2.5, 60, 60, 60));
	co_yield drive->waitUntilSettled(2000);

	// score on alliance stake w/ preload
	liftFlags->targetAngle = 220;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(255);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };

	Pose mogo1(-35.7,0);
	// move back first to get mogo
	Pose curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(mogo1.translation()), 60, 120, 100));
	co_yield util::coroutine::delay(600);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);

	Pose preRings(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(preRings).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(preRings.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose intakeRings(0,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(intakeRings).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(intakeRings.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose ring3(0,0);
	//pre ring stack

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ring3).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ring3.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);


	Pose ringStack(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(ringStack).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(ringStack.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	drive->setCurrentMotion(ProfiledMotion(-10, 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);
	
	drive->setCurrentMotion(ProfiledMotion(20, 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose alliancePreload(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(alliancePreload).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(alliancePreload.translation()), 60, 100, 85));
	co_yield util::coroutine::delay(300);
	intake->setDistStop(true);
	pnoomatics->setClamp(false);
	//maybe when the ring has gone onto the mogo (check the flag for the top dst sensor)
	co_yield drive->waitUntilSettled(2000);

	Pose mogo2(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(mogo2).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(mogo2.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);
	intake->setDistStop(false);
	pnoomatics->setClamp(true);
	intake->moveVoltage(12000);
   
	Pose m2Ring1(49,0);
	
	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(curPose.headingTo(m2Ring1).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(ProfiledMotion(curPose.translation().distanceTo(m2Ring1.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	Pose ladder(49,0);

	curPose = odom->getCurrentState().position;
	drive->setCurrentMotion(PIDTurn(180+curPose.headingTo(ladder).degrees(),PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	curPose = odom->getCurrentState().position;
	liftFlags->targetAngle = 70;
	lift->setState(Lift::HOLD);
	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	drive->setCurrentMotion(ProfiledMotion(-curPose.translation().distanceTo(ladder.translation()), 60, 100, 85));
	co_yield drive->waitUntilSettled(2000);

	co_yield util::coroutine::nextCycle();
}