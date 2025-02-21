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

	drive->setCurrentMotion(PIDTurn(315, PID(200, 1, 100, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	liftFlags->targetAngle = 210;
	lift->setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(700);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut();};
	drive->setCurrentMotion(ProfiledMotion(-5, 50, 60, 45));
	co_yield drive->waitUntilSettled(1500);

	lift->setState(Lift::STOW);
	co_yield util::coroutine::nextCycle();

	drive->setCurrentMotion(PIDTurn(340, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);
	drive->setCurrentMotion(ProfiledMotion(-34, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	intake->moveVoltage(12000);
	// original: 100
	drive->setCurrentMotion(PIDTurn(90, PID(150, 1, 200, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	drive->setCurrentMotion(ProfiledMotion(20, 60, 60, 60));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(0, PID(150, 1, 200, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	drive->setCurrentMotion(ProfiledMotion(32, 60, 60, 60));
	co_yield drive->waitUntilSettled(1500);

// turn and clear corner
	drive->setCurrentMotion(PIDTurn(60, PID(200, 1, 45, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);
	pnoomatics->setRightHammer(true);

// comment out the intaking of corner (only clear now)
	// drive->setCurrentMotion(ProfiledMotion(24, 50, 60, 25));
	// co_yield drive->waitUntilSettled(1500);
	// drive->setCurrentMotion(TimedMotion(500, 12000));
	// co_yield drive->waitUntilSettled(500);

	drive->setCurrentMotion(ProfiledMotion(14, 50, 60, 25));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(220, PID(120, 1, 155, true, 10), false, false, 0.5, 12000, true, false));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setRightHammer(false);

// TUNE THIS SPEED!!! So it aint too fast
	drive->setCurrentMotion(ProfiledMotion(10, 65, 60, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(false);

	drive->setCurrentMotion(PIDTurn(350, PID(120, 1, 155, true, 10), false, false, 0.5, 12000, true, false));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(ProfiledMotion(-20, 65, 60, 60));
	co_yield drive->waitUntilSettled(1500);


	/*drive->setCurrentMotion(ProfiledMotion(60, 50, 60, 25));

	if (robotInstance->isElim) {

	    co_yield util::coroutine::delay(750);

	    liftFlags->targetAngle = 100;
	    lift->setState(Lift::HOLD);
	    co_yield util::coroutine::nextCycle();
	}*/
	co_yield drive->waitUntilSettled(2000);
	intake->moveVoltage(0);
}