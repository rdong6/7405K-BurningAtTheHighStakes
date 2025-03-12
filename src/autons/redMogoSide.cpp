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
	drive->setCurrentMotion(ProfiledMotion(-35.7, 50, 100, 60));
	co_yield util::coroutine::delay(600);
	lift->setState(Lift::STOW);
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setClamp(true);

	// mogo clamped, turn + move to get the 2 rings under the tower
	drive->setCurrentMotion(PIDTurn(266.75,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	drive->setCurrentMotion(ProfiledMotion(19, 50, 100, 80));
	co_yield drive->waitUntilSettled(1100);

	// ring rush hamer get the 2 rings
	// get one, hammer
	pnoomatics->setRightHammer(true);
	co_yield util::coroutine::delay(150);
	drive->setCurrentMotion(ProfiledMotion(-38, 50, 100, 80));
	co_yield drive->waitUntilSettled(2000);
	pnoomatics->setRightHammer(false);

	drive->setCurrentMotion(PIDTurn(222,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	co_yield drive->waitUntilSettled(550);
	drive->setCurrentMotion(ProfiledMotion(25, 50, 100, 60));
	co_yield drive->waitUntilSettled(1300);
	co_yield util::coroutine::delay(500);

	// now intake last ring

	drive->setCurrentMotion(PIDTurn(123.5,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);
	// drive->setCurrentMotion(ProfiledMotion(22, 50, 100, 100));
	drive->setCurrentMotion(ProfiledMotion(20, 50, 100, 40));
	intake->moveVoltage(12000);

	drive->setCurrentMotion(PIDTurn(50,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(40, 50, 100, 45));
	intake->moveVoltage(12000);
	// pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(ProfiledMotion(-20, 50, 100, 80));
	co_yield drive->waitUntilSettled(1500);

	pnoomatics->setLeftHammer(true);
	drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 80));
	co_yield drive->waitUntilSettled(1500);
	// pnoomatics->setClamp(false);

	drive->setCurrentMotion(PIDTurn(255,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);

	drive->setCurrentMotion(ProfiledMotion(5, 50, 100, 80));
	co_yield drive->waitUntilSettled(1500);

	drive->setCurrentMotion(PIDTurn(255,PID(620, 1, 6500), false, false, 0.5, 12000, false, false));
	co_yield drive->waitUntilSettled(600);



	drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	// drive->setCurrentMotion(ProfiledMotion(45, 50, 100, 30));
	drive->setCurrentMotion(TimedMotion(600, 8000));
	intake->moveVoltage(0);
	pnoomatics->setLeftHammer(false);
	liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(600);

	// arc turn with right side of drive going backwards
	// drive->setCurrentMotion(PIDTurn(240, PID(200, 1, 45, true, 10), true, false, 0.5));
	// co_yield drive->waitUntilSettled(500);
	intake->moveVoltage(0);
}