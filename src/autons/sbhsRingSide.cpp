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
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"


RobotThread sbhsRedRingSide() {
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
	drive->setCurrentMotion(ProfiledMotion(49, 60, 100, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
    co_yield util::coroutine::delay(600);
    pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(2000);

    drive->setCurrentMotion(PIDTurn(30, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);

	// move back first to get mogo
	drive->setCurrentMotion(ProfiledMotion(-28, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
    intake->setDistStop(false);

	drive->setCurrentMotion(PIDTurn(63, PID(120, 1, 170, true, 10), false, false, 0.5));
    co_yield util::coroutine::delay(150);
    pnoomatics->setLeftHammer(false);
	co_yield drive->waitUntilSettled(500);

    intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(28, 50, 60, 60));
	co_yield drive->waitUntilSettled(1100);

    co_yield util::coroutine::delay(200);
    //if we want this just is for five ring everything after is for six ring

    drive->setCurrentMotion(PIDTurn(136, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);

    drive->setCurrentMotion(ProfiledMotion(44, 50, 100, 40));
	co_yield drive->waitUntilSettled(1500);
    
    drive->setCurrentMotion(ProfiledMotion(10, 50, 100, 100));
	co_yield drive->waitUntilSettled(500);

    drive->setCurrentMotion(ProfiledMotion(-7, 50, 100, 100));
	co_yield drive->waitUntilSettled(700);

    intake->moveVoltage(12000);

    drive->setCurrentMotion(PIDTurn(250, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

    drive->setCurrentMotion(ProfiledMotion(35, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);

    drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 100));
	co_yield drive->waitUntilSettled(1500);

    co_yield util::coroutine::delay(100);

    drive->setCurrentMotion(ProfiledMotion(18, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
     co_yield util::coroutine::delay(300);

    drive->setCurrentMotion(PIDTurn(0, PID(150, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(600);

    liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(300);

    drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);


    

    // five ring code

    // drive->setCurrentMotion(PIDTurn(197, PID(120, 1, 170, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(800);

    // drive->setCurrentMotion(ProfiledMotion(32, 50, 100, 60));
	// co_yield drive->waitUntilSettled(1500);

    // drive->setCurrentMotion(PIDTurn(256, PID(120, 1, 170, true, 10), false, false, 0.5));
	// co_yield drive->waitUntilSettled(700);

    // drive->setCurrentMotion(ProfiledMotion(17, 50, 100, 60));
	// co_yield drive->waitUntilSettled(1500);

    // drive->setCurrentMotion(ProfiledMotion(10, 50, 100, 60));
	// co_yield drive->waitUntilSettled(1500);
	// // written until here

}

RobotThread sbhsBlueRingSide() {
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
	drive->setCurrentMotion(ProfiledMotion(49, 60, 100, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
    co_yield util::coroutine::delay(600);
    pnoomatics->setRightHammer(true);
	co_yield drive->waitUntilSettled(2000);

    drive->setCurrentMotion(PIDTurn(330, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);

	// move back first to get mogo
	drive->setCurrentMotion(ProfiledMotion(-28, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
    intake->setDistStop(false);

	drive->setCurrentMotion(PIDTurn(297, PID(120, 1, 170, true, 10), false, false, 0.5));
    co_yield util::coroutine::delay(150);
    pnoomatics->setRightHammer(false);
	co_yield drive->waitUntilSettled(500);

    intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(28, 50, 60, 60));
	co_yield drive->waitUntilSettled(1100);

    co_yield util::coroutine::delay(200);
    //if we want this just is for five ring everything after is for six ring

    drive->setCurrentMotion(PIDTurn(224, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);

    drive->setCurrentMotion(ProfiledMotion(44, 50, 100, 40));
	co_yield drive->waitUntilSettled(1500);
    
    drive->setCurrentMotion(ProfiledMotion(10, 50, 100, 100));
	co_yield drive->waitUntilSettled(500);

    drive->setCurrentMotion(ProfiledMotion(-7, 50, 100, 100));
	co_yield drive->waitUntilSettled(700);

    intake->moveVoltage(12000);

    drive->setCurrentMotion(PIDTurn(110, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

    drive->setCurrentMotion(ProfiledMotion(35, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);

    drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 100));
	co_yield drive->waitUntilSettled(1500);

    co_yield util::coroutine::delay(100);

    drive->setCurrentMotion(ProfiledMotion(18, 50, 60, 60));
	co_yield drive->waitUntilSettled(1500);
     co_yield util::coroutine::delay(300);

    drive->setCurrentMotion(PIDTurn(0, PID(150, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(600);

    liftFlags->targetAngle = 170;
	lift->setState(Lift::HOLD);
	co_yield drive->waitUntilSettled(300);

    drive->setCurrentMotion(ProfiledMotion(15, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);
}