#include "Robot.h"
#include "autons.h"
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
	drive->setCurrentMotion(ProfiledMotion(49, 60, 100, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
    co_yield util::coroutine::delay(600);
    pnoomatics->setLeftHammer(true);
	co_yield drive->waitUntilSettled(2000);

	// original heading: 30
    drive->setCurrentMotion(PIDTurn(40, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);

	// move back first to get mogo
	drive->setCurrentMotion(ProfiledMotion(-28, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
    intake->setDistStop(false);

	drive->setCurrentMotion(PIDTurn(73, PID(120, 1, 170, true, 10), false, false, 0.5));
    co_yield util::coroutine::delay(200);
    pnoomatics->setLeftHammer(false);
	co_yield drive->waitUntilSettled(500);

    intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 50));
	co_yield drive->waitUntilSettled(1100);

	co_yield util::coroutine::delay(800);
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

    drive->setCurrentMotion(PIDTurn(245, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

    drive->setCurrentMotion(ProfiledMotion(39, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);

	drive->setCurrentMotion(PIDTurn(235, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	pnoomatics->setRightHammer(true);
	intake->moveVoltage(-12000);

	drive->setCurrentMotion(ProfiledMotion(30, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);


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
	drive->setCurrentMotion(ProfiledMotion(49, 60, 100, 100));
    intake->moveVoltage(12000);
    intake->setDistStop(true);
    co_yield util::coroutine::delay(600);
    pnoomatics->setRightHammer(true);
	co_yield drive->waitUntilSettled(2000);

	// original heading: 30
    drive->setCurrentMotion(PIDTurn(320, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(500);

	// move back first to get mogo
	drive->setCurrentMotion(ProfiledMotion(-28, 50, 100, 60));
	co_yield drive->waitUntilSettled(1500);
	pnoomatics->setClamp(true);
    intake->setDistStop(false);

	drive->setCurrentMotion(PIDTurn(287, PID(120, 1, 170, true, 10), false, false, 0.5));
    co_yield util::coroutine::delay(200);
    pnoomatics->setRightHammer(false);
	co_yield drive->waitUntilSettled(500);

    intake->moveVoltage(12000);
	drive->setCurrentMotion(ProfiledMotion(40, 40, 60, 50));
	co_yield drive->waitUntilSettled(1100);

	co_yield util::coroutine::delay(800);
    //if we want this just is for five ring everything after is for six ring

    drive->setCurrentMotion(PIDTurn(224, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(700);

    drive->setCurrentMotion(ProfiledMotion(44, 50, 100, 80));
	co_yield drive->waitUntilSettled(1500);

    drive->setCurrentMotion(ProfiledMotion(10, 50, 100, 100));
	co_yield drive->waitUntilSettled(500);

    drive->setCurrentMotion(ProfiledMotion(-7, 50, 100, 100));
	co_yield drive->waitUntilSettled(700);

    intake->moveVoltage(12000);

    drive->setCurrentMotion(PIDTurn(115, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

    drive->setCurrentMotion(ProfiledMotion(39, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);

	drive->setCurrentMotion(PIDTurn(125, PID(120, 1, 170, true, 10), false, false, 0.5));
	co_yield drive->waitUntilSettled(1000);

	pnoomatics->setLeftHammer(true);
	intake->moveVoltage(-12000);

	drive->setCurrentMotion(ProfiledMotion(30, 50, 100, 120));
	co_yield drive->waitUntilSettled(2500);




}