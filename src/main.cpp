#include "main.h"
#include "AutonSelector.h"
#include "Constants.h"
#include "Robot.h"
#include "RobotBase.h"
#include "autons.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/DriveCharacterizationMotion.h"
#include "lib/motion/PIDTurn.h"
#include "lib/motion/ProfiledMotion.h"
#include "lib/motion/RAMSETEMotion.h"
#include "lib/motion/TimedMotion.h"
#include "lib/trajectory/constraints/CentripetalAccelerationConstraint.h"
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
#include "lib/trajectory/TrajectoryManager.h"

#include <lib/spline/CubicHermiteSpline.h>

RobotThread autonomousUser();

void robot_init() {
	robotInstance = new std::decay<decltype(*robotInstance)>::type();
	// move cur alliance into the autons -> while we figure a different way to do auton selector
	// robotInstance->curAlliance = Alliance::RED; // for skills

	robotInstance->curAlliance = Alliance::BLUE;
	robotInstance->registerTask([]() { return autonomousUser(); }, TaskType::AUTON);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	// AutonSelector autonSelector;// also initalizes pros::lcd

	// qual autons
	// autonSelector.addAuton("Ring Side", false, true, new AutonSelector::AutonFnStruct([]() {return blueRingSide();}));
	// autonSelector.addAuton("Ring Side", true, true, new AutonSelector::AutonFnStruct([]() {return redRingSide();}));
	// autonSelector.addAuton("Mogo Side", false, true, new AutonSelector::AutonFnStruct([]() {return blueMogoSide();}));
	// autonSelector.addAuton("Mogo Side", true, true, new AutonSelector::AutonFnStruct([]() {return redMogoSide();}));
	pros::lcd::initialize();
	robot_init();
	// autonSelector.run();

	// TODO: Fix logger so no unaligned accesses
	// logger_initialize("test.txt", 100);

	robotTask = pros::c::task_create(
	        [](void* robot) {
		        if (robot) { static_cast<decltype(robotInstance)>(robot)->run(); }
	        },
	        robotInstance, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Scheduler");
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

	drive->setCurrentMotion(PIDTurn(210, PID(150, 1, 50, true, 10), false, false));
	co_yield drive->waitUntilSettled(1000);

	co_yield util::coroutine::nextCycle();
}

RobotThread autonomousUser() {
	robotInstance->getSubsystem<Odometry>().value()->reset();

	// for skills
	// auto skillsCoro = skillsAuton();
	// while (skillsCoro) {
	// 	co_yield skillsCoro();
	// }

	auto blueMogoSideCoro = blueMogoSide();
	while (blueMogoSideCoro) {co_yield blueMogoSideCoro(); }

	// for now defunct auton selector
	// if (robotInstance->autonFnPtr) {
	// 	auto coro = robotInstance->autonFnPtr();
	// 	while (coro) { co_yield coro(); }
	// }

	// auto coro = redRingSide();
	// auto coro = blueMogoSide();
	// auto coro = redRingSide();
	// auto coro = redMogoSide();
	// auto coro = testAuton();
	// auto coro = skillsAuton();
	// while (coro) { co_yield coro(); }

	co_yield util::coroutine::nextCycle();
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