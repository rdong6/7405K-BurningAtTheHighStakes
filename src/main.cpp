#include "main.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/utils/CoroutineGenerator.h"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.h"
#include "subsystems/Drive.h"
#include <type_traits>

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
	pros::lcd::initialize();
	robot_init();
}

// USE THIS FUNC FOR AUTON CODING!
// thread runs after all other threads run
RobotThread autonomousUser() {
	auto driveOpt = robotInstance->getSubsystem<Drive>();

	auto driveRef = driveOpt.value();
}


// !!!!!!!!!!!!!!!!!!!!!
// DON'T USE THESE FUNCS
// !!!!!!!!!!!!!!!!!!!!!

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {}