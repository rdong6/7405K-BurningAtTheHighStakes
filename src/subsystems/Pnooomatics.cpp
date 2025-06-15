#include "subsystems/Pnooomatics.h"
#include "pros/misc.h"
#include "subsystems/Controller.h"

Pnooomatics::Pnooomatics(RobotBase* robot) : Subsystem(robot) {
	clamp.set_value(clampEnabled);
	rightHammer.set_value(rightHammerDeployed);
	leftHammer.set_value(leftHammerDeployed);
}

void Pnooomatics::registerTasks() {
	// robot->registerTask([this]() { return this->autoClampCoro(); }, TaskType::AUTON,
	//                     [robot = this->robot]() { return robot->getFlag<Pnooomatics>().value()->clampMogo; });

	auto controller = robot->getSubsystem<Controller>();

	if (!controller) { return; }

	auto controllerRef = controller.value();

	// do controller keybinds here
	controllerRef->registerCallback([this]() { toggleClamp(); }, []() {}, Controller::master, Controller::b,
	                                Controller::rising);

	controllerRef->registerCallback([this]() { toggleRightHammer(); }, []() {}, Controller::master, Controller::y,
	                                Controller::rising);

	controllerRef->registerCallback([this]() { toggleLeftHammer(); }, []() {}, Controller::master, Controller::right,
									Controller::rising);
}

RobotThread Pnooomatics::autoClampCoro() {
	while (true) {
		if (dist.get_distance() <= 40 /* TUNE THIS! */) {
			setClamp(true);
			robot->getFlag<Pnooomatics>().value()->clampMogo = false;
		}

		co_yield [robot = this->robot]() { return robot->getFlag<Pnooomatics>().value()->clampMogo; };
	}
}

void Pnooomatics::setClamp(bool enable) {
	clampEnabled = enable;
	clamp.set_value(enable);
	if (clampEnabled) { pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); }
	pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Clamp: %d", enable);
}

void Pnooomatics::toggleClamp() {
	setClamp(!clampEnabled);
}

void Pnooomatics::setRightHammer(bool enable) {
	rightHammerDeployed = enable;
	rightHammer.set_value(enable);
}

void Pnooomatics::toggleRightHammer() {
	setRightHammer(!rightHammerDeployed);
}

void Pnooomatics::setLeftHammer(bool enable) {
	leftHammerDeployed = enable;
	leftHammer.set_value(leftHammerDeployed);
}

void Pnooomatics::toggleLeftHammer() {
	setLeftHammer(!leftHammerDeployed);
}