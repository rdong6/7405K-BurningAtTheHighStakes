#include "subsystems/Pnooomatics.h"
#include "pros/misc.h"
#include "subsystems/Controller.h"

Pnooomatics::Pnooomatics(RobotBase* robot) : Subsystem(robot) {
	hang.set_value(hangReleased);
	clamp.set_value(clampEnabled);
}

void Pnooomatics::registerTasks() {
	auto controller = robot->getSubsystem<Controller>();

	if (!controller) { return; }

	auto controllerRef = controller.value();

	// do controller keybinds here
	controllerRef->registerCallback([this]() { toggleClamp(); }, []() {}, Controller::master, Controller::b,
	                                Controller::rising);

	controllerRef->registerCallback([this]() { toggleHammer(); }, []() {}, Controller::master, Controller::y,
	                                Controller::rising);

	// controllerRef->registerCallback([this]() { toggleClaw(); }, []() {}, Controller::master, Controller::y,
	// Controller::rising);

	robot->registerTask([this]() { return this->autoClampCoro(); }, TaskType::AUTON,
	                    [robot = this->robot]() { return robot->getFlag<Pnooomatics>().value()->clampMogo; });
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

void Pnooomatics::setHang(bool release) {
	hangReleased = release;
	hang.set_value(release);
}

void Pnooomatics::toggleHang() {
	hangReleased = !hangReleased;
	hang.set_value(hangReleased);
}

void Pnooomatics::setClamp(bool enable) {
	clampEnabled = enable;
	clamp.set_value(enable);
	if (clampEnabled) { pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); }
	pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Clamp: %d", clampEnabled);
}

void Pnooomatics::toggleClamp() {
	setClamp(!clampEnabled);
}

void Pnooomatics::setHammer(bool enable) {
	hammerDeployed = enable;
	hammer.set_value(enable);
}

void Pnooomatics::toggleHammer() {
	hammerDeployed = !hammerDeployed;
	hammer.set_value(hammerDeployed);
}

void Pnooomatics::setClaw(bool enable) {
	clawEnabled = enable;
	claw.set_value(enable);
}

void Pnooomatics::toggleClaw() {
	clawEnabled = !clawEnabled;
	claw.set_value(clawEnabled);
}
