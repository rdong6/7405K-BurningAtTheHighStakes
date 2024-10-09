#include "subsystems/Pnooomatics.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"

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

	controllerRef->registerCallback([this]() { toggleHammer(); }, []() {}, Controller::master, Controller::down,
	                                Controller::rising);

	controllerRef->registerCallback([this]() { toggleHammer(); }, []() {}, Controller::master, Controller::a,
	                                Controller::rising);
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
}

void Pnooomatics::toggleClamp() {
	clampEnabled = !clampEnabled;
	clamp.set_value(clampEnabled);
}

void Pnooomatics::setHammer(bool enable) {
	hammerDeployed = enable;
	hammer.set_value(enable);
}

void Pnooomatics::toggleHammer() {
	hammerDeployed = !hammerDeployed;
	hammer.set_value(hammerDeployed);
}