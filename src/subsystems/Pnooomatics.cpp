#include "subsystems/Pnooomatics.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"

Pnooomatics::Pnooomatics(RobotBase* robot) : Subsystem(robot, this) {
	hang.set_value(hangReleased);
	clamp.set_value(clampEnabled);
}

void Pnooomatics::registerTasks() {
	auto controller = robot->getSubsystem<Controller>();

	if (!controller) { return; }

	auto controllerRef = controller.value();

	// do controller keybinds here
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