#include "subsystems/Pnooomatics.h"
#include "lib/utils/DelayedBool.h"
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

	controllerRef->registerCallback([this]() { toggleHammer(); }, []() {}, Controller::master, Controller::a,
	                                Controller::rising);

	controllerRef->registerCallback([this]() { toggleClaw(); }, []() {}, Controller::master, Controller::y,
	                                Controller::rising);

	robot->registerTask([this]() { return this->runner(); }, TaskType::AUTON);
}

RobotThread Pnooomatics::runner() {

	util::DelayedBool enableClamp;

	while (true) {
		if (dist.get_distance() <= 40 && robot->getFlag<Pnooomatics>().value()->clampMogo) {
			enableClamp = util::DelayedBool(50);
			robot->getFlag<Pnooomatics>().value()->clampMogo = false;
			setClamp(true);
		}

		// if (enableClamp()) {
		// 	setClamp(true);
		// 	enableClamp = util::DelayedBool();
		// }

		co_yield util::coroutine::nextCycle();
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

void Pnooomatics::setClaw(bool enable) {
	clawEnabled = enable;
	claw.set_value(enable);
}

void Pnooomatics::toggleClaw() {
	clawEnabled = !clawEnabled;
	claw.set_value(clawEnabled);
}
