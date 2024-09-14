#pragma once
#include "Subsystem.h"
#include "pros/adi.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut hangRelease{'D'};
	pros::adi::DigitalOut clamp{'H'};

	bool hangReleased = false;
	bool clampEnabled = false;

public:
	explicit Pnooomatics(RobotBase* robot);

	void registerTasks() override;
};