#pragma once
#include "Constants.h"
#include "Subsystem.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut clamp{'H'};
	pros::adi::DigitalOut rightHammer{'G'};
	pros::adi::DigitalOut leftHammer{'F'};
	pros::Distance dist{ports::clampDistance};

	bool clampEnabled = false;
	bool rightHammerDeployed = false;
	bool leftHammerDeployed = false;

	RobotThread autoClampCoro();

public:
	struct flags {
		bool clampMogo = false;// to clamp mogo if dist sensor detects (turns off post clamp)
	};

	explicit Pnooomatics(RobotBase* robot);

	void registerTasks() override;

	void setClamp(bool enable);
	void toggleClamp();

	void setRightHammer(bool enable);
	void toggleRightHammer();

	void setLeftHammer(bool enable);
	void toggleLeftHammer();
};