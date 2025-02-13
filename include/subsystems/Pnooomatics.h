#pragma once
#include "Constants.h"
#include "Subsystem.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut hang{'0'};
	pros::adi::DigitalOut clamp{'D'};
	pros::adi::DigitalOut rightHammer{'B'};
	pros::adi::DigitalOut leftHammer{'E'};
	pros::adi::DigitalOut claw{'0'};
	pros::Distance dist{ports::clampDistance};

	bool hangReleased = false;
	bool clampEnabled = false;
	bool rightHammerDeployed = false;
	bool leftHammerDeployed = false;
	bool clawEnabled = false;

	RobotThread autoClampCoro();

public:
	struct flags {
		bool clampMogo = false;// to clamp mogo if dist sensor detects (turns off post clamp)
	};

	explicit Pnooomatics(RobotBase* robot);

	void registerTasks() override;

	void setHang(bool release);
	void toggleHang();

	void setClamp(bool enable);
	void toggleClamp();

	void setRightHammer(bool enable);
	void toggleRightHammer();

	void setLeftHammer(bool enable);
	void toggleLeftHammer();

	void setClaw(bool enable);
	void toggleClaw();
};