#pragma once
#include "Constants.h"
#include "Subsystem.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut hang{'0'};
	pros::adi::DigitalOut clamp{'D'};
	pros::adi::DigitalOut hammer{'B'};
	pros::adi::DigitalOut claw{'0'};
	pros::Distance dist{ports::clampDistance};

	bool hangReleased = false;
	bool clampEnabled = false;
	bool hammerDeployed = false;
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

	void setHammer(bool enable);
	void toggleHammer();

	void setClaw(bool enable);
	void toggleClaw();
};