#pragma once
#include "Constants.h"
#include "Subsystem.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut hang{'D'};
	pros::adi::DigitalOut clamp{'H'};
	pros::adi::DigitalOut hammer{'E'};
	pros::Distance dist{ports::clampDistance};

	bool hangReleased = false;
	bool clampEnabled = false;
	bool hammerDeployed = false;

	RobotThread runner();

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
};