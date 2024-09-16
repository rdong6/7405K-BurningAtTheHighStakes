#pragma once
#include "Subsystem.h"
#include "pros/adi.hpp"

class Pnooomatics : public Subsystem {
private:
	pros::adi::DigitalOut hang{'D'};
	pros::adi::DigitalOut clamp{'H'};

	bool hangReleased = false;
	bool clampEnabled = false;

public:
	struct flags {};
	
	explicit Pnooomatics(RobotBase* robot);

	void registerTasks() override;

	void setHang(bool release);
	void toggleHang();

	void setClamp(bool enable);
	void toggleClamp();
};