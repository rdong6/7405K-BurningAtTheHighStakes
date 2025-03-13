#pragma once
#include "../pros/adi.hpp"
#include "Subsystem.h"

class AutonSelector : public Subsystem {
	// pros::adi::Potentiometer pot = pros::adi::Potentiometer('F', pros::E_ADI_POT_V2);
	pros::adi::AnalogIn pot = pros::adi::AnalogIn('F');

	[[nodiscard]] double getPotValue() const;
	RobotThread thread();
public:
	struct flags {};

	explicit AutonSelector(RobotBase* robot);

	void registerTasks() override;
};