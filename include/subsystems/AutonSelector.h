#pragma once
#include "../pros/adi.hpp"
#include "Subsystem.h"
#include "lib/utils/CircularBuffer.h"

class AutonSelector : public Subsystem {
	struct Auton {
		const char* name;
		AutonFn_t autonFunc;
		bool isQual;
	};

	// pros::adi::Potentiometer pot = pros::adi::Potentiometer('F', pros::E_ADI_POT_V2);
	pros::adi::AnalogIn pot = pros::adi::AnalogIn('F');

	util::CircularBuffer<Auton> autons = util::CircularBuffer<Auton>(16);
	int autonIndex = 0;

	[[nodiscard]] double getPotValue() const;
	RobotThread thread();
public:
	struct flags {};

	explicit AutonSelector(RobotBase* robot);

	void registerTasks() override;

	void addAuton(const char* name, AutonFn_t autonFunc, bool isQual);
};