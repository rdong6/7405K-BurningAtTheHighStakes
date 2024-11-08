#pragma once

#include "Pose.h"

class kinState {
public:
	struct substate {
		double x = 0;
		double y = 0;
		double theta = 0;
	};

private:
	substate velo_state{}, accel_state{};

public:
	kinState() = default;
	kinState(Pose pos, substate velo, substate accel) : position(pos), velo_state(velo), accel_state(accel){};

	Pose position;
	substate velocity() const;
	substate acceleration() const;

	void setVelocity(double _x, double _y, double _h);
	void setAcceleration(double _x, double _y, double _h);
};
