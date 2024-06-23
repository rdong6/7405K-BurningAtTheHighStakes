#pragma once

class SCurveProfile {
public:
	struct State {
		double pos;
		double vel;
		double accel;
	};

public:
	SCurveProfile(double dist, double max_velocity, double max_acceleration, double time_to_max_accel);

	// t = seconds since start of profile
	State getState(double t);
};