#pragma once

class TrapezoidProfile {
public:
	struct State {
		double pos, vel, acc;
	};

private:
	double start_vel;
	double dist, acc_max, dec_max, v_max;
	double t_acc, t_coast, t_dec;
	double d_acc, d_coast, d_dec;

	int direction;

	double acceleration(double t) const;
	double velocity(double t) const;
	double position(double t) const;

public:
	TrapezoidProfile();
	TrapezoidProfile(double distance, double max_acceleration, double max_deceleration, double max_velocity,
	                 double start_velocity = 0);

	// time in seconds (0 = start of profile)
	State getState(double t) const;

	double getTargetDist() const;
	double getTotalTime() const;
};