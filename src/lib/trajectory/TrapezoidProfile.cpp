#include "lib/trajectory/TrapezoidProfile.h"
#include "lib/utils/Math.h"
#include <cmath>
#include <cstdio>

TrapezoidProfile::TrapezoidProfile()
    : start_vel(0), dist(0), acc_max(0), dec_max(0), v_max(0), t_acc(0), t_coast(0), t_dec(0), d_acc(0), d_coast(0), d_dec(0),
      direction(0) {}

TrapezoidProfile::TrapezoidProfile(double distance, double max_acceleration, double max_deceleration, double max_velocity,
                                   double start_vel) {
	this->start_vel = start_vel;
	direction = distance < 0 ? -1 : 1;
	dist = std::abs(distance);
	acc_max = max_acceleration;
	dec_max = max_deceleration;
	v_max = fmin(fmin(vsqrtd(acc_max * dist) + start_vel * start_vel, vsqrtd(dec_max * dist)), max_velocity);
	printf("[Trapezoid Profile] Max Vel: %f\n", v_max);

	// generate variables

	// calculate ramp up and ramp down times
	t_acc = v_max / acc_max;
	t_dec = v_max / dec_max;

	// calculate the distance traveled along those ramp up times (∆x = v_0 * t +
	// 0.5 * a * t^2).. v_0 is 0 for both NOTE - dec_max is a positive
	// quantity... idk why i did that, but thats how it works. Thats why v_0 is
	// 0 for both d_acc and d_dec calculation even though technically d_dec
	// starts at maxvel and goes to 0.
	d_acc = start_vel * t_acc + 0.5 * acc_max * pow(t_acc, 2.0);
	d_dec = 0.5 * dec_max * pow(t_dec, 2.0);
	d_coast = dist - d_acc - d_dec;

	t_coast = d_coast / v_max;
}

// can get rid of these three functions and combine them into one func
// and just have 4 branches

// Returns the acceleration at a certain point of time in the trapezoid profile
double TrapezoidProfile::acceleration(double t) const {
	if (t <= t_acc) {
		return acc_max;
	} else if (t >= (t_acc + t_coast) && t <= (t_acc + t_coast + t_dec)) {
		return -dec_max;
	} else {
		return 0.0;
	}
}

// Returns the velocity at a certain point of time in the trapezoid profile
double TrapezoidProfile::velocity(double t) const {
	if (t <= t_acc) {
		return start_vel + acc_max * t;
	} else if (t <= (t_acc + t_coast)) {
		return v_max;
	} else if (t <= (t_acc + t_coast + t_dec)) {
		return -1 * dec_max * (t - (t_acc + t_coast)) + v_max;
	} else {
		return 0.0;
	}
}

// Returns the position  at a certain point of time in the trapezoid profile
double TrapezoidProfile::position(double t) const {
	if (t <= t_acc) {
		return start_vel * t + 0.5 * acc_max * pow(t, 2.0);
	} else if (t <= (t_acc + t_coast)) {
		return (start_vel + t_acc * acc_max * 0.5) * t_acc + v_max * (t - t_acc);
		// return v_max * t - position(t_acc);
	} else if (t <= (t_acc + t_coast + t_dec)) {
		return -0.5 * dec_max * pow(t - t_acc - t_coast - t_dec, 2.0) + position(t_acc + t_coast) + d_dec;
	} else {
		return dist;
	}
}

TrapezoidProfile::State TrapezoidProfile::getState(double t) const {
	return {position(t) * direction, velocity(t) * direction, acceleration(t) * direction};
}

double TrapezoidProfile::getTargetDist() const {
	return dist;
}

double TrapezoidProfile::getTotalTime() const {
	return t_acc + t_coast + t_dec;
}