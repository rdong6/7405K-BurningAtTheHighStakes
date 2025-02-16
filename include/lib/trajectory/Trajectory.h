#pragma once
#include "lib/geometry/Pose.h"
#include "lib/utils/Math.h"
#include <algorithm>
#include <stdexcept>
#include <vector>

// TODO: move this into Trajectory.cpp
// Time parameterized trajectory. Stores states containing pose, curvature, elapsed time, vel, accel at that point.
struct Trajectory {
	struct State {
		Pose pose;
		double vel;      // in/s
		double accel;    // in/s^2
		double curvature;// 1/r (1/in??)
		double t;        // seconds elapsed since start of trajectory

		constexpr bool operator==(const State&) const = default;

		[[nodiscard]] constexpr State interpolate(State endState, double i) const;
	};

	Trajectory() = default;
	explicit Trajectory(std::vector<State> states);

	[[nodiscard]] State sample(double t) const;

	// Transforms all poses in trajectory by the given transform.
	// Useful in transforming from robot-relative trajectory into field-relative trajectory.

	// Modifies THIS trajectory's states
	void transformByInternal(const Transform2D& transform);

	// Leaves original trajectory untouched. Returns a new trajectory that's been transformed
	[[nodiscard]] Trajectory transformBy(const Transform2D& transform) const;

	[[nodiscard]] double getTotalTime() const;

	void dumpTrajectory(const char* filename) const;

	bool operator==(const Trajectory&) const = default;

private:
	std::vector<State> states;
	double totalTime = 0;
};