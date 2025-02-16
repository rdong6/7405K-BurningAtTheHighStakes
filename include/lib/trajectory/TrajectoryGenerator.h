#pragma once
#include "Trajectory.h"
#include "lib/spline/Path.h"
#include "lib/trajectory/constraints/TrajectoryConstraint.h"
#include <cmath>
#include <stdexcept>
#include <vector>

/*
Steps:
1) generate points from spline (parameterize spline) -> parameterize until dx, dy, d𝛉
2) time parameterize our spline
    - accel forwards pass
    - accel backwards pass
    - combine & integrate, solving for dt along path as well
*/
class TrajectoryGenerator {
public:
	TrajectoryGenerator() = default;
	explicit TrajectoryGenerator(std::vector<TrajectoryConstraint> constraints) : constraints(std::move(constraints)) {}

	[[nodiscard]] Trajectory generate(const Path& path) const;

private:
	// doesn't include PoseWithCurvature -> get that from the path (same index)
	struct ConstrainedState {
		double dist;  // dist since start
		double maxVel;// cur vel
		double maxAccel;
		double minAccel;
	};

	void enforceAccelLimit(ConstrainedState& state, const Pose& pose, double curvature, bool reverse) const;

	std::vector<TrajectoryConstraint> constraints;
};

// Trajectory generation in this library will default to using this one
// unless a user needs different constraints. Then create their own trajectory generator
[[deprecated("TODO: Clean up this implementation")]] extern TrajectoryGenerator defaultTrajectoryGenerator;