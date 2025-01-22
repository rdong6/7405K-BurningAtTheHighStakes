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

	[[nodiscard]] Trajectory generate(const Path& path, double startVel, double endVel, double maxVel, double maxAcc, bool reversed) const {
		const std::vector<PoseWithCurvature>& points = path.getPoints();

		std::vector<ConstrainedState> constrainedStates(points.size());
		ConstrainedState predecessor{0, startVel, maxAcc, -maxAcc};
		constrainedStates[0] = predecessor;

		// Forward pass
		for (size_t i = 0; i < points.size(); i++) {
			auto& constrainedState = constrainedStates[i];

			// Begin constraining based on predecessor
			double ds =
			        points[i].first.translation().distanceTo(points[i == 0 ? 0 : i - 1].first.translation());// change in pos
			constrainedState.dist = ds + predecessor.dist;

			// loop as we may need to iterate through all previous points to find max end vel + common acceleration, because
			// acceleration may be function of velocity
			while (true) {
				// Enforce global max velocity and max reachable velocity by global
				// acceleration limit. v_f = √(v_i² + 2ad).

				// first constrain to max theoretical velocity before applying constraints
				constrainedState.maxVel =
				        std::fmin(vsqrtd(predecessor.maxVel * predecessor.maxVel + 2.0 * predecessor.maxAccel * ds), maxVel);
				constrainedState.minAccel = -maxAcc;
				constrainedState.maxAccel = maxAcc;


				// Now go through all custom-defined user constraints (ex: differential drive kinematics, wheel slip,
				// centripetal acceleration constraints)
				for (const auto& constraint : constraints) {
					constrainedState.maxVel =
					        std::fmin(constrainedState.maxVel,
					                  constraint->getMaxVel(points[i].first, points[i].second, constrainedState.maxVel));
				}


				// enforce acceleration limits @ this point
				enforceAccelLimit(constrainedState, points[i].first, points[i].second, reversed);

				if (ds < std::numeric_limits<double>::epsilon()) { break; }

				// If actual acceleration for this state is higher than the constrained max accel, then we need to reduce max
				// accel of the predecessor and try again.
				// Actual accel from rearranging v_f = √(v_i² + 2ad)
				double actualAccel =
				        (constrainedState.maxVel * constrainedState.maxVel - predecessor.maxVel * predecessor.maxVel) /
				        (2.0 * ds);

				// we violate max accel constraint. modify predecessor
				if (actualAccel - 1E-6 > constrainedState.maxAccel) {
					predecessor.maxAccel = constrainedState.maxAccel;
				} else {
					// constrains predecessor's max accel to the current accel (so we can actually reach current state's vel)
					if (actualAccel > predecessor.minAccel + 1E-6) { predecessor.maxAccel = actualAccel; }

					// if actual accel is less than predecessor's min accel, it'll be repaired in backward pass.
					break;
				}
			}

			predecessor = constrainedState;
		}

		// Backward pass
		ConstrainedState successor{constrainedStates.back().dist, endVel, maxAcc, -maxAcc};
		for (int i = constrainedStates.size() - 1; i >= 0; i--) {
			auto& constrainedState = constrainedStates[i];

			double ds = constrainedState.dist - successor.dist;// negative

			while (true) {
				// Enforce max velocity limit (reverse)
				// v_f = √(v_i² + 2ad), where v_i = successor.
				double newMaxVel = vsqrtd(successor.maxVel * successor.maxVel + 2.0 * successor.minAccel * ds);

				// no more limits to impose
				if (newMaxVel >= constrainedState.maxVel) { break; }

				constrainedState.maxVel = newMaxVel;

				// ensure accel is within constraints w/ new max vel
				enforceAccelLimit(constrainedState, points[i].first, points[i].second, reversed);
				if (ds > -std::numeric_limits<double>::epsilon()) { break; }

				// If the actual acceleration for this state is lower than the min
				// acceleration, then we need to lower the min acceleration of the
				// successor and try again.
				double actualAccel =
				        (constrainedState.maxVel * constrainedState.maxVel - successor.maxVel * successor.maxVel) / (2.0 * ds);
				if (constrainedState.minAccel > actualAccel + 1E-6) {
					successor.minAccel = constrainedState.minAccel;
				} else {
					successor.minAccel = actualAccel;
					break;
				}
			}

			successor = constrainedState;
		}

		// Integrate forwards to get finalized data
		std::vector<Trajectory::State> trajectoryStates(points.size());
		double t = 0.0;// 0s
		double s = 0.0;// 0in
		double v = 0.0;// 0m/s

		for (unsigned int i = 0; i < constrainedStates.size(); i++) {
			const auto& state = constrainedStates[i];

			// calculate change between current state and previous state
			double ds = state.dist - s;

			// calc accel between current state & previous state
			double accel = (state.maxVel * state.maxVel - v * v) / (2.0 * ds);

			double dt = 0.0;
			if (i > 0) {
				trajectoryStates.at(i - 1).accel = reversed ? -accel : accel;

				// calculate dt using either ∆v or ∆s
				if (!util::fpEquality(accel, 0.0)) {
					// v_f = v_0 + at
					dt = (state.maxVel - v) / accel;
				} else {
					// ∆s = v∆t
					// so ∆t = ∆s/v
					dt = ds / v;
				}
			}

			v = state.maxVel;
			s = state.dist;
			t += dt;

			trajectoryStates[i] = {.pose = points[i].first,
			                       .vel = reversed ? -v : v,
			                       .accel = reversed ? -accel : accel,
			                       .curvature = points[i].second,
			                       .t = t};

			// printf("t: %f\ts: %f\tv: %f\ta: %f\n", t, s, v, accel);
			printf("%f %f %f %f %f\n", t, points[i].first.X(), points[i].first.Y(), v, accel);
		}

		printf("Total Length: %f\n", constrainedStates.back().dist);

		return Trajectory(std::move(trajectoryStates));
	}

private:
	// doesn't include PoseWithCurvature -> get that from the path (same index)
	struct ConstrainedState {
		double dist;  // dist since start
		double maxVel;// cur vel
		double maxAccel;
		double minAccel;
	};

	void enforceAccelLimit(ConstrainedState& state, const Pose& pose, double curvature, bool reverse) const {
		for (auto& constraint : constraints) {
			double factor = reverse ? -1.0 : 1.0;

			ITrajectoryConstraint::Acceleration minMaxAccel =
			        constraint->getMinMaxAcceleration(pose, curvature, state.maxVel * factor);

			if (minMaxAccel.min > minMaxAccel.max) {
				throw std::runtime_error("There was infeasible trajectory constraint. Min acceleration was calculated to be "
				                         "greater than max acceleration.");
			}

			state.minAccel = std::fmax(state.minAccel, reverse ? -minMaxAccel.max : minMaxAccel.min);
			state.maxAccel = std::fmin(state.maxAccel, reverse ? -minMaxAccel.min : minMaxAccel.max);
		}
	}

	std::vector<TrajectoryConstraint> constraints;
};