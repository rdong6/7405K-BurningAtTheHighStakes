#pragma once
#include "lib/geometry/Pose.h"
#include "lib/utils/Math.h"
#include <algorithm>
#include <stdexcept>
#include <vector>

// Time parameterized trajectory. Stores states containing pose, curvature, elapsed time, vel, accel at that point.
struct Trajectory {
	struct State {
		Pose pose;
		double vel;      // in/s
		double accel;    // in/s^2
		double curvature;// 1/r (1/in??)
		double t;        // seconds elapsed since start of trajectory

		constexpr bool operator==(const State&) const = default;

		[[nodiscard]] constexpr State interpolate(State endState, double i) const {
			const double newT = util::lerp(t, endState.t, i);
			const double dt = newT - t;// dt between new interpolated time and cur time

			// if delta time is neg, flip interpolation order
			if (dt < 0) { return endState.interpolate(*this, 1.0 - i); }

			const bool isReversing = vel < 0.0 || (std::abs(vel) < 1E-9 && accel < 0.0);

			// calcs new vel
			// v = v_0 + at
			const double newVel = vel + (accel * dt);

			// cals change in position
			// ∆pos = v_0*t + 0.5at²
			const double newS = (vel * dt + 0.5 * accel * dt * dt) * (isReversing ? -1.0 : 1.0);

			// interpolationFrac used to find new position, interpolating between the two endpoint poses. The fraction for
			// interpolation is calculated by change in position (∆s) divided by total dist between the 2 endpoints.
			const double interpolationFrac = newS / endState.pose.translation().distanceTo(pose.translation());

			return {.pose = pose.lerp(endState.pose, interpolationFrac),
			        .vel = newVel,
			        .accel = accel,
			        .curvature = util::lerp(curvature, endState.curvature, interpolationFrac),
			        .t = newT};
		}
	};

	Trajectory() = default;
	explicit Trajectory(std::vector<State>& states) : states(states) {
		if (this->states.empty()) { throw std::invalid_argument("Trajectory manually initialized w/ 0 states."); }

		totalTime = this->states.back().t;
	}

	explicit Trajectory(std::vector<State>&& states) : states(std::move(states)) {
		if (this->states.empty()) { throw std::invalid_argument("Trajectory manually initialized w/ 0 states."); }

		totalTime = this->states.back().t;
	}

	[[nodiscard]] State sample(double t) const {
		if (states.empty()) {
			// either throw error or something
			return State{};
		}

		// if t is outta bounds, return first/last point
		if (t <= states.front().t) { return states.front(); }
		if (t >= totalTime) { return states.back(); }

		// Use binary search to get the element with a timestamp no less than the
		// requested timestamp. This starts at 1 because we use the previous state
		// later on for interpolation.
		auto sample =
		        std::lower_bound(states.cbegin() + 1, states.cend(), t, [](const auto& a, const auto& b) { return a.t < b; });
		auto prevSample = sample - 1;


		// The sample's timestamp is now greater than or equal to the requested
		// timestamp. If it is greater, we need to interpolate between the
		// previous state and the current state to get the exact state that we
		// want.

		// if sample's timestamp == requested timestamp
		if (util::fpEquality(sample->t, t)) { return *sample; }

		// if difference in states is negligible, don't interpolate
		if (util::fpEquality(sample->t, prevSample->t)) { return *sample; }

		// interpolate between the 2 states to get the state we want
		return prevSample->interpolate(*sample, (t - prevSample->t) / (sample->t - prevSample->t));
	}

	// Transforms all poses in trajectory by the given transform.
	// Useful in transforming from robot-relative trajectory into field-relative trajectory.

	// Modifies THIS trajectory's states
	void transformByInternal(const Transform2D& transform) {
		// Calculate the first transformed pose
		const Pose firstPose = states[0].pose;// must be a copy as we modify states[0]'s pose
		const Pose transformedFirstPose = firstPose + transform;
		states[0].pose = transformedFirstPose;

		// Then transform each subsequent pose in the trajectory relative to the initial pose
		for (size_t i = 1; i < states.size(); i++) {
			auto& state = states[i];
			state.pose = transformedFirstPose + (state.pose - firstPose);
		}
	}

	// Leaves original trajectory untouched. Returns a new trajectory that's been transformed
	[[nodiscard]] Trajectory transformBy(const Transform2D& transform) const {
		// Calculate the first transformed pose
		auto newStates = states;
		const Pose& firstPose = states[0].pose;
		const Pose transformedFirstPose = firstPose + transform;
		newStates[0].pose = transformedFirstPose;

		// Then transform each subsequent pose in the trajectory relative to the initial pose
		for (size_t i = 1; i < newStates.size(); i++) {
			auto& state = newStates[i];
			state.pose = transformedFirstPose + (state.pose - firstPose);
		}

		return Trajectory(newStates);
	}

	[[nodiscard]] double getTotalTime() const { return totalTime; }

	bool operator==(const Trajectory&) const = default;

private:
	std::vector<State> states;
	double totalTime = 0;
};