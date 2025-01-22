#pragma once
#include "lib/geometry/Pose.h"
#include <any>
#include <limits>
#include <type_traits>

class ITrajectoryConstraint {
public:
	// represents min & max acceleration
	struct Acceleration {
		double min = -std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::max();
	};

	constexpr ITrajectoryConstraint() = default;
	constexpr ITrajectoryConstraint(const ITrajectoryConstraint&) = default;
	constexpr ITrajectoryConstraint& operator=(const ITrajectoryConstraint&) = default;

	constexpr ITrajectoryConstraint(ITrajectoryConstraint&&) = default;
	constexpr ITrajectoryConstraint& operator=(ITrajectoryConstraint&&) = default;

	constexpr virtual ~ITrajectoryConstraint() = default;

	// pose = cur pose @ point in trajectory
	// curvature = cur curvature @ point in trajectory
	// vel = current velocity @ point in trajectory before constraints applied
	//
	// returns absolute max vel
	[[nodiscard]] constexpr virtual double getMaxVel(const Pose& pose, double curvature, double vel) const = 0;

	[[nodiscard]] constexpr virtual Acceleration getMinMaxAcceleration(const Pose& pose, double curvature,
	                                                                   double vel) const = 0;
};

template<class T>
concept TrajectoryConstraintType = std::is_base_of<ITrajectoryConstraint, T>::value;

// Trick used in Motion.h to allow for usage of polymorphism while bypassing need for using pointers
// So I can do std::vector<TrajectoryConstraint> instead of a vector of pointers
class TrajectoryConstraint {
public:
	template<TrajectoryConstraintType ConcreteTrajectoryConstraint>
	TrajectoryConstraint(ConcreteTrajectoryConstraint&& constraint)
	    : storage(std::forward<ConcreteTrajectoryConstraint>(constraint)),
	      getter{[](std::any& storage) -> ITrajectoryConstraint& {
		      return std::any_cast<ConcreteTrajectoryConstraint&>(storage);
	      }} {}

	ITrajectoryConstraint* operator->() const { return std::addressof(getter(storage)); }

private:
	mutable std::any storage;
	ITrajectoryConstraint& (*getter)(std::any&);
};