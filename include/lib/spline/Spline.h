#pragma once
#include "lib/geometry/Pose.h"
#include <any>
#include <type_traits>
#include <utility>
#include <vector>

using PoseWithCurvature = std::pair<Pose, double>;

class ISpline {
public:
	constexpr ISpline() = default;

	constexpr ISpline(const ISpline&) = default;
	constexpr ISpline& operator=(const ISpline&) = default;

	constexpr ISpline(ISpline&&) = default;
	constexpr ISpline& operator=(ISpline&&) = default;

	constexpr virtual ~ISpline() = default;

	[[deprecated("Use Spline::getPointWithCurvature(double t) instead")]] [[nodiscard]] virtual Pose
	getPoint(double t) const = 0;
	[[nodiscard]] virtual PoseWithCurvature getPointWithCurvature(double t) const = 0;
	[[nodiscard]] std::vector<PoseWithCurvature> parameterize(double t0 = 0.0, double t1 = 1.0) const;

protected:
	[[nodiscard]] double getCurvature(double dx, double dy, double ddx, double ddy) const;
	[[nodiscard]] double getCurvature(const Pose& firstDeriv, const Pose& secondDeriv) const;

private:
	// Constraints for spline parameterization. (max change from last point)
	static constexpr double kMaxDx = 0.25;
	static constexpr double kMaxDy = 0.01;
	// static  constexpr double kMaxDtheta = 0.05235989251996422;
	static constexpr double kMaxDtheta = 0.5;


	/**
	 * A malformed spline does not actually explode the LIFO stack size. Instead,
	 * the stack size stays at a relatively small number (e.g. 30) and never
	 * decreases. Because of this, we must count iterations. Even long, complex
	 * paths don't usually go over 300 iterations, so hitting this maximum should
	 * definitely indicate something has gone wrong.
	 */
	static constexpr int kMaxIterations = 10000;
};

template<class T>
concept SplineType = std::is_base_of<ISpline, typename std::remove_reference<T>::type>::value;

// Encapsulates the ISpline class, allowing us to use polymorphism without the need for dynamic allocation
// Some very cursed stuff that I'm pretty sure isn't good practice, but I don't want to template everything or deal w/ dynamic
// memory for polymorphism
class Spline {
private:
	std::any storage;
	ISpline& (*getter)(std::any&);

public:
	template<SplineType ConcreteSpline>
	Spline(ConcreteSpline&& motion)
	    : storage(std::forward<ConcreteSpline>(motion)),
	      getter{[](std::any& storage) -> ISpline& { return std::any_cast<ConcreteSpline&>(storage); }} {}

	ISpline* operator->() { return std::addressof(getter(storage)); }
};