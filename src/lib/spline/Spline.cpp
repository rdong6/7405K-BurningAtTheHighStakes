#include "lib/spline/Spline.h"
#include "lib/utils/Math.h"
#include <cstdio>
#include <stack>
#include <stdexcept>

namespace {
	struct StackContents {
		double t0;
		double t1;
	};
}// namespace

double ISpline::getCurvature(double dx, double dy, double ddx, double ddy) const {
	double denominator = std::pow(dx * dx + dy * dy, 1.5);
	double k = (dx * dy - dy * dx) / denominator;

	if (std::isnan(k)) {
		return 0.0;// idk what to do w/ a NaN curvature
	}

	return -k;
}


double ISpline::getCurvature(const Pose& firstDeriv, const Pose& secondDeriv) const {
	return getCurvature(firstDeriv.X(), firstDeriv.Y(), secondDeriv.X(), secondDeriv.Y());

	/*double denominator = std::pow(firstDeriv.X() * firstDeriv.X() + firstDeriv.Y() * firstDeriv.Y(), 1.5);
	double k = (secondDeriv.X() * firstDeriv.Y() - secondDeriv.Y() * firstDeriv.X()) / denominator;

	if (std::isnan(k)) {
	    return 0.0;// idk what to do w/ a NaN curvature
	}

	return -k;*/
}

// Paramaterizes by subdividing a spline until change between points is under a threshold
std::vector<PoseWithCurvature> ISpline::parameterize(double t0, double t1) const {
	std::vector<PoseWithCurvature> splinePoints;

	// add initial point as paramaterization doesn't do it
	splinePoints.push_back(getPointWithCurvature(0));

	// instead of recursively calling to subdivide the spline
	// allows us to if it's a malformed spline and we can't paramaterize it, instead of just stack overflowing
	std::stack<StackContents> stack;// stores t0 and t1 for each subdivision
	stack.emplace(StackContents{t0, t1});

	size_t itterations = 0;// keeps track of itterations, if we exceed certain amount it's a malformed spline
	while (!stack.empty()) {
		auto current = stack.top();
		stack.pop();

		PoseWithCurvature start = getPointWithCurvature(current.t0);
		PoseWithCurvature end = getPointWithCurvature(current.t1);

		// map a twist from start to end to obtain dx, dy, dtheta
		Twist2D twist = start.first.log(end.first);

		// if dx, dy, dtheta is too large, subdivide the spline
		// if it's under threshold, add it to points
		if (std::fabs(twist.dx) > kMaxDx || std::fabs(twist.dy) > kMaxDy || std::fabs(twist.dtheta) > kMaxDtheta) {
			stack.emplace(StackContents{(current.t0 + current.t1) / 2.0, current.t1});
			stack.emplace(StackContents{current.t0, (current.t0 + current.t1) / 2.0});
		} else {
			splinePoints.emplace_back(end);
		}

		if (itterations++ >= kMaxIterations) {
			throw std::runtime_error(
			        "Couldn't paramaterize a malformed spline. For hermite splines, this means that you probably had two "
			        "or "
			        "more adjacent waypoints that were very close together with headings in opposite directions.");
		}
	}

	return splinePoints;
}