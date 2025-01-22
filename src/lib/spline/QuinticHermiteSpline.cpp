#include "lib/spline/QuinticHermiteSpline.h"

QuinticHermiteSpline::QuinticHermiteSpline(ControlVector start, ControlVector end) {
	Eigen::Vector<double, 6> x{start.x[0], start.x[1], start.x[2], end.x[0], end.x[1], end.x[2]};
	Eigen::Vector<double, 6> y{start.y[0], start.y[1], start.y[2], end.y[0], end.y[1], end.y[2]};

	// setup coefficients for P(t)
	coefficients.block<1, 6>(0, 0) = hermiteBasis * x;
	coefficients.block<1, 6>(1, 0) = hermiteBasis * y;
}


Pose QuinticHermiteSpline::getPoint(double t) const {
	return {};
}

PoseWithCurvature QuinticHermiteSpline::getPointWithCurvature(double t) const {
	return {};
}
