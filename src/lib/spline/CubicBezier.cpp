#include "lib/spline/CubicBezier.h"

// TODO: clean up so I can store all coefficients in 1 matrix like CubicHermiteSpline so I can truly remove Spline::getPoint()
CubicBezier::CubicBezier(const Pose& p0, const Pose& p1, const Pose& p2, const Pose& p3) {
	// clang-format off
    this->p <<
    p0.X(), p0.Y(),
    p1.X(), p1.Y(),
    p2.X(), p2.Y(),
    p3.X(), p3.Y();

    // column major order
    // col 0 = p0
    // col 1 = p1
    // col 2 = p2
    // col 3 = p3
    this->coefficients <<
    1, 0, 0, 0,
    -3, 3, 0, 0,
    3, -6, 3, 0,
    -1, 3, -3, 1;

    this->first_deriv_coefficients <<
    -3, 3, 0, 0,
    6, -12, 6, 0,
    -3, 9, -9, 3;

    // (6 - 6t) * P0 + (-12 + 18t) * P1 + (6 - 18t) * P2 + (0 + 6t) * P3
    this->second_deriv_coefficients <<
    6, -12, 6, 0,
    -6, 18, -18, 6;
	// clang-format on
}

Pose CubicBezier::getPoint(double t) const {
	// P(t) = [1 t t^2 t^3] * coefficients * p

	Eigen::RowVector4d T{1, t, t * t, t * t * t};
	auto result = (T * coefficients) * p;
	Pose firstDeriv = getFirstDeriv(t);
	return Pose(result(0), result(1), Rotation2D(firstDeriv.X(), firstDeriv.Y()));
}

PoseWithCurvature CubicBezier::getPointWithCurvature(double t) const {
	return {getPoint(t), getCurvature(getFirstDeriv(t), getSecondDeriv(t))};
}

Pose CubicBezier::getFirstDeriv(double t) const {
	Eigen::RowVector3d T{1, t, t * t};
	auto result = (T * first_deriv_coefficients) * p;
	return Pose(result(0), result(1));
}

Pose CubicBezier::getSecondDeriv(double t) const {
	Eigen::RowVector2d T{1, t};
	auto result = (T * second_deriv_coefficients) * p;
	return Pose(result(0), result(1));
}