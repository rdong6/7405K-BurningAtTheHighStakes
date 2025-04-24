#include "lib/spline/Boomerang.h"

Boomerang::Boomerang(const Pose& start, const Pose& end, double dlead) {
	const double dist = start.translation().distanceTo(end.translation());
	const Pose carrot = Pose(end.X() - dist * end.rotation().cos() * dlead, end.Y() - dist * end.rotation().sin() * dlead);

	Eigen::Vector3d x{start.X(), carrot.X(), end.X()};
	Eigen::Vector3d y{start.Y(), carrot.Y(), end.Y()};

	coefficients.block<1, 3>(0, 0) = characteristicMatrix * x;
	coefficients.block<1, 3>(1, 0) = characteristicMatrix * y;

	// calc coefficients for P'(t)
	for (int i = 0; i < 2; i++) { coefficients.block<2, 1>(2, i + 1) = coefficients.block<2, 1>(0, i) * (2 - i); }

	// calc coefficients for P''(t)
	coefficients.block<2, 1>(4, 2) = coefficients.block<2, 1>(2, 1);
}

Pose Boomerang::getPoint(double t) const {
	// P(t) = a₂t² + a₁t + a₀
	Eigen::Vector3d T{t * t, t, 1};
	Eigen::Vector<double, 6> result = coefficients * T;

	// If t = 0, all other terms in the equation cancel out to zero. We can use
	// the last x^0 term in the equation.
	double dx, dy;
	if (util::fpEquality(t, 0.0)) {
		dx = coefficients(2, 3);
		dy = coefficients(3, 3);
	} else {
		dx = result(2);
		dy = result(3);
	}

	return {result(0), result(1), Rotation2D{dx, dy}};
}

PoseWithCurvature Boomerang::getPointWithCurvature(double t) const {
	// P(t) = a₂t² + a₁t + a₀
	Eigen::Vector3d T{t * t, t, 1};
	Eigen::Vector<double, 6> result = coefficients * T;

	double dx, dy, ddx, ddy;
	// If t = 0, all other terms in the equation cancel out to zero. We can use
	// the last x^0 term in the equation.
	if (util::fpEquality(t, 0.0)) {
		dx = coefficients(2, 2);
		dy = coefficients(3, 2);
		ddx = coefficients(4, 2);
		ddy = coefficients(5, 2);
	} else {
		dx = result(2);
		dy = result(3);
		ddx = result(4);
		ddy = result(5);
	}
	return {Pose(result(0), result(1), Rotation2D{dx, dy}), getCurvature(dx, dy, ddx, ddy)};
}