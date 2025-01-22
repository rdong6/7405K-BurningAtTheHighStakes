#pragma once
#include "Spline.h"
#include <Eigen/Dense>

class QuinticHermiteSpline final : ISpline {
public:
	// stores location of point, 1st derivative, 2nd derivative
	struct ControlVector {
		std::array<double, 3> x;
		std::array<double, 3> y;
	};

	QuinticHermiteSpline(ControlVector start, ControlVector end);

	[[nodiscard]] Pose getPoint(double t) const override;
	[[nodiscard]] PoseWithCurvature getPointWithCurvature(double t) const override;

private:
	Eigen::Matrix<double, 6, 6> coefficients = Eigen::Matrix<double, 6, 6>::Zero();

	//@formatter:off
	//clang-format off
	static constexpr Eigen::Matrix<double, 6, 6> hermiteBasis{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
	                                                          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
	                                                          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	//clang-format on
	//@formatter:on
};
