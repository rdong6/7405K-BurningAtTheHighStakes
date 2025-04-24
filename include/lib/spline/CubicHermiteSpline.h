#pragma once
#include "Path.h"
#include "Spline.h"
#include <Eigen/Dense>
#include <vector>

// https://www.uio.no/studier/emner/matnat/ifi/nedlagte-emner/INF-MAT4350/h08/undervisningsmateriale/chap7alecture.pdf
// Algorithm to construct a cubic hermite spline over a dataset, ensuring continuous curvature (used to generate each
// HermiteSpline to each interval (Xᵢ, Xᵢ₊₁))

// The first and last point aren't actually
class CubicHermiteSpline final : public ISpline {
public:
	// Control vector for hermite splines. Control Vector = {{x, cos(θ) * scalar}, {y, sin(θ) * scalar}}
	// stores 0th derivative (0th index) & 1st derivative (1st index)
	struct ControlVector {
		std::array<double, 2> x;// [x, cos(θ) * scalar]
		std::array<double, 2> y;// [y, sin(θ) * scalar]
	};

	CubicHermiteSpline(ControlVector start, ControlVector end);

	[[nodiscard]] Pose getPoint(double t) const override;
	[[nodiscard]] PoseWithCurvature getPointWithCurvature(double t) const override;

private:
	// each row is coefficients for different func
	// row 0 = coefficients of P(t) for X
	// row 1 = coefficients of P(t) for Y
	// row 2 = coefficients of P'(t) for X
	// row 3 = coefficients of P'(t) for Y
	// row 4 = coefficients of P''(t) for X
	// row 5 = coefficients of P''(t) for Y
	Eigen::Matrix<double, 6, 4> coefficients = Eigen::Matrix<double, 6, 4>::Zero();

	/*
	These are our 4 control vectors:
	P(0)    = P0
	P(1)    = P1
	P'(0)   = V(0)  = V0
	P'(1)   = V(1)  = V1

	Basic Matrix Representation of Cubic Hermite Spline:
	                        [1  0   0   0] [P0]
	P(t) =  [1  t   t²  t³] [0  1   0   0] [V0]
	                        [-3 -2  3  -1] [P1]
	                        [2  1   -2  1] [V1]

	We want to find the coefficients of the spline: P(t) = a₃t³ + a₂t² + a₁t + a₀
	P(0)    = a₀
	P'(0)   = a₁
	P(1)    = a₃ + a₂ + a₁ + a₀
	P'(1)   = 3a₃ + 2a₂ + a₁

	so:
	[P(0)   ] =   [0 0 0 1][a₃]
	[P'(0)  ] =   [0 0 1 0][a₂]
	[P(1)   ] =   [1 1 1 1][a₁]
	[P'(1)  ] =   [3 2 1 0][a₀]

	To solve for the coefficients, invert the 4x4 matrix and move to the other side:
	[a₃] = [ 2  1 -2  1][P(0)   ]
	[a₂] = [-3 -2  3 -1][P'(0)  ]
	[a₁] = [ 0  1  0  0][P(1)   ]
	[a₀] = [ 1  0  0  0][P'(1)  ]

	As it can be seen, it's just the inverse of the characteristic matrix from the basic matrix representation above.

	Hermite Basis matrix isn't the normal one, but the inverse one to solve for the coefficients.
	*/
	static constexpr Eigen::Matrix4d hermiteBasis{{+2.0, +1.0, -2.0, +1.0},
	                                              {-3.0, -2.0, +3.0, -1.0},
	                                              {+0.0, +1.0, +0.0, +0.0},
	                                              {+1.0, +0.0, +0.0, +0.0}};
};

Path createCubicSplinesFromWaypoints(const Pose& start, const Pose& end, std::vector<Translation2D> interiorWaypoints);