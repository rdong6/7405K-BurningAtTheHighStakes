#include "lib/spline/CubicHermiteSpline.h"
#include "Eigen/src/Core/Block.h"
#include "Eigen/src/Core/Matrix.h"

CubicHermiteSpline::CubicHermiteSpline(ControlVector start, ControlVector end) {
	// To get a₃, a₂, a₁, a₀ coefficients for X axis, it's hermite basis * [start's x, start's x vel, end's x, end's x vel]
	Eigen::Vector4d x{start.x[0], start.x[1], end.x[0], end.x[1]};
	Eigen::Vector4d y{start.y[0], start.y[1], end.y[0], end.y[1]};

	// setup coefficients for P(t)
	coefficients.block<1, 4>(0, 0) = hermiteBasis * x;
	coefficients.block<1, 4>(1, 0) = hermiteBasis * y;

	// calc coefficients for P'(t)
	for (int i = 0; i < 3; i++) { coefficients.block<2, 1>(2, i + 1) = coefficients.block<2, 1>(0, i) * (3 - i); }

	// calc coefficients for P''(t)
	for (int i = 0; i < 2; i++) { coefficients.block<2, 1>(4, i + 2) = coefficients.block<2, 1>(2, i + 1) * (2 - i); }
}

Pose CubicHermiteSpline::getPoint(double t) const {
	// P(t) = a₃t³ + a₂t² + a₁t + a₀
	Eigen::Vector4d polynomialBases{t * t * t, t * t, t, 1};
	Eigen::Vector<double, 6> result = coefficients * polynomialBases;

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

PoseWithCurvature CubicHermiteSpline::getPointWithCurvature(double t) const {
	// P(t) = a₃t³ + a₂t² + a₁t + a₀
	Eigen::Vector4d polynomialBases{t * t * t, t * t, t, 1};
	Eigen::Vector<double, 6> result = coefficients * polynomialBases;

	double dx, dy, ddx, ddy;
	// If t = 0, all other terms in the equation cancel out to zero. We can use
	// the last x^0 term in the equation.
	if (util::fpEquality(t, 0.0)) {
		dx = coefficients(2, 3);
		dy = coefficients(3, 3);
		ddx = coefficients(4, 3);
		ddy = coefficients(5, 3);
	} else {
		dx = result(2);
		dy = result(3);
		ddx = result(4);
		ddy = result(5);
	}

	return {Pose(result(0), result(1), Rotation2D{dx, dy}), getCurvature(dx, dy, ddx, ddy)};
}

// returns control vector for initial and endpoints
static std::array<CubicHermiteSpline::ControlVector, 2>
getInitialControlVectors(const Pose& start, const Pose& end, const std::vector<Translation2D>& interiorWaypoints) {
	double scalar;
	if (interiorWaypoints.empty()) {
		scalar = 1.2 * start.translation().distanceTo(end.translation());
	} else {
		scalar = 1.2 * start.translation().distanceTo(interiorWaypoints.front());
	}

	const CubicHermiteSpline::ControlVector initialCV{{start.X(), start.rotation().cos() * scalar},
	                                                  {start.Y(), start.rotation().sin() * scalar}};

	if (!interiorWaypoints.empty()) { scalar = 1.2 * end.translation().distanceTo(interiorWaypoints.back()); }

	const CubicHermiteSpline::ControlVector finalCV{{end.X(), end.rotation().cos() * scalar},
	                                                {end.Y(), end.rotation().sin() * scalar}};

	return {initialCV, finalCV};
}

// https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm -> Thomas Algorithm
// a = values of A above the diagonal
// b = values of A on the diagonal
// c = values of A below the diagonal
// d = vector on RHS
// solution = output solution vector
// scratch vectors to store temp matrices to save on excess allocations & deallocations when calling this algorithm many times
static void thomasAlgorithim(const std::vector<double>& a, const std::vector<double>& b, const std::vector<double>& c,
                             const std::vector<double>& d, std::vector<double>& solution, std::vector<double>& d_scratch,
                             std::vector<double>& c_scratch) {
	const size_t N = d.size();

	// TODO: Check for divide by 0!!
	// updates coefficients in 1st row
	c_scratch[0] = c[0] / b[0];
	d_scratch[0] = d[0] / b[0];

	// create c_star & d_star coefficients in first sweep
	for (size_t i = 1; i < N; i++) {
		double m = 1.0 / (b[i] - a[i] * c_scratch[i - 1]);
		c_scratch[i] = c[i] * m;
		d_scratch[i] = (d[i] - a[i] * d_scratch[i - 1]) * m;
	}

	// reverse sweep to update the solution vector
	solution[N - 1] = d_scratch[N - 1];
	for (int i = N - 2; i >= 0; i--) { solution[i] = d_scratch[i] - c_scratch[i] * solution[i + 1]; }
}

Path createCubicSplinesFromWaypoints(const Pose& start, const Pose& end, std::vector<Translation2D> interiorWaypoints) {
	std::vector<Spline> splines;

	// Get control vector for first and end points
	auto initialCVs = getInitialControlVectors(start, end, interiorWaypoints);
	const auto& startCV = initialCVs[0];
	const auto& endCV = initialCVs[1];

	if (interiorWaypoints.size() > 1) {
		// calculates the interior waypoints' control vectors to ensure a C2 continuous hermite spline

		// adds start and end points to list of waypoints before solving
		interiorWaypoints.emplace(interiorWaypoints.begin(), startCV.x[0], startCV.y[0]);
		interiorWaypoints.emplace_back(endCV.x[0], endCV.y[0]);

		// do some math here that i don't understand 🙂 ⁌--- 💥🔫

		// Solve some tridiagonal system for clamped cubics
		// https://www.uio.no/studier/emner/matnat/ifi/nedlpaagte-emner/INF-MAT4350/h08/undervisningsmateriale/chap7alecture.pdf
		// https://www.youtube.com/watch?v=zhSoqOOD1pA

		// above-diagonal of tridiagonal matrix, zero-padded
		std::vector<double> a;
		// diagonal of tridiagonal matrix
		std::vector<double> b(interiorWaypoints.size() - 2, 4.0);
		// below-diagonal of tridiagonal matrix, zero-padded
		std::vector<double> c;
		// rhs vectors
		std::vector<double> dx, dy;
		// solution vectors
		std::vector<double> fx(interiorWaypoints.size() - 2, 0.0), fy(interiorWaypoints.size() - 2, 0.0);

		// populate above-diagonal and below-diagonal vectors
		a.emplace_back(0);
		for (size_t i = 0; i < interiorWaypoints.size() - 3; ++i) {
			a.emplace_back(1);
			c.emplace_back(1);
		}
		c.emplace_back(0);

		// populate rhs vectors
		dx.emplace_back(3 * (interiorWaypoints[2].X() - interiorWaypoints[0].X()) - startCV.x[1]);
		dy.emplace_back(3 * (interiorWaypoints[2].Y() - interiorWaypoints[0].Y()) - startCV.y[1]);
		if (interiorWaypoints.size() > 4) {
			for (size_t i = 1; i <= interiorWaypoints.size() - 4; ++i) {
				// dx and dy represent the derivatives of the internal waypoints. The
				// derivative of the second internal waypoint should involve the third
				// and first internal waypoint, which have indices of 1 and 3 in the
				// waypoints list (which contains ALL waypoints).
				dx.emplace_back(3 * (interiorWaypoints[i + 2].X() - interiorWaypoints[i].X()));
				dy.emplace_back(3 * (interiorWaypoints[i + 2].Y() - interiorWaypoints[i].Y()));
			}
		}
		dx.emplace_back(3 * (interiorWaypoints[interiorWaypoints.size() - 1].X() -
		                     interiorWaypoints[interiorWaypoints.size() - 3].X()) -
		                endCV.x[1]);
		dy.emplace_back(3 * (interiorWaypoints[interiorWaypoints.size() - 1].Y() -
		                     interiorWaypoints[interiorWaypoints.size() - 3].Y()) -
		                endCV.y[1]);

		// compute solution to tridiagonal system
		std::vector<double> d_star(dx.size(), 0.0);
		std::vector<double> c_star(dx.size(), 0.0);
		thomasAlgorithim(a, b, c, dx, fx, d_star, c_star);
		thomasAlgorithim(a, b, c, dy, fy, d_star, c_star);

		// include the start and end control vectors before going over every sub-interval and creating a spline
		fx.emplace(fx.begin(), startCV.x[1]);
		fx.emplace_back(endCV.x[1]);
		fy.emplace(fy.begin(), startCV.y[1]);
		fy.emplace_back(endCV.y[1]);

		// finally create all the splines
		for (size_t i = 0; i < fx.size() - 1; i++) {
			splines.emplace_back(CubicHermiteSpline{
			        CubicHermiteSpline::ControlVector{interiorWaypoints[i].X(), fx[i], {interiorWaypoints[i].Y(), fy[i]}},
			        CubicHermiteSpline::ControlVector{interiorWaypoints[i + 1].X(),
			                                          fx[i + 1],
			                                          {interiorWaypoints[i + 1].Y(), fy[i + 1]}}});
		}
	} else if (interiorWaypoints.size() == 1) {
		const double xDeriv = (3.0 * (endCV.x[0] - startCV.x[0]) - endCV.x[1] - startCV.x[1]) / 4.0;
		const double yDeriv = (3.0 * (endCV.y[0] - startCV.y[0]) - endCV.y[1] - startCV.y[1]) / 4.0;

		const CubicHermiteSpline::ControlVector midControlVector{{interiorWaypoints[0].X(), xDeriv},
		                                                         {interiorWaypoints[0].Y(), yDeriv}};

		splines.emplace_back(CubicHermiteSpline{startCV, midControlVector});
		splines.emplace_back(CubicHermiteSpline{midControlVector, endCV});
	} else {
		// only 1 spline 🙂🙂🙂🙂🙂
		splines.emplace_back(CubicHermiteSpline{startCV, endCV});
	}

	return Path(std::move(splines));
}