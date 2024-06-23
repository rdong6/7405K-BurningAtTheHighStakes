#pragma once
#include <cmath>
#include <numbers>

namespace util {
	template<typename T>
	/**
	 * @param T a numerical type
	 * @param min lowest value you want to return
	 * @param max highest value you want to return
	 * @param curr the value you want to clamp
	 * @return curr if curr is between min and max, otherwise it returns min or max
	 */
	static T clamp(T min, T max, T curr) {
		if (curr > max) {
			return max;
		} else if (curr < min) {
			return min;
		} else {
			return curr;
		}
	}

	// linear interpolate between 2 values
	// 0 ≤ t ≤ 1
	constexpr double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}


	template<class T>
	T normalize(T val, T range) {
		return std::fmod((std::fmod(val, range) + range), range);
	}

	// linear interpolate between two values
	// t is between 0 and 1
	template<typename T>
	constexpr T lerp(const T& startValue, const T& endValue, double t) {
		return startValue + (endValue - startValue) * t;
	}

	constexpr double toRad(double deg) {
		return deg * (std::numbers::pi / 180);
	}

	constexpr double toDeg(double rad) {
		return rad * (180 / std::numbers::pi);
	}

	// all values are in degrees
	constexpr double getShortestAngle(double curHeading, double targetHeading) {
		curHeading = std::fmod(curHeading, 180) - 180.0 * std::round(curHeading / 360);
		double headingErr = targetHeading - curHeading;
		if (std::fabs(headingErr) > 180) { headingErr = headingErr > 0 ? headingErr - 360 : headingErr + 360; }

		return headingErr;
	}

	// clamps degrees to +-180
	constexpr double clampDegrees(double deg) {
		return std::fmod(deg, 180.0) - 180.0 * std::round(deg / (360.0));
	}

	template<class T>
	constexpr bool fpEquality(T a, T b) {
		return std::abs(a - b) < std::numeric_limits<T>::epsilon();
	}

	// sign(-1) = -1, sign(0) = 1, sign(1) = 1
	template<class T>
	constexpr int sign(T n) {
		return (n >= 0) - (n < 0);
	}

	template<class T>
	constexpr T max(T val, T val2) {
		if (val > val2) { return val; }

		return val2;
	}
}// namespace util