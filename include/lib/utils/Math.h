#pragma once

#include <cmath>
#include <numbers>

// These 2 sqrt funcs only exist because current PROS's libm's std::sqrt() still does not compile into a hardware instruction
__attribute__((always_inline)) inline double vsqrtd(double val) {
#ifdef VEX
	double ret;
	// clang-format off
	asm("vsqrt.f64 %P0, %P1"
	: "=w"(ret) /* output. w = VFP 64bit registers */
	: "w"(val) /* input */
	);
	// clang-format on
	return ret;
#else
	return std::sqrt(val);
#endif
}

__attribute__((always_inline)) inline float vsqrtf(float val) {
#ifdef VEX
	float ret;
	// clang-format off
	asm("vsqrt.f32 %0, %1"
	: "=t"(ret) /* output. t = VFP 32bit registers*/
	: "t"(val) /* input */
	);
	// clang-format on
	return ret;
#else
	return std::sqrtf(val);
#endif
}

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

	// normalizes value to be within a certain range (0-range)
	// ex: clamp angle to between 0˚-360˚ instead of angle being continuous
	//	- normalize(angle, 2π) -> restrict to [0, 2π]
	template<class T>
	T normalize(T val, T range) {
		return std::fmod((std::fmod(val, range) + range), range);
	}

	// linear interpolate between two values
	// 0 <= t <= 1
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

	// normalize degrees to +-180
	constexpr double clampDegrees(double deg) {
		deg = std::fmod(deg + 180, 360.0);
		return deg < 0 ? deg + 360 : deg - 180;
	}

	// normalize radians to +-π
	constexpr double clampRadians(double rads) {
		rads = std::fmod(rads + std::numbers::pi, 2.0 * std::numbers::pi);
		return rads < 0 ? rads + 2.0 * std::numbers::pi : rads - std::numbers::pi;
	}

	// all values are in degrees
	constexpr double getShortestAngle(double curHeading, double targetHeading) {
		curHeading = std::fmod(curHeading, 180) - 180.0 * std::round(curHeading / 360);
		double headingErr = targetHeading - curHeading;
		if (std::fabs(headingErr) > 180) { headingErr = headingErr > 0 ? headingErr - 360 : headingErr + 360; }

		return headingErr;
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