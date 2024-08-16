#pragma once

// Path output from 7405OnlyCurves

#ifndef ONLY_CURVES_PATH_DEFS
#define ONLY_CURVES_PATH_DEFS
#include <array>
namespace fttbtkjfk {
	struct GeneratedPoint {
		double time;
		struct {
			double x, y, heading;
		} pose;
		struct {
			double left, right;
		} wheelVels;
		double vel, accel, curv;
	};
}// namespace fttbtkjfk
#endif