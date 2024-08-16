#pragma once
#ifndef GENERATED_PATH_DEFS
#define GENERATED_PATH_DEFS
#include <array>

// FOR USE IN PURE PURSUIT FROM THE PYTHON GUI

namespace fttbtkjfk {
	struct PursuitPoint {
		struct {
			double x, y;
		} pose;

		double curv;
		// unsigned char event;
	};// PursuitPoint
}// namespace fttbtkjfk
#endif// GENERATED_PATH_DEFS