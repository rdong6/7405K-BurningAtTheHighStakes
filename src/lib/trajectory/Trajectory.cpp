#include "lib/trajectory/Trajectory.h"
#include <fstream>

void Trajectory::dumpTrajectory(const char* filename) {
	std::ofstream outputFile(filename);
	double curVel = 0;
	double curT = 0;
	for (const auto& state : states) {
		// double accel = (state.maxVel * state.maxVel - v * v) / (2.0 * ds);
		double dt = state.t - curT;
		double accel = (state.vel - curVel) / dt;

		outputFile << state.t << ' ' << state.pose.X() << ' ' << state.pose.Y() << ' ' << state.vel << ' ' << accel << ' '
		           << state.curvature << '\n';

		curVel = state.vel;
		curT = state.t;
	}
}