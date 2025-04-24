#pragma once
#include "Spline.h"

// Use this macro w/ Path class declarations to pregen the trajectory
// Doesn't work with static local objs because of C++ Initialization.
#define PREGEN_TRAJECTORY __attribute__((init_priority(105))) __attribute__((section(".path_data")))

class TrajectoryManager;

struct TrajectoryConfig {
	double startVel;
	double endVel;
	double maxVel;
	double maxAcc;
	bool reversed;
};

// Simply a container that stores multiple splines to form a path
class Path {
public:
	// TODO: potentially fix this. i think i have some unwanted behaviour due to type deduction
	// TODO: lazily parameterize the splines
	template<SplineType... splines>
	explicit Path(TrajectoryConfig config, splines&&... args)
	    : config(config), splinesVector{Spline(std::forward<splines>(args))...}, id(counter++) {
		parameterizeSplines();
	}

	explicit Path(std::vector<Spline>&& splines) : config(), splinesVector(std::forward<std::vector<Spline>>(splines)), id(counter++) {
		parameterizeSplines();
	}

	[[nodiscard]] size_t getNumSplines() const;

	[[nodiscard]] size_t getNumPoints() const;

	[[nodiscard]] const TrajectoryConfig& getConfig() const;

	[[nodiscard]] const std::vector<PoseWithCurvature>& getPoints() const;

	// for debugging
	void dumpPath(const char* filename) const;

private:
	// TODO: update this to provide an actual way of keeping track of every unique path, not just a crappy counter -> so if I
	// create a new Path obj with same splines, it'll point to same trajectory in trajectory manager
	//
	// global counter -> used as an id for each unique path created (simply gets incremented in ctor)
	static unsigned int counter;

	// parameterize all the splines and store points
	void parameterizeSplines();

	TrajectoryConfig config;
	std::vector<Spline> splinesVector;
	std::vector<PoseWithCurvature> splinePoints;// points from parameterizing all the splines

	/*
	 * why not use a ptr to a generated trajectory?
	 * it'd work fine for all pre-gen'd trajectories as copy ctor would also copy the ptr to the generated trajectory
	 *
	 * but what if i make a copy of this path obj before i generated the trajectory? then the copy would have 0 clue where the
	 * generated trajectory is
	 *
	 * end goal: we generate 1 trajectory for each unique path -> duplicate paths point to same generated trajectory
	 */
	unsigned int id;// id for this unique path -> used to obtain pre-gen'd trajectory

#ifdef VEX
	friend TrajectoryManager;
#endif
};