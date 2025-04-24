#pragma once
#include "Trajectory.h"
#include <unordered_map>

// Only want to include trajectory manager in robot's code, not PathEditor code
#ifdef VEX
#define sTrajectoryManager TrajectoryManager::getInstance()

class Path;

// Singleton!! People hate on singletons, I can't figure out another design pattern that does what I need
// 1 global instance and ONLY 1 INSTANCE!
class TrajectoryManager {
private:
	TrajectoryManager() = default;

public:
	TrajectoryManager(const TrajectoryManager&) = delete;
	TrajectoryManager& operator=(const TrajectoryManager&) = delete;

	static TrajectoryManager& getInstance() {
		static TrajectoryManager INSTANCE;

		return INSTANCE;
	}

	const Trajectory& getTrajectory(const Path& path);
	void storeTrajectory(const Path& path, Trajectory trajectory);

private:
	std::unordered_map<unsigned int, Trajectory> trajectories{};
};
#endif