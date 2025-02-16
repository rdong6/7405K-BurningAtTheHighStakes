// Only want to include trajectory manager in robot's code, not PathEditor code
#ifdef VEX
#include "lib/trajectory/TrajectoryManager.h"
#include "lib/spline/Path.h"
#include "lib/trajectory/TrajectoryGenerator.h"

const Trajectory& TrajectoryManager::getTrajectory(const Path& path) {
	if (trajectories.contains(path.id)) { return trajectories.at(path.id); }

	// trajectory wasn't already generated so we generate it first
	trajectories.emplace(path.id, defaultTrajectoryGenerator.generate(path));
	return trajectories.at(path.id);
}

void TrajectoryManager::storeTrajectory(const Path& path, Trajectory trajectory) {
	trajectories.insert_or_assign(path.id, std::move(trajectory));
}
#endif