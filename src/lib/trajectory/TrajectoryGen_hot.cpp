#ifdef VEX
#include "lib/spline/Path.h"
#include "lib/trajectory/TrajectoryGenerator.h"
#include "lib/trajectory/TrajectoryManager.h"
#include <cstdint>

extern uint8_t __path_data_start[];
extern uint8_t __path_data_end[];
__attribute__((constructor(200))) void pregen_trajectories() {
	const void* pathDataSectionStart = &__path_data_start;
	const void* pathDataSectionEnd = &__path_data_end;
	const uintptr_t pathDataSize =
	        (reinterpret_cast<uintptr_t>(pathDataSectionEnd) - reinterpret_cast<uintptr_t>(pathDataSectionStart));

	const Path* const pathsPregend = static_cast<const Path*>(pathDataSectionStart);
	for (int i = 0; i < pathDataSize / sizeof(Path); i++) {
		const Path& path = pathsPregend[i];

		// generate trajectories and store them in trajectory manager
		sTrajectoryManager.getInstance().storeTrajectory(path, defaultTrajectoryGenerator.generate(path));
	}
}
#endif