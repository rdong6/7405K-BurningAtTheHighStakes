#include "lib/spline/Path.h"

unsigned int Path::counter = 0;

size_t Path::getNumSplines() const {
	return splinesVector.size();
}

size_t Path::getNumPoints() const {
	return splinePoints.size();
}
const TrajectoryConfig& Path::getConfig() const {
	return config;
}

const std::vector<PoseWithCurvature>& Path::getPoints() const {
	return splinePoints;
}

void Path::dumpPath(const char* filename) const {
	std::ofstream outputFile(filename);
	for (const auto& point : splinePoints) { outputFile << point.first.X() << ' ' << point.first.Y() << '\n'; }
}

void Path::parameterizeSplines() {
	if (!splinePoints.empty()) { return; }

	// insert the first point as parameterization doesn't handle it
	splinePoints.emplace_back(splinesVector.front()->getPointWithCurvature(0.0));

	for (auto& spline : splinesVector) {
		auto points = spline->parameterize();
		// append the points into the vector. remove the first point as it's a duplicate of the last point from the previous
		// spline (because C⁰ continuity)
		splinePoints.insert(std::end(splinePoints), std::begin(points) + 1, std::end(points));
	}
}