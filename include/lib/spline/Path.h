#pragma once
#include "Spline.h"
#include <fstream>

/*class Path {
public:
    template<SplineType... splines>
    // explicit Path(splines&&... args) : pimpl(std::make_unique<PathInternal<splines...>>(std::forward<splines>(args)...)) {}
    explicit Path(splines... args) : pimpl(std::make_unique<PathInternal<splines...>>(args...)) {}

    [[nodiscard]] size_t getNumSplines() const { return pimpl->getNumSplines(); }

    [[nodiscard]] size_t getNumPoints() const { return pimpl->getNumPoints(); }

    [[nodiscard]] const std::vector<PoseWithCurvature>& getPoints() const { return pimpl->getPoints(); }

private:
    class IPathImpl {
    public:
        virtual ~IPathImpl() = 0;
        [[nodiscard]] virtual size_t getNumSplines() const = 0;
        [[nodiscard]] virtual size_t getNumPoints() const = 0;
        [[nodiscard]] virtual const std::vector<PoseWithCurvature>& getPoints() const = 0;
    };

    template<SplineType... splines>
    class PathInternal : public IPathImpl {
    public:
        explicit PathInternal(splines... args) : splinesTuple(args...) {
            // insert the first point as parameterization doesn't handle it
            splinePoints.emplace_back(std::get<0>(splinesTuple).getPoint(0.0), std::get<0>(splinesTuple).getCurvature(0.0));

            util::tuple::for_each(splinesTuple, [&](const Spline& spline) {
                // do the spline parameterization for each spline
                auto points = spline.parameterize();

                // append the points into the vector. remove the first point as it's a duplicate of the last point from the
                // previous spline (C⁰ continuity)
                splinePoints.insert(std::end(splinePoints), std::begin(points) + 1, std::end(points));
            });

            numSplines = std::tuple_size_v<std::tuple<splines...>>;
            // static_assert(std::tuple_size_v<std::tuple<splines...>> - 1 > 0);
        }

        PathInternal(const PathInternal& other) {
            //
        }

        ~PathInternal() override;

        [[nodiscard]] size_t getNumSplines() const override { return numSplines; }
        [[nodiscard]] size_t getNumPoints() const override { return splinePoints.size(); }

        const std::vector<PoseWithCurvature>& getPoints() const override { return splinePoints; }

    private:
        const std::tuple<splines...> splinesTuple;
        std::vector<PoseWithCurvature> splinePoints;// spline points from parameterizing all the splines
        std::size_t numSplines = 1;
    };

    std::unique_ptr<IPathImpl> pimpl;// pointer to impl
};*/


// Initial thing
/*template<SplineType... splines>
class Path {
public:
    explicit Path(splines... args) : splinesTuple(args...) {
        // insert the first point as parameterization doesn't handle it
        // splinePoints.emplace_back(std::get<0>(splinesTuple).getPoint(0.0), std::get<0>(splinesTuple).getCurvature(0.0));
        splinePoints.emplace_back(std::get<0>(splinesTuple).getPointWithCurvature(0.0));

        util::tuple::for_each(splinesTuple, [&](const ISpline& spline) {
            // do the spline parameterization for each spline
            auto points = spline.parameterize();

            // append the points into the vector. remove the first point as it's a duplicate of the last point from the previous
            // spline (C⁰ continuity)
            splinePoints.insert(std::end(splinePoints), std::begin(points) + 1, std::end(points));
        });

        numSplines = std::tuple_size_v<std::tuple<splines...>>;
        // static_assert(std::tuple_size_v<std::tuple<splines...>> - 1 > 0);
    }

    [[nodiscard]] size_t getNumSplines() const { return numSplines; }
    [[nodiscard]] size_t getNumPoints() const { return splinePoints.size(); }

    [[nodiscard]] const std::vector<PoseWithCurvature>& getPoints() const { return splinePoints; }

    void dumpPath(const char* filename) const {
        std::ofstream outputFile(filename);
        for (const auto& point : splinePoints) { outputFile << point.first.X() << ' ' << point.first.Y() << '\n'; }
    }

private:
    const std::tuple<splines...> splinesTuple;
    std::vector<PoseWithCurvature> splinePoints;// spline points from parameterizing all the splines
    std::size_t numSplines = 1;
};*/

// Simply a container that stores multiple splines to form a path
class Path {
public:
	template<SplineType... splines>
	explicit Path(splines&&... args) : splinesVector{Spline(std::forward<splines>(args))...} {
		parameterizeSplines();
	}

	explicit Path(std::vector<Spline>&& splines) : splinesVector(std::forward<std::vector<Spline>>(splines)) {
		parameterizeSplines();
	}

	[[nodiscard]] size_t getNumSplines() const { return splinesVector.size(); }
	[[nodiscard]] size_t getNumPoints() const { return splinePoints.size(); }

	[[nodiscard]] const std::vector<PoseWithCurvature>& getPoints() const { return splinePoints; }

	// for debugging
	void dumpPath(const char* filename) const {
		std::ofstream outputFile(filename);
		for (const auto& point : splinePoints) { outputFile << point.first.X() << ' ' << point.first.Y() << '\n'; }
	}

private:
	// parameterize all the splines and store points
	void parameterizeSplines() {
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

	std::vector<Spline> splinesVector;
	std::vector<PoseWithCurvature> splinePoints;// points from parameterizing all the splines
};