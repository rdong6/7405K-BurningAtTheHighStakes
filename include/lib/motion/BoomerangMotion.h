#pragma once
#include "Constants.h"
#include "Motion.h"
#include "RAMSETEMotion.h"
#include "lib/spline/Boomerang.h"
#include "lib/spline/Path.h"
#include "lib/trajectory/Trajectory.h"
#include "lib/trajectory/TrajectoryGenerator.h"
#include "lib/trajectory/constraints/CentripetalAccelerationConstraint.h"

// motion using a boomerang controller
// https://www.desmos.com/calculator/sptjw5szex

class BoomerangMotion : public IMotion {
private:
	TrajectoryGenerator trajectoryGenerator{{CentripetalAccelerationConstraint{trajectory::maxCentripetalAcceleration}}};

	Path boomerangPath;
	Trajectory generatedTrajectory;
	double elapsedTime;
	double prevTime;

public:
	// The reason why we pass in the start pose which is obtained from odometry in the ctor is because of how scheduling works
	// if we generated/paramaterized the path in IMotion::start(), the time it takes could mess up the rest of the cycle as it
	// would be called during the drive code instead, we do it on constructor of boomerang motion because that happens in the
	// autonomousUser thread, which happens at the end of everything else
	//
	// dlead between 0-1 inclusive
	BoomerangMotion(const Pose& start, const Pose& target, double dlead, double maxVel, double maxAcc, bool reversed = false);
	MotorVoltages calculate(const kinState& state) override;
	void start() override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState& state) override;
};