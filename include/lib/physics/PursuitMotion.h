#pragma once
#include "Logger.h"
#include "Motion.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/trajectory/PursuitPoint.h"
#include <span>
#include <utility>

class PursuitMotion : public IMotion {
private:
	std::span<fttbtkjfk::PursuitPoint> path;
	std::shared_ptr<double[]> vels;// stores vels along a path. shared_ptr as unique_ptr is incompatabile w/ std::any as
	                               // std::any requires objects to be copy constructible
	const double k;
	const double threshold;
	const double lookahead;// lookahead dist
	double prevTime;
	const double maxVel;
	const double maxAccel;
	double curVel;                 // output of rate limited
	mutable int lastLookaheadIndex;// last index searched when calculating lookahead point
	Pose lastLookaheadPose;        // used incase we cannot get a circle line intersect

	const int isReversed;

private:
	// find index of closest point on path to the robot's position
	int findClosestIndex(const Pose& pos) const;

	/*
	Finds intersection point between line and circle

	p1 = start of line
	p2 = end of line
	pos = robot's position

	returns proportion along the line that the robot's lookahead intersects
	*/
	double circleIntersect(const Pose& p1, const Pose& p2, const Pose& pos) const;

	/*
	calculates next lookahead pos from circle line intersection of line between robot's pos and line drawn from 2 points
	on path
	*/
	Pose getLookaheadPoint(const Pose& pos) const;

	/*
	heading is just robot heading converted for math

	pos = current robot position
	heading = current robot heading based on X axis instead of along Y axis - for normal trig functions
	lookahead = lookahead pose

	calculates curvature from robot position and the lookahead position
	*/
	double calcCurv(const Pose& pos, double heading, const Pose& lookahead) const;

	double curDistFromEnd(kinState state) const;

public:
	PursuitMotion(std::span<fttbtkjfk::PursuitPoint>&& path, double lookahead, double maxVel, double maxAccel,
	              bool reversed = false, double threshold = 1, double k = 2);
	void start() override;
	MotorVoltages calculate(const kinState state) override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState state) override;
};