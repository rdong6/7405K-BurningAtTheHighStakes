#include "lib/controllers/FeedForward.h"
#include "lib/utils/Math.h"

// initializes class with our 3 constnats
FeedForward::FeedForward(double kS, double kV, double kA) : kS(kS), kV(kV), kA(kA) {}

// does the calculation. ex: double result = ff(vel, acc);
double FeedForward::operator()(double vel, double acc) const noexcept {
	int sign = util::sign(vel);

	// fpEquality because you cannot just compare two floats by doing float == float
	// we have to do a comparison with espilon because of how floating numbers are stored
	if (util::fpEquality(vel, 0.0)) { sign = 0; }

	return kS * sign + kV * vel + kA * acc;
}

void FeedForward::setKS(double kS) noexcept {
	this->kS = kS;
}

void FeedForward::setKV(double kV) noexcept {
	this->kV = kV;
}

void FeedForward::setKA(double kA) noexcept {
	this->kA = kA;
}

[[nodiscard]] double FeedForward::getKS() const noexcept {
	return kS;
}

[[nodiscard]] double FeedForward::getKV() const noexcept {
	return kV;
}

[[nodiscard]] double FeedForward::getKA() const noexcept {
	return kA;
}