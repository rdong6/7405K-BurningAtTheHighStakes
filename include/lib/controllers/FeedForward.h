#pragma once

class FeedForward {
private:
	double kS = 0;
	double kV = 0;
	double kA = 0;

public:
	FeedForward() = default;
	FeedForward(double kS, double kV, double kA);

	double operator()(double vel, double acc) const noexcept;

	// typical things in case they are actually needed

	void setKS(double kS) noexcept;
	void setKV(double kV) noexcept;
	void setKA(double kA) noexcept;

	[[nodiscard]] double getKS() const noexcept;
	[[nodiscard]] double getKV() const noexcept;
	[[nodiscard]] double getKA() const noexcept;
};