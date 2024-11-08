#pragma once
#include "lib/geometry/kinState.h"
#include "lib/utils/is_derived.h"
#include <any>
#include <cstdint>
#include <memory>
#include <type_traits>

// Base class for every motion that our robot does either during auton or opcontrol
// Basically a command that the drivetrain uses to do a certain thing
class IMotion {
public:
	// unit: mv
	struct MotorVoltages {
		double left, right;
	};

	// Global timeout for motion is done through Drive::waitUntilSettled()
	//
	// Exit condition used for motion chaining or for steady state err (small & large values)
	struct ExitCondition {
		//
	};

	IMotion() = default;
	virtual void start();
	// makes drive use motor's set_velocity instead of set_voltage - defaults to voltage
	virtual bool isVelocityControlled() const;
	virtual MotorVoltages calculate(const kinState state) = 0;
	virtual bool isSettled(const kinState state) = 0;

protected:
	// in ms
	uint32_t startTime = 0;

	std::optional<ExitCondition> exitCondition = std::nullopt;
};


// Encapsulates the IMotion class, allowing us to use polymorphism without the need for a pointer
class Motion {
private:
	std::any storage;
	IMotion& (*getter)(std::any&);

public:
	template<typename ConcreteMotion>
	    requires util::Derived<ConcreteMotion, IMotion>
	Motion(ConcreteMotion&& motion)
	    : storage(std::forward<ConcreteMotion>(motion)), getter{[](std::any& storage) -> IMotion& {
		      //   static_assert(std::is_base_of<IMotion, ConcreteMotion>(),
		      //                 "Motion object must derive from IMotion class.");
		      return std::any_cast<ConcreteMotion&>(storage);
	      }} {}

	IMotion* operator->() { return std::addressof(getter(storage)); }
};