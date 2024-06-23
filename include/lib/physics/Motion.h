#pragma once
#include "lib/geometry/kinState.h"
#include "lib/utils/is_derived.h"
#include <any>
#include <cstdint>
#include <type_traits>

// Base class for every motion that our robot does either during auton or opcontrol
// Basically a command that the drivetrain uses to do a certain thing
class IMotion {
protected:
	// in ms
	uint32_t startTime = 0;

public:
	// in mv
	struct MotorVoltages {
		double left, right;
	};

	IMotion() = default;
	virtual void start();
	// makes drive use motor's set_velocity instead of set_voltage - defaults to voltage
	virtual bool isVelocityControlled() const;
	virtual MotorVoltages calculateVoltages(kinState state) = 0;
	virtual bool isSettled(kinState state) = 0;
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

	IMotion* operator->() {
		return &getter(storage);
	}
};