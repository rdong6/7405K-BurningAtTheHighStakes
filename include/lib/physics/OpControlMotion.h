#pragma once
#include "Motion.h"
#include <vector>

class OpControlMotion : public IMotion {
private:
    std::vector<double> averageRecentYVel = std::vector<double>(5);
    std::vector<double> averageRecentXVel = std::vector<double>(5);

public:
    OpControlMotion() = default;
    MotorVoltages calculateVoltages(kinState state);
    bool isSettled(kinState state);
};