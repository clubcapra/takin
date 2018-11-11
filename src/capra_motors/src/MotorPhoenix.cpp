//
// Created by ludovic on 11/11/18.
//

#include "MotorPhoenix.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

capra::MotorPhoenix(int, bool) {
    talonSRX = new TalonSRX(int);
    position = bool;
}

void MotorPhoenix::setPercentOutput(double d) {
    talonSRX.Set(ControlMode::PercentOutput, d);
}
