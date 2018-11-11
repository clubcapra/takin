//
// Created by ludovic on 11/11/18.
//

#ifndef PROJECT_MOTOR_H
#define PROJECT_MOTOR_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

class MotorPhoenix {

    TalonSRX talonSRX;
    bool position;

public :
    MotorPhoenix(int, bool);

    static const bool LEFT_MOTOR = false;
    static const bool RIGHT_MOTOR = true;

    void setPercentOutput(double);
};

#endif //PROJECT_MOTOR_H
