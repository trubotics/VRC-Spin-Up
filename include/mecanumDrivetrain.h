#ifndef MECANUMDRIVETRAIN_H
#define MECANUMDRIVETRAIN_H

#include <vex.h>
using namespace vex;

class MecanumDriveTrain
{
private:
    motor leftFront = NULL;
    motor leftBack = NULL;
    motor rightFront = NULL;
    motor rightBack = NULL;
    void convertMotorValues(int forward, int strafe, int turn, int motorValues[]);

public:
    MecanumDriveTrain(int32_t leftFrontPort, int32_t leftBackPort, int32_t rightFrontPort, int32_t rightBackPort);

    void drive(int forward, int strafe, int turn);
};

#endif