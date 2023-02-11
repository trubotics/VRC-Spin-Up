#ifndef MECANUMDRIVETRAIN_H
#define MECANUMDRIVETRAIN_H

#include <vex.h>
using namespace vex;

class MecanumDriveTrain
{
private:
    const double GEAR_MULTIPLIER = 4;

    motor leftFront = NULL;
    motor leftBack = NULL;
    motor rightFront = NULL;
    motor rightBack = NULL;

    bool motorLock = true;

    void convertMotorValues(int forward, int strafe, int turn, int motorValues[]);

public:
    MecanumDriveTrain(int32_t leftFrontPort, bool leftFrontReversed,
                      int32_t leftBackPort, bool leftBackReversed,
                      int32_t rightFrontPort, bool rightFrontReversed,
                      int32_t rightBackPort, bool rightBackReversed);

    void drive(int forward, int strafe, int turn);
    void driveFor(int forward, int strafe, int turn, double rotations);

    bool getMotorLock();
    void setMotorLock(bool lock);
};

#endif