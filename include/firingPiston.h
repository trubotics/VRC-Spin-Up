#ifndef FIRINGPISTON_H
#define FIRINGPISTON_H

#include <vex.h>
using namespace vex;

class FiringPiston
{
private:
    brain *Brain = nullptr;
    motor_group *flywheel = nullptr;
    pneumatics *piston = nullptr;
    double lastFiringTime = -200; // The time when the last disk was fired

public:
    FiringPiston(brain Brain, motor_group flywheel, vex::triport::port port);

    void fireDisk(bool skipPreCheck = false);
};

#endif