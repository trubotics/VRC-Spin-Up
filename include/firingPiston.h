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

public:
    FiringPiston(brain Brain, motor_group flywheel, vex::triport::port port);

    void fireDisk(bool skipPreCheck = false);
    void checkPistonRetract();
};

#endif