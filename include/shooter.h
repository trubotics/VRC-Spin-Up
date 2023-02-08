#ifndef FIRINGPISTON_H
#define FIRINGPISTON_H

#include <vex.h>
using namespace vex;

class Shooter
{
private:
    brain *Brain = nullptr;
    motor_group *flywheel = nullptr;
    pneumatics *piston = nullptr;
    double targetVelocity = 50; // The velocity the flywheel should be at to fire
    double lastFiringTime = -200; // The time when the last disk was fired

public:
    Shooter(brain Brain, motor_group flywheel, vex::triport::port port);

    void setTargetVelocity(double targetVelocity);
    void fireDisk(bool skipPreCheck = false);
};

#endif