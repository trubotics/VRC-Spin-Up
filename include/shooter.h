#ifndef FIRINGPISTON_H
#define FIRINGPISTON_H

#include <vex.h>
using namespace vex;

class Shooter
{
private:
    // Speed constants
    const double MIN_VELOCITY = 60;
    const double MAX_VELOCITY = 75;
    const double VELOCITY_RANGE = MAX_VELOCITY - MIN_VELOCITY; 

    // PID constants
    const double kP = 0.05;
    const double kI = 0.09;
    const double kD = 0.01;
    
    brain *Brain = nullptr;
    motor_group *flywheel = nullptr;
    pneumatics *piston = nullptr;
    double targetVelocity = MAX_VELOCITY;   // The velocity the flywheel should be at to fire
    double lastFiringTime = -200; // The time when the last disk was fired
    bool isSpinning = false;      // Whether the flywheel is spinning or not
public:
    Shooter(brain Brain, motor_group flywheel, pneumatics piston);

    void setTargetVelocity(double targetVelocity);
    void setRelativeTargetVelocity(double targetVelocity);
    void updateVelocity();
    // void changeTargetVelocity(double deltaVelocity); // set target velocity to current velocity + deltaVelocity
    void spinUp();
    void stop();
    bool fireDisk(bool skipPreCheck = false);
};

#endif