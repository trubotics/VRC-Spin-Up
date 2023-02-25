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

    // PID "constants"
    double kP = 0.0003;
    double kI = 0.0003;
    double kD = 0.0000;
public:
    Shooter(brain Brain, motor_group flywheel, pneumatics piston);

    // temp
    void changePID(int index, double deltaPID);

    void setTargetVelocity(double targetVelocity);
    void setRelativeTargetVelocity(double targetVelocity);
    void updateVelocity();
    void pidLoop();
    // void changeTargetVelocity(double deltaVelocity); // set target velocity to current velocity + deltaVelocity
    void spinUp();
    void stop();
    bool fireDisk(bool skipPreCheck = false);
};

#endif