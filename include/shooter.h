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
    double targetVelocity = 80;   // The velocity the flywheel should be at to fire
    double lastFiringTime = -200; // The time when the last disk was fired
    bool isSpinning = false;      // Whether the flywheel is spinning or not

    // PID "constants"
    double kP = 0.05;
    double kI = 0.09;
    double kD = 0.01;
public:
    Shooter(brain Brain, motor_group flywheel, vex::triport::port port);

    // temp
    void changePID(int index, double deltaPID);

    void setTargetVelocity(double targetVelocity);
    void updateVelocity();
    // void changeTargetVelocity(double deltaVelocity); // set target velocity to current velocity + deltaVelocity
    void spinUp();
    void stop();
    bool fireDisk(bool skipPreCheck = false);
};

#endif