#ifndef INTAKE_H
#define INTAKE_H

#include <vex.h>
using namespace vex;

class Intake
{
private:
    motor *intake = nullptr;
public:
    Intake(int32_t intakePort);

    void spinIntake(directionType direction);
    void spinIntake(bool R1, bool R2);
    void stopIntake();
};

#endif