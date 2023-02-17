#ifndef ROLLERROLLER_H
#define ROLLERROLLER_H

#include <vex.h>
using namespace vex;

class RollerRoller
{
private:
    bool isBlue = false;

    motor *roller = nullptr;
    optical *colorSensor = nullptr;

    bool isBlueDetected();
public:
    RollerRoller(int32_t rollerPort, optical &colorSensor);

    void calibrateTeamColor();
    void rollRoller();
    void rollRoller(directionType direction);
    void stopRoller();
};

#endif