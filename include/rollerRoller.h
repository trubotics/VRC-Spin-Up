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

    bool isSpinning = false;
    
    bool isBlueDetected();
public:
    RollerRoller(int32_t rollerPort, optical &colorSensor);

    void calibrateTeamColor();
    void rollRoller();
    void rollRoller(directionType direction);
    void rollRoller(bool L1, bool L2);
    void stopRoller();
};

#endif