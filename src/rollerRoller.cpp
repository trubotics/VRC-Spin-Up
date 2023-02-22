/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       rollerRoller.cpp                                          */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-02-17, 12:15:05 a.m.                                 */
/*    Description:  Rolls the rollers using sensors                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <rollerRoller.h>

RollerRoller::RollerRoller(int32_t rollerPort, optical &colorSensor)
{
    roller = new motor(rollerPort, ratio18_1, false);
    roller->setVelocity(25, vex::velocityUnits::pct);

    this->colorSensor = &colorSensor;
}

void RollerRoller::calibrateTeamColor()
{
    isBlue = colorSensor->getRgb().blue > colorSensor->getRgb().red;
}

bool RollerRoller::isBlueDetected()
{
    return colorSensor->getRgb().blue > colorSensor->getRgb().red;
}

void RollerRoller::rollRoller()
{
    bool blueDetected = isBlueDetected();
    isSpinning = true;
    if (isBlue == blueDetected)
    {
        roller->spin(vex::directionType::rev);
        waitUntil(isBlue != isBlueDetected() || !isSpinning);
    }
    else
    {
        roller->spin(vex::directionType::fwd);
        waitUntil(isBlue == isBlueDetected() || !isSpinning);
    }

    stopRoller();
}

// Manual control in case nothing works
void RollerRoller::rollRoller(directionType direction)
{
    isSpinning = true;
    roller->spin(direction);
}
void RollerRoller::stopRoller()
{
    isSpinning = false;
    roller->stop();
}