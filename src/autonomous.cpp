/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       autonomous.cpp                                            */
/*    Author:       Trubotics                                                 */
/*    Created:      2023-02-11, 01:14:45 a.m.                                 */
/*    Description:  Autonomous related functions                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <autonomous.h>

using namespace vex;

Autonomous::Autonomous(MecanumDriveTrain& drive, Shooter& shooter, RollerRoller& roller,
    distance& leftDistance, distance& rightDistance, inertial& inertialSensor)
{
    this->drive = &drive;
    this->shooter = &shooter;
    this->roller = &roller;

    this->leftDistance = &leftDistance;
    this->rightDistance = &rightDistance;
    this->inertialSensor = &inertialSensor;
}

Strategy Autonomous::getStrategy()
{
    return strategy;
}

std::string Autonomous::getStrategyString()
{
    switch (strategy)
    {
    case Strategy::None:
        return "None";
    case Strategy::LoaderRoller:
        return "Loader Roller";
    case Strategy::SideRoller:
        return "Side Roller";
    default:
        return "Unknown";
    }
}

void Autonomous::setStrategy(Strategy strategy)
{
    Autonomous::strategy = strategy;
}

// Strafe using the distance sensors (***IN INCHES***) because all the game element measurements are in inches
// Use negative distance to strafe left
// By default, this strafes towards the target object, use a negative speed to strafe away
void Autonomous::sensorStrafe(double targetDistance, double speed)
{
    // select sensor
    vex::distance* selectedSensor;
    if (targetDistance > 0)
    {
        selectedSensor = rightDistance;
    }
    else
    {
        selectedSensor = leftDistance;
        targetDistance = -targetDistance;
        speed = -speed;
    }

    // strafe
    if (speed > 0)
    {
        while (selectedSensor->objectDistance(vex::distanceUnits::in) > targetDistance)
        {
            drive->drive(0, speed, 0);
        }
    }
    else
    {
        while (selectedSensor->objectDistance(vex::distanceUnits::in) < targetDistance)
        {
            drive->drive(0, speed, 0);
        }
    }

    drive->drive(0, 0, 0);
}

// Rolls the roller
void Autonomous::rollRoller()
{
    // "creep" to roller
    drive->drive(-5, 0, 0);
    waitUntil(drive->getAvgTorque() > 0.5);

    // calibrate team color
    roller->calibrateTeamColor();

    // continue driving into the roller very gently and roll the roller
    roller->rollRoller(directionType::fwd);
    wait(300, timeUnits::msec);
    roller->stopRoller();
    drive->drive(0, 0, 0);
}

void Autonomous::run()
{
    leftDistance->objectDistance(vex::distanceUnits::in);
    rightDistance->objectDistance(vex::distanceUnits::in);

    switch (Autonomous::strategy)
    {
    case Strategy::None:
        break;
    case Strategy::LoaderRoller:
        // move left to the roller
        rollRoller();
        break;
    case Strategy::SideRoller:
        // move right to the roller
        sensorStrafe(26);
        rollRoller();
        break;
    }
}