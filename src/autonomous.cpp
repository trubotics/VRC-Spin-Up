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

Autonomous::Autonomous(MecanumDriveTrain &drive, Shooter &shooter, RollerRoller &roller,
                       distance &leftDistance, distance &rightDistance, inertial &inertialSensor)
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
    vex::distance *selectedSensor;
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

// Rotate using the inertial sensor
// Use negative degrees to rotate left
void Autonomous::sensorRotate(double deltaAngle, double speed)
{
    double initialRotation = inertialSensor->rotation(vex::rotationUnits::deg);

    // rotate
    if (deltaAngle > 0)
    {
        while (inertialSensor->rotation(vex::rotationUnits::deg) - initialRotation < deltaAngle)
        {
            drive->drive(0, 0, speed);
        }
    }
    else
    {
        while (inertialSensor->rotation(vex::rotationUnits::deg) - initialRotation > deltaAngle)
        {
            drive->drive(0, 0, -speed);
        }
    }

    drive->drive(0, 0, 0);
}

// Rolls the roller
void Autonomous::rollRoller()
{
    // "creep" to roller
    drive->drive(-30, 0, 0);
    waitUntil(drive->getAvgTorque() > 0.5);

    // calibrate team color
    roller->calibrateTeamColor();

    // continue driving into the roller very gently and roll the roller
    drive->drive(-10, 0, 0);
    roller->rollRoller(directionType::fwd);
    wait(500, timeUnits::msec);
    roller->stopRoller();
    drive->drive(0, 0, 0);
}

// Spins up the flywheel and fires a disk
void Autonomous::fireDisk(int count, double velocity)
{
    // spin up flywheel to speed
    shooter->setTargetVelocity(velocity);
    shooter->spinUp();

    // fire disks when possible
    for (int i = 0; i < count; i++)
    {
        // manually wait for flywheel to spin up
        wait(2, sec);
        waitUntil(shooter->fireDisk(true)); // wait until disk is fired successfully
    }

    // stop flywheel
    shooter->stop();
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
        sensorStrafe(-24);
        rollRoller();
        // move forward slightly and fire two disks
        drive->driveFor(100, 0, 0, 0.25);
        fireDisk();
        break;
    case Strategy::SideRoller:
        // move right to the roller
        sensorStrafe(24);
        rollRoller();
        // move forward slightly and fire two disks
        drive->driveFor(100, 0, 0, 0.25);
        fireDisk();
        break;
    }
}