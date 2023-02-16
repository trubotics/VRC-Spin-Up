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

Autonomous::Autonomous(MecanumDriveTrain &drive, Shooter &shooter, motor_group &flywheel, motor &roller)
{
    this->drive = &drive;
    this->shooter = &shooter;
    this->flywheel = &flywheel;
    this->roller = &roller;
}

Strategy Autonomous::getStrategy()
{
    return strategy;
}
std::string Autonomous::getStrategyString()
{
    return strategyMap[strategy];
}
int Autonomous::getStrategyCount()
{
    return strategyMap.size();
}
void Autonomous::setStrategy(Strategy strategy)
{
    Autonomous::strategy = strategy;
}

// Rolls the roller
void Autonomous::rollRoller()
{
    // "creep" to roller
    drive->drive(-30, 0, 0);
    waitUntil(drive->getAvgTorque() > 0.5);

    // continue driving into the roller very gently and roll the roller
    drive->drive(-10, 0, 0);
    roller->spinFor(0.5, vex::rotationUnits::rev, 25, vex::velocityUnits::pct);
    drive->drive(0, 0, 0);
}

// Spins up the flywheel and fires a disk
void Autonomous::fireDisk(int count, double velocity)
{
    // spin up flywheel to speed
    shooter->setTargetVelocity(velocity);
    flywheel->spin(vex::directionType::fwd);

    // fire disks when possible
    for (int i = 0; i < count; i++)
    {
        // manually wait for flywheel to spin up
        wait(1, sec);
        waitUntil(shooter->fireDisk(true)); // wait until disk is fired successfully
    }

    // stop flywheel
    flywheel->stop();
}

void Autonomous::run()
{
    switch (Autonomous::strategy)
    {
    case Strategy::None:
        break;
    case Strategy::LoaderRoller:
        // move a little left to get into position
        drive->driveFor(0, -100, 0, 0.5);
        rollRoller();
        // move forward slightly and fire two disks
        drive->driveFor(100, 0, 0, 0.25);
        fireDisk();
        break;
    case Strategy::SideRoller:
        // move one tile right to roller
        drive->driveFor(0, 100, 0, 2.5);
        rollRoller();
        // move forward slightly and fire two disks
        drive->driveFor(100, 0, 0, 0.25);
        fireDisk();
        break;
    }
}