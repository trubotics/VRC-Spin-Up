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

Autonomous::Autonomous(MecanumDriveTrain &drive, Shooter &shooter, motor &roller)
{
    this->drive = &drive;
    this->shooter = &shooter;
    this->roller = &roller;
}

Strategy Autonomous::getStrategy()
{
    return Autonomous::STRATEGY;
}

// Rolls the roller
void Autonomous::rollRoller()
{
    // "creep" to roller
    drive->drive(15, 0, 0);
    waitUntil(drive->getAvgTorque() > 0.5);

    // continue driving into the roller very gently and roll the roller
    drive->drive(5, 0, 0);
    roller->spinFor(0.5, vex::rotationUnits::rev, 100, vex::velocityUnits::pct);
    drive->drive(0, 0, 0);
}

// Spins up the flywheel and fires a disk
void Autonomous::fireDisk(double velocity)
{
    shooter->setTargetVelocity(velocity);
    waitUntil(shooter->fireDisk()); // wait until disk is fired successfully
}

void Autonomous::run()
{
    switch (Autonomous::STRATEGY)
    {
    case Strategy::None:
        break;
    case Strategy::LoaderRoller:
        break;
    case Strategy::SideRoller:
        break;
    }
}