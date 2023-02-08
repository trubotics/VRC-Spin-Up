/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       shooter.cpp                                          */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  Pnematics are hard (handles the pnematic disk shooter)    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <shooter.h>

using namespace vex;

Shooter::Shooter(brain Brain, motor_group flywheel, vex::triport::port port)
{
    this->Brain = &Brain;
    this->flywheel = &flywheel;
    this->flywheel->setVelocity(targetVelocity, vex::velocityUnits::pct); // set initial flywheel velocity
    pneumatics p = pneumatics(port);
    this->piston = &p;
}

void Shooter::setTargetVelocity(double targetVelocity)
{
    this->targetVelocity = targetVelocity;
    this->flywheel->setVelocity(targetVelocity, vex::velocityUnits::pct);
}
void Shooter::changeTargetVelocity(double deltaVelocity)
{
    this->targetVelocity += deltaVelocity;
    this->flywheel->setVelocity(targetVelocity, vex::velocityUnits::pct);

    Brain->Screen.clearScreen();
    Brain->Screen.setCursor(1, 1);
    Brain->Screen.print("Target Velocity: %f", targetVelocity);
}

void Shooter::fireDisk(bool skipPreCheck)
{
  // check preconditions: firing cooldown, flywheel speed
  if (!skipPreCheck && // precheck override
      ((*Brain).timer(timeUnits::msec) - lastFiringTime <= 400 // firing cooldown (400 ms)
      || abs((*flywheel).velocity(vex::velocityUnits::pct) - targetVelocity) > 10)) // flywheel speed (+- 10%)
  {
    return; // failed prechecks
  }

  lastFiringTime = (*Brain).timer(timeUnits::msec); // update last firing time

  // fire disk (extend piston and retract after 200 ms)
  (*piston).set(true);
  wait(200, msec); // wait for piston to extend fully
  (*piston).set(false);
}