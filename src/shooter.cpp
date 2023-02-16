/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       shooter.cpp                                               */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  Pnematics are hard (handles the pnematic disk shooter)    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath>
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
    this->targetVelocity = fmin(fmax(targetVelocity, 70), 100); // clamp target velocity between 70% and 100%
    this->flywheel->setVelocity(this->targetVelocity, vex::velocityUnits::pct);

    Brain->Screen.clearScreen();
    Brain->Screen.setCursor(1, 1);
    Brain->Screen.print("Target Velocity: %f", this->targetVelocity);
}
void Shooter::changeTargetVelocity(double deltaVelocity)
{
    setTargetVelocity(targetVelocity + deltaVelocity);
}

bool Shooter::fireDisk(bool skipPreCheck)
{
  // check preconditions: firing cooldown, flywheel speed
  if (!skipPreCheck && // precheck override
      ((*Brain).timer(timeUnits::msec) - lastFiringTime <= 75 // firing cooldown (75 ms)
      || std::abs((*flywheel).velocity(vex::velocityUnits::pct) - targetVelocity) > 5)) // flywheel speed (+- 5%)
  {
    return false; // failed prechecks
  }

  lastFiringTime = (*Brain).timer(timeUnits::msec); // update last firing time

  // fire disk (extend piston and retract after 75 ms)
  (*piston).set(true);
  wait(75, msec); // wait for piston to extend fully
  (*piston).set(false);

  return true; // success
}