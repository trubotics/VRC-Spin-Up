/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       firingPiston.cpp                                          */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  Pnematics are hard (handles the pnematic disk shooter)    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <firingPiston.h>

using namespace vex;

FiringPiston::FiringPiston(brain Brain, motor_group flywheel, vex::triport::port port)
{
    this->Brain = &Brain;
    this->flywheel = &flywheel;
    pneumatics p = pneumatics(port);
    this->piston = &p;
}

void FiringPiston::fireDisk(bool skipPreCheck)
{
  // check preconditions: firing cooldown, flywheel speed
  if (!skipPreCheck && // precheck override
      (lastFiringTime + 500 > (*Brain).timer(timeUnits::msec) // firing cooldown (500 ms)
      || (*flywheel).velocity(vex::velocityUnits::pct) < 90)) // flywheel speed
  {
    return; // failed prechecks
  }

  lastFiringTime = (*Brain).timer(timeUnits::msec); // update last firing time

  // fire disk (extend piston and retract after 200 ms)
  (*piston).set(true);
  wait(200, msec); // wait for piston to extend fully
  (*piston).set(false);
}