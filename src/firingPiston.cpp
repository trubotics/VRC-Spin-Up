/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       firingPiston.cpp                                          */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  Pnematics are hard                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <firingPiston.h>

using namespace vex;

FiringPiston::FiringPiston(brain Brain, motor_group flywheel, vex::triport::port port)
{
    this->Brain = &Brain;
    this->flywheel = &flywheel;
    piston = &pneumatics(port);
}

double lastFiringTime = -200; // The time when the last disk was fired
void FiringPiston::fireDisk(bool skipPreCheck = false)
{
  // check preconditions: cooldown (100 ms), flywheel speed
  if (!skipPreCheck &&                                     // precheck override
      (lastFiringTime + 200 > (*Brain).timer(timeUnits::msec) // cooldown
       || (*flywheel).velocity(vex::velocityUnits::pct) < 90)) // flywheel speed
  {
    return;
  }

  (*piston).set(true);
}

// Check if the piston should retract (after 100 ms of firing)
void FiringPiston::checkPistonRetract()
{
    if (lastFiringTime + 100 > (*Brain).timer(timeUnits::msec))
    {
        (*piston).set(false);
    }
}