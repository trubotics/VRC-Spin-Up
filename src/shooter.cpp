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

Shooter::Shooter(brain Brain, motor_group flywheel, pneumatics p)
{
  this->Brain = &Brain;
  this->flywheel = &flywheel;
  this->flywheel->setVelocity(0, vex::velocityUnits::pct); // set initial flywheel velocity
  this->piston = &p;
}

// temp
void Shooter::changePID(int index, double deltaPID)
{
  switch (index)
  {
  case 0:
    kP += deltaPID;
    break;
  case 1:
    kI += deltaPID;
    break;
  case 2:
    kD += deltaPID;
    break;
  }

  Brain->Screen.setCursor(2, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("PID: %f %f %f", kP, kI, kD);
}

//////////////////////////////////////////////////////
/// @brief basic PID control for flywheel velocity ///
//////////////////////////////////////////////////////
double lastError, lastTime, integralTotal;
void Shooter::updateVelocity()
{
  // don't need to calculate when not spinning
  if (!isSpinning)
    return;

  double error = this->targetVelocity - this->flywheel->velocity(vex::velocityUnits::pct);

  // for proportional
  double proportional = error * kP;

  // for derivative & integral
  double derivative = 0;
  double currentTime = Brain->timer(timeUnits::msec);
  if (lastTime == 0)
  {
    lastTime = currentTime;
    lastError = error;
    integralTotal = 0;
  }
  else
  {
    double deltaTime = currentTime - lastTime;

    double integral = error * deltaTime * kI;
    integralTotal += integral;

    derivative = (error - lastError) / deltaTime * kD;
  }

  lastError = error;
  lastTime = currentTime;

  // modify velocity
  double output = proportional + integralTotal + derivative;
  flywheel->setVelocity(targetVelocity + output, vex::velocityUnits::pct); // set new flywheel velocity target

  // statistic printing
  Brain->Screen.setCursor(2, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("PID: %f %f %f", kP, kI, kD);
  Brain->Screen.newLine();
  Brain->Screen.clearLine();
  Brain->Screen.print("Measured Velocity: %f", this->flywheel->velocity(vex::velocityUnits::pct));
  Brain->Screen.newLine();
  Brain->Screen.clearLine();
  Brain->Screen.print("Modified Target Velocity: %f", targetVelocity + output);
}
void Shooter::setTargetVelocity(double targetVelocity)
{
  this->targetVelocity = fmin(fmax(targetVelocity, MIN_VELOCITY), MAX_VELOCITY); // clamp target velocity

  lastTime = 0;

  Brain->Screen.setCursor(1, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("Target Velocity: %f", this->targetVelocity);
}
void Shooter::setRelativeTargetVelocity(double targetVelocity) // uses the min and max to convert relative to true velocity
{
  double trueVelocity = targetVelocity * VELOCITY_RANGE + MIN_VELOCITY;
  setTargetVelocity(trueVelocity);
}
void Shooter::pidLoop()
{
  while (true)
  {
    if (!isSpinning)
      wait(100, timeUnits::msec);
      continue;
    updateVelocity();
    wait(100, timeUnits::msec);
  }
}

void Shooter::spinUp() // get it?
{
  // flywheel->setVelocity(0, vex::velocityUnits::pct); // wait for PID function to set velocity
  lastTime = 0;
  isSpinning = true;
  flywheel->spin(vex::directionType::fwd);
}
void Shooter::stop()
{
  flywheel->stop();
  isSpinning = false;
}

bool Shooter::fireDisk(bool skipPreCheck)
{
  // check preconditions: firing cooldown, flywheel speed
  if (!skipPreCheck &&                                                                     // precheck override
                                                                                           //(Brain->timer(timeUnits::msec) - lastFiringTime <= 75                            // firing cooldown (50 ms)
      /*||*/ std::abs((*flywheel).velocity(vex::velocityUnits::pct) - targetVelocity) > 5) //) // flywheel speed (+- 5%)
  {
    return false; // failed prechecks
  }

  lastFiringTime = Brain->timer(timeUnits::msec); // update last firing time
  Brain->Screen.setCursor(19, 0);
  Brain->Screen.print(lastFiringTime);

  // fire disk (extend piston and retract after 75 ms)
  piston->set(true);
  wait(75, timeUnits::msec); // wait for piston to extend fully
  piston->set(false);

  return true; // success
}