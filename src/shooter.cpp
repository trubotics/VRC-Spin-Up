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

//////////////////////////////////////////////////////
/// @brief basic PID control for flywheel velocity ///
//////////////////////////////////////////////////////
double sumError = 0, lastError = 0, lastTime = 0;
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
  }
  else
  {
    double deltaTime = currentTime - lastTime;
    derivative = (error - lastError) / (currentTime - lastTime) * kD;

    sumError += error * (currentTime - lastTime);
  }

  double integral = sumError * kI;

  lastError = error;
  lastTime = currentTime;

  // modify velocity
  double output = proportional + integral + derivative;
  flywheel->setVelocity(targetVelocity + output, vex::velocityUnits::pct); // set new flywheel velocity target

  Brain->Screen.setCursor(2, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("Output Velocity: %f", this->targetVelocity + output);

  Brain->Screen.setCursor(3, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("Measured Velocity: %f", this->flywheel->velocity(vex::velocityUnits::pct));

}
void Shooter::setTargetVelocity(double targetVelocity)
{
  this->targetVelocity = fmin(fmax(targetVelocity, MIN_VELOCITY), MAX_VELOCITY); // clamp target velocity

  sumError = lastError = lastTime = 0;

  Brain->Screen.setCursor(1, 1);
  Brain->Screen.clearLine();
  Brain->Screen.print("Target Velocity: %f", this->targetVelocity);
}
void Shooter::setRelativeTargetVelocity(double targetVelocity) // uses the min and max to convert relative to true velocity
{
  double trueVelocity = targetVelocity * VELOCITY_RANGE + MIN_VELOCITY;
  setTargetVelocity(trueVelocity);
}

void Shooter::spinUp() // get it?
{
  //flywheel->setVelocity(0, vex::velocityUnits::pct); // wait for PID function to set velocity
  sumError = lastError = lastTime = 0;
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
  if (!skipPreCheck &&                                                                   // precheck override
      //(Brain->timer(timeUnits::msec) - lastFiringTime <= 75                            // firing cooldown (50 ms)
      /*||*/ std::abs((*flywheel).velocity(vex::velocityUnits::pct) - targetVelocity) > 5)//) // flywheel speed (+- 5%)
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