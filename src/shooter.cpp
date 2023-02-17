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
  this->flywheel->setVelocity(0, vex::velocityUnits::pct); // set initial flywheel velocity
  pneumatics p = pneumatics(port);
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

  Brain->Screen.clearScreen();
  Brain->Screen.setCursor(1, 1);
  Brain->Screen.print("kP: %f", kP);
  Brain->Screen.setCursor(2, 1);
  Brain->Screen.print("kI: %f", kI);
  Brain->Screen.setCursor(3, 1);
  Brain->Screen.print("kD: %f", kD);
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

  // for derivative
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
  }

  // for integral
  sumError += error;
  double integral = sumError * kI;

  lastError = error;
  lastTime = currentTime;

  // modify velocity
  double output = proportional + integral + derivative;
  flywheel->setVelocity(targetVelocity + output, vex::velocityUnits::pct); // set new flywheel velocity target

  // no idea if these print statements correct
  Brain->Screen.clearScreen();
  Brain->Screen.setCursor(1, 1);
  Brain->Screen.print("Modified Target Velocity: %f", targetVelocity + output);
  Brain->Screen.setCursor(2, 1);
  Brain->Screen.print("Measured Velocity: %f", this->flywheel->velocity(vex::velocityUnits::pct));
  Brain->Screen.setCursor(4, 1);
  Brain->Screen.print("PID: %f %f %f", kP, kI, kD);
}
void Shooter::setTargetVelocity(double targetVelocity)
{
    this->targetVelocity = fmin(fmax(targetVelocity, 0), 100); // clamp target velocity between 70% and 100%
  // this->flywheel->setVelocity(this->targetVelocity, vex::velocityUnits::pct);

  sumError = lastError = lastTime = 0;

  Brain->Screen.clearScreen();
  Brain->Screen.setCursor(1, 1);
  Brain->Screen.print("Target Velocity: %f", this->targetVelocity);
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
      ((*Brain).timer(timeUnits::msec) - lastFiringTime <= 75                            // firing cooldown (50 ms)
       || std::abs((*flywheel).velocity(vex::velocityUnits::pct) - targetVelocity) > 5)) // flywheel speed (+- 5%)
  {
    return false; // failed prechecks
  }

  Brain->Screen.clearScreen();
  Brain->Screen.setCursor(1, 1);
  Brain->Screen.print(lastFiringTime);

  lastFiringTime = (*Brain).timer(timeUnits::msec); // update last firing time
  
  Brain->Screen.setCursor(2,1);
  Brain->Screen.print(lastFiringTime);

  // fire disk (extend piston and retract after 75 ms)
  piston->set(true);

  Brain->Screen.setCursor(3,1);
  Brain->Screen.print("Firing");

  wait(75, timeUnits::msec); // wait for piston to extend fully

  Brain->Screen.setCursor(4,1);
  Brain->Screen.print("Retracting");
  
  piston->set(false);

  return true; // success
}