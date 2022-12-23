/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "mecanumDrivetrain.cpp"

using namespace vex;

// A global instance of competition
competition Competition = competition();

// define your global instances of motors and other devices here

// brain
brain Brain = brain();

// controller
controller primaryController = controller(primary);

// drive train
motor frontLeft = motor(PORT1, ratio18_1, false);
motor backLeft = motor(PORT2, ratio18_1, false);
motor frontRight = motor(PORT3, ratio18_1, false);
motor backRight = motor(PORT4, ratio18_1, false);
MecanumDriveTrain drive = MecanumDriveTrain(PORT1, PORT2, PORT3, PORT4);

// intake
motor intake = motor(PORT7, ratio6_1, false);

// firing piston
pneumatics firingPiston = pneumatics(Brain.ThreeWirePort.A);

// flywheel
motor flywheelFront = motor(PORT5, ratio36_1, false);
motor flywheelBack = motor(PORT6, ratio36_1, false);
motor_group flywheel = motor_group(flywheelFront, flywheelBack);

/* Global Functions */

double lastFiringTime = -100; // The time when the last disk was fired
// Fires a disk if the flywheel is spinning fast enough and the cooldown has passed, returns true if a disk was fired
bool fireDisk(bool skipPreCheck = false)
{
  // check preconditions: cooldown (100 ms), flywheel speed
  if (!skipPreCheck &&                                     // precheck override
      (lastFiringTime + 100 > Brain.timer(timeUnits::msec) // cooldown
       || flywheel.velocity(vex::velocityUnits::pct) < 90))
  { // flywheel speed
    return false;
  }

  firingPiston.set(true);
  firingPiston.set(false);

  lastFiringTime = Brain.timer(timeUnits::msec);
  return true;
}

void pre_auton(void)
{
  // flywheel
  flywheel.setVelocity(100, vex::velocityUnits::pct);
  flywheel.setStopping(vex::brakeType::coast);
}

void autonomous(void)
{
  Brain.Screen.clearScreen();
}

void userControl(void)
{
  // User control code here, inside the loop
  Brain.Screen.clearScreen();

  while (1)
  {
    // arcade drive
    int forward = primaryController.Axis3.position();
    int strafe = primaryController.Axis4.position();
    int turn = primaryController.Axis1.position();
    drive.drive(forward, strafe, turn);

    // intake
    if (primaryController.ButtonL1.pressing())
    {
      intake.spin(vex::forward);
    }
    else if (primaryController.ButtonL2.pressing())
    {
      intake.spin(vex::reverse);
    }
    else
    {
      intake.stop();
    }

    // fire disk
    if (primaryController.ButtonR1.pressing())
    {
      fireDisk();
    }

    // spin flywheel
    if (primaryController.ButtonL1.pressing())
    {
      flywheel.spin(vex::forward);
    }
    else
    {
      flywheel.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
