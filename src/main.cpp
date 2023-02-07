/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <mecanumDrivetrain.h>
#include <firingPiston.h>

using namespace vex;

// A global instance of competition
competition Competition = competition();

// define your global instances of motors and other devices here

// brain
brain Brain = brain();

// controller
controller primaryController = controller(primary);

// drive train
MecanumDriveTrain drive = MecanumDriveTrain(PORT15, true, PORT16, true, PORT5, false, PORT6, false);
bool driveInverted = false;

// intake
motor intake = motor(PORT8, ratio6_1, true);

// roller
motor roller = motor(PORT10, ratio6_1, false);

// firing piston
FiringPiston firingPiston = FiringPiston(Brain, flywheel, Brain.ThreeWirePort.A);

// flywheel
motor flywheelFront = motor(PORT11, ratio36_1, false);
motor flywheelBack = motor(PORT12, ratio36_1, false);
motor_group flywheel = motor_group(flywheelFront, flywheelBack);

/* Global Functions */

void pre_auton(void)
{
  // flywheel
  flywheel.setVelocity(100, vex::velocityUnits::pct);
  flywheel.setStopping(vex::brakeType::coast);

  //intake
  intake.setVelocity(100, vex::velocityUnits::pct);

  // roller
  roller.setVelocity(25, vex::velocityUnits::pct);
}

void autonomous(void)
{
  Brain.Screen.clearScreen();
}

void userControl(void)
{
  // User control code here, inside the loop
  Brain.Screen.clearScreen();

  // callbacks
  primaryController.ButtonA.pressed(
      []() {
        driveInverted = !driveInverted;
      }
  );

  while (1)
  {
    // arcade drive
    int forward = primaryController.Axis3.position();
    int strafe = primaryController.Axis4.position();
    int turn = primaryController.Axis1.position();
    if (driveInverted) // check if controls should be inverted (intake forward)
    {
      forward *= -1;
      strafe *= -1;
    }
    if (primaryController.ButtonX.pressing()) // slow mode (1/3 speed)
    {
      forward /= 3;
      strafe /= 3;
      turn /= 3;
    }
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

    // roller
    if (!primaryController.ButtonX.pressing())
    {
      roller.stop();
    }
    else {
      if (primaryController.ButtonL1.pressing())
      {
        roller.spin(vex::forward);
      }
      else if (primaryController.ButtonL2.pressing())
      {
        roller.spin(vex::reverse);
      }
    }

    // fire disk
    if (primaryController.ButtonR1.pressing())
    {
      firingPiston.fireDisk();
    }

    // spin flywheel
    if (primaryController.ButtonR2.pressing())
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
    firingPiston.checkPistonRetract();
  }
}
