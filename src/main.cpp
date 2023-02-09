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
#include <shooter.h>

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

// flywheel
motor flywheelFront = motor(PORT11, ratio36_1, false);
motor flywheelBack = motor(PORT12, ratio36_1, false);
motor_group flywheel = motor_group(flywheelFront, flywheelBack);

// firing piston
Shooter shooter = Shooter(Brain, flywheel, Brain.ThreeWirePort.A);

/* Global Functions */

void pre_auton(void)
{
  // flywheel
  // velocity managed by shooter
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

  // callback controls
  primaryController.ButtonA.pressed( // toggle inverted controls (drive with the intake forward) [A]
      []() {
        driveInverted = !driveInverted;
      }
  );
  primaryController.ButtonUp.pressed( // increase flywheel speed (10% increments) [Up]
      []() {
        shooter.changeTargetVelocity(10);
      }
  );
  primaryController.ButtonDown.pressed( // decrease flywheel speed (10% increments) [Down]
      []() {
        shooter.changeTargetVelocity(-10);
      }
  );
  primaryController.ButtonLeft.pressed( // set flywheel speed to the minimum value [Left]
      []() {
        shooter.setTargetVelocity(0); // will be overriden by the shooter class
      }
  );
  primaryController.ButtonRight.pressed( // set flywheel speed to the maximum value [Right]
      []() {
        shooter.setTargetVelocity(100);
      }
  );
  primaryController.ButtonR1.pressed( // fire disk (there is a list of preconditions specified in the class) [R1]
      []() {
        shooter.fireDisk();
      }
  );

  while (1)
  {
    // arcade drive (left stick controls forward/backward and strafe, right stick controls turning) [LS, RS]
    int forward = primaryController.Axis3.position();
    int strafe = primaryController.Axis4.position();
    int turn = primaryController.Axis1.position();
    if (driveInverted) // check if controls should be inverted (drive intake forward) [A]
    {
      forward *= -1;
      strafe *= -1;
    }
    if (primaryController.ButtonX.pressing()) // slow mode (1/3 speed) [X]
    {
      forward /= 3;
      strafe /= 3;
      turn /= 3;
    }
    drive.drive(forward, strafe, turn);

    // intake (left trigger; top button takes in, bottom button reverses) [L1/L2]
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

    // roller spinner (left triggers with X held; continues to spin until X is released; top -> up, bot -> down) [L1/L2 + X]
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

    // spin flywheel (hold the button to start spinning, release to stop) [R2]
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
  }
}
