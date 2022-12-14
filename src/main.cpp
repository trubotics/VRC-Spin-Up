/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

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
bool fireDisk(bool skipPreCheck = false) {
  // check preconditions: cooldown (100 ms), flywheel speed
  if (!skipPreCheck && // precheck override
  (lastFiringTime + 100 > Brain.timer(timeUnits::msec) // cooldown
  || flywheel.velocity(vex::velocityUnits::pct) < 90)) { // flywheel speed
    return false;
  }

  firingPiston.set(true);
  firingPiston.set(false);

  lastFiringTime = Brain.timer(timeUnits::msec);
  return true;
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  Brain.Screen.print("Battery: %d%", Brain.Battery.capacity());

  // List disconnected devices
  Brain.Screen.print("Disconnected devices:");
  Brain.Screen.newLine();
  
  device devices[] = {frontLeft, backLeft, frontRight, backRight, intake, flywheelFront, flywheelBack};
  char* deviceNames[] = {"Drivetrain: Front Left", "Drivetrain: Back Left", "Drivetrain: Front Right", "Drivetrain: Back Right", "Intake", "Flywheel: Front", "Flywheel: Back"};
  for (int i = 0; i < sizeof(devices); i++) {
    if (!devices[i].installed()) {
      Brain.Screen.print(deviceNames[i]);
      Brain.Screen.newLine();
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  Brain.Screen.clearScreen();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
  // User control code here, inside the loop
  Brain.Screen.clearScreen();

  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // arcade drive
    int forward = primaryController.Axis3.position();
    int right = primaryController.Axis4.position();
    int turn = primaryController.Axis1.position();

    frontRight.spin(vex::forward, forward - right + turn, vex::percent);
    frontLeft.spin(vex::forward, forward + right - turn, vex::percent);
    backRight.spin(vex::forward, forward + right + turn, vex::percent);
    backLeft.spin(vex::forward, forward - right - turn, vex::percent);

    // fire disk
    if (primaryController.ButtonA.pressing()) {
      fireDisk();
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
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
