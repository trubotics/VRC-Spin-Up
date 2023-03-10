/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <buildDefines.h>
#include <autonomous.h>
#include <mecanumDrivetrain.h>
#include <intake.h>
#include <shooter.h>
#include <rollerRoller.h>
#include <expansion.h>

using namespace vex;

// A global instance of competition
competition Competition = competition();

// define your global instances of motors and other devices here

// brain
brain Brain = brain();

// controller
controller primaryController = controller(primary);
controller secondaryController = controller(partner);

// sensors
distance leftDist = distance(PORT17);
distance rightDist = distance(PORT7);
inertial inertialSensor = inertial(PORT3);
optical opticalSensor = optical(PORT20);

// drive train
MecanumDriveTrain drive = MecanumDriveTrain(PORT15, true, PORT16, true, PORT5, false, PORT6, false);

// intake
Intake intake = Intake(PORT8);

// roller
RollerRoller roller = RollerRoller(PORT10, opticalSensor);

// flywheel
motor flywheelFront = motor(PORT11, ratio36_1, false);
motor flywheelBack = motor(PORT12, ratio36_1, false);
motor_group flywheel = motor_group(flywheelFront, flywheelBack);

// firing piston
pneumatics firingPiston = pneumatics(Brain.ThreeWirePort.A);
Shooter shooter = Shooter(Brain, flywheel, firingPiston);

// endgame expansion
pneumatics expansionPiston = pneumatics(Brain.ThreeWirePort.B);
Expansion expansion = Expansion(expansionPiston, primaryController, secondaryController);

// initialize autonomous class
Autonomous autonomous = Autonomous(drive, shooter, roller,
                                   leftDist, rightDist, inertialSensor);

/* Global Functions */

void displayStrategy()
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonomous.getStrategyString().c_str());
}

// Creates an onscreen button
void createButton(int x, int y, int width, int height, std::string text, color penColor, color fillColor)
{
  Brain.Screen.setPenColor(penColor);
  Brain.Screen.setFillColor(fillColor);

  Brain.Screen.drawRectangle(x, y, width, height);
  Brain.Screen.printAt(x + width / 2, y + height / 2, text.c_str());

  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFillColor(color::black);
}

void drawStrategyMenu()
{
  displayStrategy();
  Brain.Screen.newLine();
  // create a rectangle with a strategy name for each strategy
  // strategies: None, Loader Roller, Side Roller
  createButton(0, 100, 100, 50, "None", color::black, color::white);
  createButton(100, 100, 100, 50, "Loader Roller", color::black, color::yellow);
  createButton(200, 100, 100, 50, "Side Roller", color::black, color::purple);
}

void selectStrategy()
{
  // Check if along button row
  if (Brain.Screen.yPosition() < 100 || Brain.Screen.yPosition() > 150)
    return;

  // Set the strategy based on the location pressed
  if (Brain.Screen.xPosition() < 100)
  {
    autonomous.setStrategy(Strategy::None);
  }
  else if (Brain.Screen.xPosition() < 200)
  {
    autonomous.setStrategy(Strategy::LoaderRoller);
  }
  else if (Brain.Screen.xPosition() < 300)
  {
    autonomous.setStrategy(Strategy::SideRoller);
  }
}

void pre_auton(void)
{
  // flywheel
  // velocity managed by shooter
  flywheel.setStopping(vex::brakeType::coast);

  // pid loop
  thread pidThread = thread([]()
                            { shooter.pidLoop(); });

  displayStrategy();
  drawStrategyMenu();
  Brain.Screen.pressed(
      []()
      {
        selectStrategy();
        displayStrategy();
        drawStrategyMenu();
      });
}

void userControl(void)
{
  // User control code here, inside the loop
  Brain.Screen.clearScreen();
  drive.setMotorLock(false); // unlock the drivetrain

  bool intakeDebounce = false;

  // Toggle drivetrain lock (toggles motors between brake and hold) [X]
  primaryController.ButtonX.pressed(
      []()
      {
        drive.setMotorLock(!drive.getMotorLock());
      });

  primaryController.ButtonA.pressed(
      []()
      {
        roller.rollRoller();
      });

  while (1)
  {
    // arcade drive (left stick controls forward/backward and strafe, right stick controls turning) [LS, RS]
    // primary controller drives intake forward (negative values), secondary drives shooter forwards (positive values)
    int forward = -primaryController.Axis3.position();
    int strafe = -primaryController.Axis4.position();
    int turn = primaryController.Axis1.position();
    if (abs(forward) + abs(strafe) + abs(turn) <= 9) // if primary unused, use secondary values
    {
      forward = secondaryController.Axis3.position();
      strafe = secondaryController.Axis4.position();
      turn = secondaryController.Axis1.position();
    }
    if (primaryController.ButtonB.pressing() || secondaryController.ButtonB.pressing()) // slow mode (1/3 speed) [B]
    {
      forward /= 3;
      strafe /= 3;
      turn /= 3;
    }
    drive.drive(forward, strafe, turn);

    // Roller binds
    if (primaryController.ButtonL1.pressing() || primaryController.ButtonL2.pressing())
    {
      roller.rollRoller(primaryController.ButtonL1.pressing(), primaryController.ButtonL2.pressing());
    }
    else if (!primaryController.ButtonA.pressing())
    {
      roller.stopRoller();
    }

    // Intake binds
    intake.spinIntake(primaryController.ButtonR1.pressing(), primaryController.ButtonR2.pressing());

    // Shooter binds
    if (secondaryController.ButtonR1.pressing())
    {
      shooter.fireDisk();
    }
    if (secondaryController.ButtonR2.pressing())
    {
      shooter.updateVelocity();
      shooter.spinUp();
    }
    else if (secondaryController.ButtonL2.pressing())
    {
      flywheel.setVelocity(100, vex::percentUnits::pct);
      flywheel.spin(vex::directionType::rev);
    }
    else
    {
      shooter.stop();
    }

    if (secondaryController.ButtonUp.pressing())
    {
      shooter.setRelativeTargetVelocity(1);
    }
    if (secondaryController.ButtonDown.pressing())
    {
      shooter.setRelativeTargetVelocity(0);
    }
    if (secondaryController.ButtonLeft.pressing())
    {
      shooter.setRelativeTargetVelocity(1 / 3);
    }
    if (secondaryController.ButtonRight.pressing())
    {
      shooter.setRelativeTargetVelocity(2 / 3);
    }

    // Expansion binds
    if (primaryController.ButtonY.pressing() || secondaryController.ButtonY.pressing())
    {
      expansion.tryExpand();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

bool debounce = false;
int tuningIndex = 0; // The current value being tuned
// 0 = P
// 1 = I
// 2 = D

void tuningAdjuster()
{
  // Temporary tuning mode
  if (primaryController.ButtonR1.pressing() && primaryController.ButtonR2.pressing())
  {
    double increment = 0.01;
    if (primaryController.ButtonA.pressing())
    {
      increment = 0.0001;
    }
    if (primaryController.ButtonB.pressing())
    {
      increment = 0.001;
    }
    if (primaryController.ButtonX.pressing())
    {
      increment = 0.1;
    }
    if (primaryController.ButtonY.pressing())
    {
      increment = 1;
    }

    if (!debounce)
    {
      // change the tuning index
      if (primaryController.ButtonLeft.pressing())
      {
        tuningIndex--;
        if (tuningIndex < 0)
        {
          tuningIndex = 2;
        }
        debounce = true;
      }
      else if (primaryController.ButtonRight.pressing())
      {
        tuningIndex++;
        if (tuningIndex > 2)
        {
          tuningIndex = 0;
        }
        debounce = true;
      }
      // change the value
      else if (primaryController.ButtonUp.pressing())
      {
        shooter.changePID(tuningIndex, increment);
        debounce = true;
      }
      else if (primaryController.ButtonDown.pressing())
      {
        shooter.changePID(tuningIndex, -increment);
        debounce = true;
      }
    }
  }
}

void debounceResetter()
{
  if (!(primaryController.ButtonUp.pressing() || primaryController.ButtonDown.pressing() || primaryController.ButtonLeft.pressing() || primaryController.ButtonRight.pressing()))
  {
    debounce = false;
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Run the pre-autonomous function.
  pre_auton();

  // Set up competition functions
  Competition.autonomous(
      []()
      {
        Brain.Screen.clearScreen();
        autonomous.run();
      });

  Competition.drivercontrol(userControl);

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);

    tuningAdjuster();
    debounceResetter();

    // disable preauton config menu when enabled
    if (Competition.isEnabled())
    {
      Brain.Screen.pressed([]() {});
    }

    // display git commit hash
    Brain.Screen.setCursor(20, 1);
    Brain.Screen.print("Git Commit: %s", VERSION);
  }
}