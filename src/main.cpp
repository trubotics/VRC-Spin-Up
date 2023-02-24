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

void bindControls()
{
  // Drivetrain Binds
  // Toggle drivetrain lock (toggles motors between brake and hold) [X]
  primaryController.ButtonX.pressed(
      []()
      {
        drive.setMotorLock(!drive.getMotorLock());
      });

  // Smart Roller Binds
  // Start smart roller (Player 1) [A+]
  primaryController.ButtonA.pressed(
      []()
      {
        roller.rollRoller();
      });
  // Start smart roller (Player 2) [L1]
  secondaryController.ButtonL1.pressed(
      []()
      {
        roller.rollRoller();
      });
  // Stop smart roller (Player 1) [A-]
  primaryController.ButtonA.released(
      []()
      {
        roller.stopRoller();
      });
  // Stop smart roller (Player 2) [L2]
  secondaryController.ButtonL2.pressed(
      []()
      {
        roller.stopRoller();
      });

  // Manual Roller Binds
  // Roll Upwards (Player 1) [L1+]
  primaryController.ButtonL1.pressed(
      []()
      {
        roller.rollRoller(true, primaryController.ButtonL2.pressing());
      });
  // Roll Downwards (Player 1) [L2+]
  primaryController.ButtonL2.pressed(
      []()
      {
        roller.rollRoller(primaryController.ButtonL1.pressing(), true);
      });
  // Stop roller (Player 1) [L1-&L2-]
  primaryController.ButtonL1.released(
      []()
      {
        roller.rollRoller(false, primaryController.ButtonL2.pressing());
      });
  primaryController.ButtonL2.released(
      []()
      {
        roller.rollRoller(primaryController.ButtonL1.pressing(), false);
      });

  // Intake Binds
  // Take in (Player 1) [R1+]
  primaryController.ButtonR1.pressed(
      []()
      {
        intake.spinIntake(true, primaryController.ButtonR2.pressing());
      });
  // Spit out (Player 1) [R2+]
  primaryController.ButtonR2.pressed(
      []()
      {
        intake.spinIntake(primaryController.ButtonR1.pressing(), true);
      });
  // Stop intake (Player 1) [R1-&R2-]
  primaryController.ButtonR1.released(
      []()
      {
        intake.spinIntake(false, primaryController.ButtonR2.pressing());
      });
  primaryController.ButtonR2.released(
      []()
      {
        intake.spinIntake(primaryController.ButtonR1.pressing(), false);
      });

  // Shooter Binds (Player 2)
  // Fire Disk [R1]
  secondaryController.ButtonR1.pressed(
      []()
      {
        shooter.fireDisk();
      });
  // Spin up flywheel [R2+]
  secondaryController.ButtonR2.pressed(
      []()
      {
        shooter.spinUp();
      });
  // Stop flywheel [R2-]
  secondaryController.ButtonR2.released(
      []()
      {
        shooter.stop();
      });
  // Set flywheel to the max speed [Up]
  secondaryController.ButtonUp.pressed(
      []()
      {
        shooter.setRelativeTargetVelocity(1);
      });
  // Set flywheel to the min speed [Down]
  secondaryController.ButtonDown.pressed(
      []()
      {
        shooter.setRelativeTargetVelocity(0);
      });
  // Set the flywheel to 1/3 speed [Left]
  secondaryController.ButtonLeft.pressed(
      []()
      {
        shooter.setRelativeTargetVelocity(1/3);
      });
  // Set the flywheel to 2/3 speed [Right]
  secondaryController.ButtonRight.pressed(
      []()
      {
        shooter.setRelativeTargetVelocity(2/3);
      });

  // Expansion Binds
  // Try expanding [Y]
  primaryController.ButtonY.pressed(
      []()
      {
        expansion.tryExpand();
      });
  secondaryController.ButtonY.pressed(
      []()
      {
        expansion.tryExpand();
      });
}
void userControl(void)
{
  // User control code here, inside the loop
  Brain.Screen.clearScreen();
  drive.setMotorLock(false); // unlock the drivetrain

  // callback controls
  bindControls();

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

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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

    // disable preauton config menu when enabled
    if (Competition.isEnabled())
    {
      Brain.Screen.pressed([]() {});
    }

    // call update functions
    shooter.updateVelocity();

    // display git commit hash
    Brain.Screen.setCursor(20, 1);
    Brain.Screen.print("Git Commit: %s", VERSION);
  }
}