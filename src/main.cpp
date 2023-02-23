/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <autonomous.h>
#include <mecanumDrivetrain.h>
#include <shooter.h>
#include <rollerRoller.h>

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
motor intake = motor(PORT8, ratio6_1, true);

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
pneumatics expansion = pneumatics(Brain.ThreeWirePort.B);

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
  // intake
  intake.setVelocity(100, vex::velocityUnits::pct);

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

  // callback controls
  primaryController.ButtonX.pressed( // toggle drivetrain lock (sets brake mode to hold) [X]
      []()
      {
        drive.setMotorLock(!drive.getMotorLock());
      });
  primaryController.ButtonA.pressed( // start smart roller [A]
      []()
      {
        roller.rollRoller();
      });

  secondaryController.ButtonUp.pressed( // increase flywheel speed (10% increments) [Up]
      []()
      {
        shooter.setTargetVelocity(80);
      });
  secondaryController.ButtonDown.pressed( // decrease flywheel speed (10% increments) [Down]
      []()
      {
        shooter.setTargetVelocity(50);
      });
  secondaryController.ButtonLeft.pressed( // set flywheel speed to the minimum value [Left]
      []()
      {
        shooter.setTargetVelocity(60); // will be overriden by the shooter class
      });
  secondaryController.ButtonRight.pressed( // set flywheel speed to the maximum value [Right]
      []()
      {
        shooter.setTargetVelocity(70);
      });
  secondaryController.ButtonR1.pressed( // fire disk (there is a list of preconditions specified in the class) [R1]
      []()
      {
        shooter.fireDisk(true);
      });
  secondaryController.ButtonL1.pressed( // start smart roller [L1]
      []()
      {
        roller.rollRoller();
      });
  secondaryController.ButtonL2.pressed( // stop roller [L2]
      []()
      {
        roller.stopRoller();
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

    // intake (left trigger; top button takes in, bottom button reverses) [R1/R2]
    if (primaryController.ButtonR1.pressing())
    {
      intake.spin(vex::forward);
    }
    else if (primaryController.ButtonR2.pressing())
    {
      intake.spin(vex::reverse);
    }
    else
    {
      intake.stop();
    }

    // backup manual roller spinner (top -> up, bot -> down) for primary [L1/L2]
    if (primaryController.ButtonL1.pressing())
    {
      roller.rollRoller(vex::forward);
    }
    else if (primaryController.ButtonL2.pressing())
    {
      roller.rollRoller(vex::reverse);
    }
    else if (!primaryController.ButtonA.pressing())
    {
      roller.stopRoller();
    }

    // spin flywheel (hold the button to start spinning, release to stop) [R2]
    if (secondaryController.ButtonR2.pressing())
    {
      shooter.spinUp();
      // update the flywheel velocity with PID
      shooter.updateVelocity();
    }
    else
    {
      shooter.stop();
    }

    if (primaryController.ButtonY.pressing() || secondaryController.ButtonY.pressing()) // endgame expansion (both) [Y]
    {
      double pressedStartTime = Brain.timer(timeUnits::msec);
      primaryController.rumble("-");
      secondaryController.rumble("-");
      waitUntil(!(primaryController.ButtonY.pressing() && secondaryController.ButtonY.pressing()) || Brain.timer(timeUnits::msec) - pressedStartTime > 1000); // wait until the button is released or 1 second has passed

      // only activate if the buttons were held for 1 second
      if (Brain.timer(timeUnits::msec) - pressedStartTime > 1000)
      {
        expansion.set(true);
      }
    };

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
  Competition.autonomous([]()
                         { autonomous.run(); });

  Competition.drivercontrol(userControl);

  int funnyTimer = 0;

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
    funnyTimer += 100;

    // disable preauton config menu when enabled
    if (Competition.isEnabled())
    {
      Brain.Screen.clearScreen();
      Brain.Screen.pressed([]() {});
    }

    // call update functions
    shooter.updateVelocity();

    Brain.Screen.setCursor(20, 0);
    Brain.Screen.print(funnyTimer);
    Brain.Screen.newLine();
    Brain.Screen.print(Brain.Timer.system());
    Brain.Screen.newLine();
    Brain.Screen.print(Brain.timer(timeUnits::msec));
  }
}