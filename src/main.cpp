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

using namespace vex;

// A global instance of competition
competition Competition = competition();

// define your global instances of motors and other devices here

// brain
brain Brain = brain();

// controller
controller primaryController = controller(primary);
controller secondaryController = controller(partner);

// drive train
MecanumDriveTrain drive = MecanumDriveTrain(PORT15, true, PORT16, true, PORT5, false, PORT6, false);

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

// initialize autonomous class
Autonomous autonomous = Autonomous(drive, shooter, roller);

/* Global Functions */

void displayStrategy()
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonomous.getStrategyString().c_str());
}

void pre_auton(void)
{
  // flywheel
  // velocity managed by shooter
  flywheel.setStopping(vex::brakeType::coast);
  // intake
  intake.setVelocity(100, vex::velocityUnits::pct);
  // roller
  roller.setVelocity(25, vex::velocityUnits::pct);

  displayStrategy();
  // allow strategy changes during pre-auton
  primaryController.ButtonLeft.pressed( // previous strategy [Left]
      []()
      {
        int newStrategy = (int)autonomous.getStrategy() - 1;
        if (newStrategy < 0)
          newStrategy = autonomous.getStrategyCount() - 1;
        autonomous.setStrategy((Strategy)newStrategy);
        displayStrategy();
      });
  primaryController.ButtonRight.pressed( // next strategy [Right]
      []()
      {
        int newStrategy = (int)autonomous.getStrategy() + 1;
        if (newStrategy >= autonomous.getStrategyCount())
          newStrategy = 0;
        autonomous.setStrategy((Strategy)newStrategy);
        displayStrategy();
      });
  primaryController.ButtonUp.pressed( // default strategy [Up]
      []()
      {
        autonomous.setStrategy(Autonomous::DEFAULT_STRATEGY);
        displayStrategy();
      });
  primaryController.ButtonDown.pressed( // no strategy [Down]
      []()
      {
        autonomous.setStrategy(Strategy::None);
        displayStrategy();
      });

  // Preset strategy buttons
  primaryController.ButtonX.pressed( // Loader Roller [X]
      []()
      {
        autonomous.setStrategy(Strategy::LoaderRoller);
        displayStrategy();
      });
  primaryController.ButtonY.pressed( // Side Roller[Y]
      []()
      {
        autonomous.setStrategy(Strategy::SideRoller);
        displayStrategy();
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

  secondaryController.ButtonUp.pressed( // increase flywheel speed (10% increments) [Up]
      []()
      {
        shooter.setTargetVelocity(100);
      });
  secondaryController.ButtonDown.pressed( // decrease flywheel speed (10% increments) [Down]
      []()
      {
        shooter.setTargetVelocity(70);
      });
  secondaryController.ButtonLeft.pressed( // set flywheel speed to the minimum value [Left]
      []()
      {
        shooter.setTargetVelocity(80); // will be overriden by the shooter class
      });
  secondaryController.ButtonRight.pressed( // set flywheel speed to the maximum value [Right]
      []()
      {
        shooter.setTargetVelocity(90);
      });
  secondaryController.ButtonR1.pressed( // fire disk (there is a list of preconditions specified in the class) [R1]
      []()
      {
        shooter.fireDisk();
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

    // roller spinner (top -> up, bot -> down) [L1/L2]
    if (primaryController.ButtonL1.pressing() || secondaryController.ButtonL1.pressing())
    {
      roller.spin(vex::forward);
    }
    else if (primaryController.ButtonL2.pressing() || secondaryController.ButtonL2.pressing())
    {
      roller.spin(vex::reverse);
    }
    else
    {
      roller.stop();
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
                         {
          // override controls during autonomous
          primaryController.ButtonLeft.pressed([](){});
          primaryController.ButtonRight.pressed([](){});
          primaryController.ButtonUp.pressed([](){});
          primaryController.ButtonDown.pressed([](){});
          primaryController.ButtonX.pressed([](){});
          primaryController.ButtonY.pressed([](){});
          autonomous.run(); });

  Competition.drivercontrol(userControl);

  int tuningIndex = 0; // The current value being tuned
  // 0 = P
  // 1 = I
  // 2 = D
  bool debounce = false;

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);

    // Temporary tuning mode
    if (primaryController.ButtonY.pressing() && primaryController.ButtonR2.pressing())
    {
      double increment = 0.01;
      if (primaryController.ButtonR1.pressing())
      {
        increment = 0.1;
      }

      if (!debounce)
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
    if (!(primaryController.ButtonUp.pressing() || primaryController.ButtonDown.pressing() || primaryController.ButtonLeft.pressing() || primaryController.ButtonRight.pressing()))
    {
      debounce = false;
    }
  }

  // call update functions
  shooter.updateVelocity();
}