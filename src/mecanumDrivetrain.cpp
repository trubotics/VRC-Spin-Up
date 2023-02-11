/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       mecanumDrivetrain.cpp                                     */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  A cool thing made to simplify the mecanum drive train     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <mecanumDrivetrain.h>

using namespace vex;

MecanumDriveTrain::MecanumDriveTrain(int32_t leftFrontPort, bool leftFrontReversed,
                                     int32_t leftBackPort, bool leftBackReversed,
                                     int32_t rightFrontPort, bool rightFrontReversed,
                                     int32_t rightBackPort, bool rightBackReversed)
{
    leftFront = motor(leftFrontPort, ratio18_1, leftFrontReversed);
    leftBack = motor(leftBackPort, ratio18_1, leftBackReversed);
    rightFront = motor(rightFrontPort, ratio18_1, rightFrontReversed);
    rightBack = motor(rightBackPort, ratio18_1, rightBackReversed);

    // default to max velocity
    leftFront.setVelocity(100, vex::velocityUnits::pct);
    leftBack.setVelocity(100, vex::velocityUnits::pct);
    rightFront.setVelocity(100, vex::velocityUnits::pct);
    rightBack.setVelocity(100, vex::velocityUnits::pct);

    // set initial brake modes
    setMotorLock(true);
}

// Convert the joystick values to motor speeds
void MecanumDriveTrain::convertMotorValues(int forward, int strafe, int turn, int motorValues[])
{
    motorValues[0] = forward + strafe + turn; // front left
    motorValues[1] = forward - strafe + turn; // back left
    motorValues[2] = forward - strafe - turn; // front right
    motorValues[3] = forward + strafe - turn; // back right
}

// Drive the robot using the joystick values
void MecanumDriveTrain::drive(int forward, int strafe, int turn)
{
    int motorValues[4];
    convertMotorValues(forward, strafe, turn, motorValues);
    leftFront.spin(vex::forward, motorValues[0], vex::percent);
    leftBack.spin(vex::forward, motorValues[1], vex::percent);
    rightFront.spin(vex::forward, motorValues[2], vex::percent);
    rightBack.spin(vex::forward, motorValues[3], vex::percent);
}

// Drive for a certain number of rotations (primarily used for autonomous)
void MecanumDriveTrain::driveFor(int forward, int strafe, int turn, double rotations, double velocity)
{
    rotations /= GEAR_MULTIPLIER;

    int motorValues[4];
    convertMotorValues(forward, strafe, turn, motorValues);
    leftFront.spinFor(rotations * motorValues[0] / 100, vex::rotationUnits::rev, velocity, vex::velocityUnits::pct, false);
    leftBack.spinFor(rotations * motorValues[1] / 100, vex::rotationUnits::rev, velocity, vex::velocityUnits::pct, false);
    rightFront.spinFor(rotations * motorValues[2] / 100, vex::rotationUnits::rev, velocity, vex::velocityUnits::pct, false);
    rightBack.spinFor(rotations * motorValues[3] / 100, vex::rotationUnits::rev, velocity, vex::velocityUnits::pct, false);

    // Wait for all motors to stop
    while (leftFront.isSpinning() || leftBack.isSpinning() || rightFront.isSpinning() || rightBack.isSpinning())
    {
        vex::task::sleep(10);
    }
}

// Functions to get/set whether the motors hold/lock position or use natural breaking
bool MecanumDriveTrain::getMotorLock()
{
    return motorLock;
}
void MecanumDriveTrain::setMotorLock(bool lock)
{
    if (lock)
    {
        leftFront.setStopping(vex::brakeType::hold);
        leftBack.setStopping(vex::brakeType::hold);
        rightFront.setStopping(vex::brakeType::hold);
        rightBack.setStopping(vex::brakeType::hold);
    }
    else
    {
        leftFront.setStopping(vex::brakeType::brake);
        leftBack.setStopping(vex::brakeType::brake);
        rightFront.setStopping(vex::brakeType::brake);
        rightBack.setStopping(vex::brakeType::brake);
    }

    motorLock = lock;
}

// Get the average torque of all motors
double MecanumDriveTrain::getAvgTorque()
{
    return (leftFront.torque() + leftBack.torque() + rightFront.torque() + rightBack.torque()) / 4;
}