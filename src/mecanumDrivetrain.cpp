/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       mecanumDrivetrain.cpp                                     */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  A cool thing made to simplify the mecanum drive train     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath>
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

    // set brake modes
    leftFront.setStopping(vex::brakeType::brake);
    leftBack.setStopping(vex::brakeType::brake);
    rightFront.setStopping(vex::brakeType::brake);
    rightBack.setStopping(vex::brakeType::brake);
}

void MecanumDriveTrain::convertMotorValues(int forward, int strafe, int turn, int motorValues[])
{
    motorValues[0] = forward + strafe + turn; // front left
    motorValues[1] = forward - strafe + turn; // back left
    motorValues[2] = forward - strafe - turn; // front right
    motorValues[3] = forward + strafe - turn; // back right
    return;
}

void MecanumDriveTrain::drive(int forward, int strafe, int turn)
{
    int motorValues[4];
    convertMotorValues(forward, strafe, turn, motorValues);
    leftFront.spin(vex::forward, motorValues[0], vex::percent);
    leftBack.spin(vex::forward, motorValues[1], vex::percent);
    rightFront.spin(vex::forward, motorValues[2], vex::percent);
    rightBack.spin(vex::forward, motorValues[3], vex::percent);
    return;
}

// TODO: add more drive functions as needed (for autonomous period, etc.)
