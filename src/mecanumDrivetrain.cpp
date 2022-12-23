/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       mecanumDrivetrain.cpp                                     */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-13, 10:55:45 p.m.                                 */
/*    Description:  A cool thing made to simplify the mecanum drive train     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

class MecanumDriveTrain
{
private:
    motor leftFront = NULL;
    motor leftBack = NULL;
    motor rightFront = NULL;
    motor rightBack = NULL;
    void convertMotorValues(int forward, int strafe, int turn, int motorValues[]);

public:
    MecanumDriveTrain(int32_t leftFrontPort, int32_t leftBackPort, int32_t rightFrontPort, int32_t rightBackPort);

    void drive(int forward, int strafe, int turn);
};

MecanumDriveTrain::MecanumDriveTrain(int32_t leftFrontPort, int32_t leftBackPort, int32_t rightFrontPort, int32_t rightBackPort)
{
    leftFront = motor(leftFrontPort, ratio18_1, false);
    leftBack = motor(leftBackPort, ratio18_1, false);
    rightFront = motor(rightFrontPort, ratio18_1, false);
    rightBack = motor(rightBackPort, ratio18_1, false);

    // set brake modes
    leftFront.setStopping(vex::brakeType::hold);
    leftBack.setStopping(vex::brakeType::hold);
    rightFront.setStopping(vex::brakeType::hold);
    rightBack.setStopping(vex::brakeType::hold);
}

void MecanumDriveTrain::convertMotorValues(int forward, int strafe, int turn, int motorValues[])
{
    motorValues[0] = forward + strafe - turn; // front left
    motorValues[1] = forward - strafe - turn; // back left
    motorValues[2] = forward - strafe + turn; // front right
    motorValues[3] = forward + strafe + turn; // back right
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