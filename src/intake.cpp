/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       intake.cpp                                                */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-02-24, 11:51:29 a.m.                                 */
/*    Description:  Manages the intake                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "intake.h"

Intake::Intake(int32_t intakePort)
{
    intake = new motor(intakePort, ratio6_1, true);
    intake->setVelocity(100, velocityUnits::pct);
}

void Intake::spinIntake(directionType direction)
{
    intake->spin(direction);
}

void Intake::spinIntake(bool R1, bool R2)
{
    if (R1 == R2) // If both are pressed or neither are pressed, stop
        intake->stop();
    // If only one is pressed, spin in that direction
    else if (R1)
        intake->spin(directionType::fwd); // Spin in
    else if (R2)
        intake->spin(directionType::rev); // Spin out
}

void Intake::stopIntake()
{
    intake->stop();
}