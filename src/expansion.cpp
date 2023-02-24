/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       expansion.cpp                                             */
/*    Author:       Trubotics                                                 */
/*    Created:      2022-12-09, 10:36:28 a.m.                                 */
/*    Description:  Manages the endgame expansion                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "expansion.h"

Expansion::Expansion(pneumatics &expansion, controller &primaryController, controller &secondaryController)
{
    this->expansion = &expansion;
    this->primaryController = &primaryController;
    this->secondaryController = &secondaryController;
}

void Expansion::expand()
{
    expansion->set(true);
}

void Expansion::tryExpand()
{
    primaryController->rumble("-");
    secondaryController->rumble("-");

    if (!(primaryController->ButtonR1.pressing() && secondaryController->ButtonR1.pressing()))
        return;

    for (int i = 0; i < 1000; i += 100)
    {
        wait(100, msec);
        primaryController->rumble(".");
        secondaryController->rumble(".");

        if (!(primaryController->ButtonR1.pressing() && secondaryController->ButtonR1.pressing()))
            return;
    }

    expand();
}