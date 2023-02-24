#ifndef EXPANSION_H
#define EXPANSION_H

#include <vex.h>
using namespace vex;

class Expansion
{
private:
    pneumatics *expansion = nullptr;

    controller *primaryController = nullptr;
    controller *secondaryController = nullptr;
public:
    Expansion(pneumatics &expansion, controller &primaryController, controller &secondaryController);

    void tryExpand();
    void expand();
};

#endif