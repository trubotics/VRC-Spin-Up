#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <vex.h>
#include <mecanumDrivetrain.h>
#include <shooter.h>
using namespace vex;

enum class Strategy
{
    None, // Waste 15 seconds and get carried
    LoaderRoller,   /* Rolls the roller left of the loader
                        - Place in front of roller at start
                        - Robot "creeps" to roller
                        - Robot rolls roller
                        - Robot backs up, turns to net and blindfires two preloads
                    */
    SideRoller,     /* Rolls the other roller at the side of the field
                        - Place on tile closest to roller at start
                        - Robot strafes right to roller (1 tile)
                        - Robot "creeps" to roller
                        - Robot rolls roller
                        - Robot backs up, turns to net and blindfires two preloads
                    */
};

class Autonomous
{
private:
    const bool IS_BLUE = true;
    const Strategy STRATEGY = Strategy::LoaderRoller;

    MecanumDriveTrain *drive = nullptr;
    Shooter *shooter = nullptr;
    motor *roller = nullptr;

    void rollRoller();
    void fireDisk(double velocity = 100);
public:
    Autonomous(MecanumDriveTrain &drive, Shooter &shooter, motor &roller);

    void run();

    Strategy getStrategy();
};

#endif