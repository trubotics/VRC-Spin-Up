#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <string>
#include <map>
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
    // Strategy map to string
    std::map<Strategy, std::string> strategyMap = {
        {Strategy::None, "None"},
        {Strategy::LoaderRoller, "Loader Roller"},
        {Strategy::SideRoller, "Side Roller"},
    };

    Strategy strategy = DEFAULT_STRATEGY;

    MecanumDriveTrain *drive = nullptr;
    Shooter *shooter = nullptr;
    motor_group *flywheel = nullptr;
    motor *roller = nullptr;

    void rollRoller();
    void fireDisk(int count = 2, double velocity = 100);
public:
    static const Strategy DEFAULT_STRATEGY = Strategy::LoaderRoller;

    Autonomous(MecanumDriveTrain &drive, Shooter &shooter, motor_group &flywheel, motor &roller);

    void run();

    std::string getStrategyString();
    Strategy getStrategy();
    int getStrategyCount();
    void setStrategy(Strategy strategy);
};

#endif