#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <string>
#include <vex.h>
#include <mecanumDrivetrain.h>
#include <shooter.h>
#include <rollerRoller.h>
using namespace vex;

enum class Strategy
{
    None,         // Waste 15 seconds and get carried
    LoaderRoller, /* Rolls the roller left of the loader
                      - Place in front of roller at start
                      - Robot "creeps" to roller
                      - Robot rolls roller
                      - Robot backs up, turns to net and blindfires two preloads
                  */
    SideRoller,   /* Rolls the other roller at the side of the field
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
    Strategy strategy = DEFAULT_STRATEGY;

    MecanumDriveTrain *drive = nullptr;
    Shooter *shooter = nullptr;
    RollerRoller *roller = nullptr;

    distance *leftDistance = nullptr;
    distance *rightDistance = nullptr;
    inertial *inertialSensor = nullptr;

    void sensorStrafe(double targetDistance, double velocity = 50);
    void sensorRotate(double deltaAngle, double velocity = 50);
    void rollRoller();
    void fireDisk(int count = 2, double velocity = 70);

public:
    static const Strategy DEFAULT_STRATEGY = Strategy::LoaderRoller;

    Autonomous(MecanumDriveTrain &drive, Shooter &shooter, RollerRoller &roller,
               distance &leftDistance, distance &rightDistance, inertial &inertialSensor);

    void run();

    std::string getStrategyString();
    Strategy getStrategy();
    void setStrategy(Strategy strategy);
};

#endif