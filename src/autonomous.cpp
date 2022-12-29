/* See autonomous branch */
 
#include <cmath>
#include <vex.h>
#include <mecanumDrivetrain.h>

const int  drivePCT_AUTO = 50;
const float wheelCirc = 4.1251 * 3.1416;
const float gearRatio = 0.50;
const int turnDiam = 19;

namespace colours 
{
const int red = 1;
const int blue = 2;
const int miss = 3;
} 

// Sensors 
pros::Distance indexerSensor(2);
pros::Optical rollerSensor(3);
pros::IMU imu_sensor(1);


// define OPTICAL_PORT 1
void initialize() 
{
  pros::Optical optical_sensor(OPTICAL_PORT);
}
