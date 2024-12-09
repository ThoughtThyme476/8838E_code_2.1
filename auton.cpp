#include "main.h"
#include "api.h"
#include "pid.h"
#include "robot.h"
using namespace pros;
void autons1 () {
driveStraight(1600);
driveTurn(15);
delay(50);
driveSlow(300, 100);
StakeWing.set_value(true);
//goal grab
delay(50);
driveSlow(-1000, 50);
StakeWing.set_value(false);
driveTurn(120);
delay(50);
driveTurn(72.5);
// driveClampSlow(-00, 10, 60);
driveStraight(-950);
delay(50);
//goal clamp
justIntake(1);
// first ring 
driveTurn(-55);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////