#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"

using namespace pros;
using namespace c;
using namespace std;

bool stallProt1 = false;
bool stalled;

int stallTime = 0;
int direc;
int clampDistance;
int tunetime2 =0;
int hookPos;
int prevHookPos;

int viewTime;
/*string CD = string(int clampDistance());
(int clampDistance)string = string CD;*/
//constants used for caluclating power/voltage 
double vKp;
double vKi;
double vKd;
float error; //annount from target 
double prevError;
int intergral; 
int derivative;
int time2 = 0;
double power; //voltage provided to motors at any given time to reach the target

//constants used for caluclating power/voltage 2
double vKp2;
double vKi2;
double vKd2;
float error2; //annount from target 
double prevError2;
int intergral2; 
int derivative2;
int time22;
double power2; //voltage provided to motors at any given time to reach the target

//constants used for caluclating power/voltage 3
double vKi3;
double vKd3;
double error3; //annount from target 
double prevError3;
int intergral3; 
int derivative3;
int time23;
double power3; //voltage provided to motors at any given time to reach the target

void setConstants( double kp, double ki, double kd){
    vKp = kp;
    vKi = ki;
    vKd = kd;
    }

    void resetEncoders(){//resets the chassis motors every time a target is reached
        LF.tare_position();
        LM.tare_position();
        LB.tare_position();
        RF.tare_position();
        RM.tare_position();
        RB.tare_position();
    }

void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRB, int voltageRM){
        LF.move(voltageLF);
        LM.move(voltageLB);
        LB.move(voltageLM);
        RF.move(voltageRF);
        RM.move(voltageRM);
        RB.move(voltageRB);
}

double calPID(double target, double input, int integralKi, int maxIntergral){ //basically tuning i here
    int intergal;

    prevError = error;
    error = target - input;

    if(abs(error) < integralKi) {
        intergral += error; 
    } else {
        intergral = 0;
    }
if(intergral >= 0){
    intergral = min(intergral, maxIntergral); //min means take whichever value is smaller bwtween intergral and maxI
} else {
    intergral = max(intergral, -maxIntergral);//negative max
}
    derivative = error - prevError;

   power = (vKp * error) + (vKi * intergral) + (vKd * derivative);

    return power;
}

double calPID2(double target, double input, int integralKi, int maxIntergral){ //basically tuning i here
    int intergal2;

    prevError2 = error2;
    error2 = target - input;

    if(abs(error2) < integralKi) {
        intergral2 += error2; 
    } else {
        intergral2 = 0;
    }
if(intergral2 >= 0){
    intergral2 = min(intergral2, maxIntergral); //min means take whichever value is smaller bwtween intergral and maxI
} else {
    intergral2 = max(intergral2, -maxIntergral);//negative max
}
    derivative2 = error2 - prevError2;

   power2 = (vKp * error2) + (vKi * intergral2) + (vKd * derivative2);

    return power2;
}

double calPID3(double target, double input, int integralKi, int maxIntergral){ //basically tuning i here
    int intergal3;

    prevError3 = error3;
    error3 = target - input;

    if(abs(error) < integralKi) {
        intergral3 += error; 
    } else {
        intergral3 = 0;
    }
if(intergral3 >= 0){
    intergral3 = min(intergral3, maxIntergral); //min means take whichever value is smaller bwtween intergral and maxI
} else {
    intergral3 = max(intergral3, -maxIntergral);//negative max
}
    derivative3 = error3 - prevError3;

   power3 = (vKp * error3) + (vKi * intergral3) + (vKd * derivative3);

    return power3;
}

//driving strait 
void driveStraight (int target) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;


setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

imu.tare();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);

if(init_heading > 180){
    init_heading = (360 - init_heading);
}

if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 4;

if (voltage > 127){
    voltage = 127;
}else if (voltage <-127){
    voltage = -127;
}


chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
    tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f       ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"HeadingError!%f          ", float(init_heading));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f      ",float(time2));
}

    delay(10);
    time2 += 10;
}

LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}
//Drive Slow
void driveSlow (int target, int speed) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = 30;//( * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if (voltage > 127* double(speed)/100.0){
    voltage = 127* double(speed)/100.0;
}else if (voltage <-127* double(speed)/100.0){
    voltage = -127* double(speed)/100.0;
}


chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
    tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}

//Drive Strait 2 
void driveStraight2 (int target) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = 30;//(0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if (voltage > 127){
    voltage = 127;
}else if (voltage <-127){
    voltage = -127;
}


chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
    viewTime = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}

//Turning 
void driveTurn(int target){ //tagret is inputted in autons 
double voltage;
double position;
int count = 0;
time2=0;

setConstants(TURN_KP,TURN_KI, TURN_KD);
int timeout = 30000;
double variKP;
double variKD;
double x = 0;
x = double(abs(target));
variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; 
variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; 
//timeout = 30;//(0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

imu.tare_heading();

while(true){
    position = imu.get_heading(); //this is where the units are set to be degrees
    
    if (position > 180) {
        position = position - 360; //360
    }

    voltage = calPID(target, position, TURN_INTRGRAL_KI,TURN_MAX_INTEGRAL);

    chasMove(voltage,voltage,voltage,-voltage,-voltage,-voltage);

    if (abs(target - position) <= 1) count ++;
    if (count >= 20 || time2 > timeout){
         tunetime2 = time2;
    break;
    }

    if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"Imu!%f        ", float(imu.get_heading()));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    time2 += 10;
    delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


//Drive Turn 2
void driveTurn2(int target){ //tagret is inputted in autons 
double voltage;
double position;
int count = 0;
time2=0;
int turnv=0;

position = imu.get_heading(); //this is wher the nits are set to be degrees 

if (position > 180){
    position = ((350-position)*-1 );
}

if ((target < 0) && position >0) {
    if((position - target) >= 180){
        target = target +360;
        position = imu.get_heading();
        turnv = (target - position); //target + position
    } else {
        turnv = (abs(position) + abs(target));
    }
} else if ((target >0) && (position <0)){
    if ((target - position) >=180){
        position = imu.get_heading();
        turnv = abs(abs(position) -abs(target));
    } else {
        turnv = (abs(position) + target);
    }
} else {
    turnv = abs(abs(position) - abs(target));
}

setConstants(TURN_KP,TURN_KI, TURN_KD);
int timeout = 30000;
double variKP;
double variKD;
double x = 0;
x = double(abs(turnv));
variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; 
variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; 
//timeout = (30000 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 


while(true){
   position = imu.get_heading(); //this is wher the numbers are set to be degrees 

if (position > 180){
    position = ((350-position)*-1 );
}

if ((target < 0) && position >0) {
    if((position - target) >= 180){
        target = target +360;
        position = imu.get_heading();
        turnv = (target - position); //target + position
    } else {
        turnv = (abs(position) + abs(target));
    }
} else if ((target >0) && (position <0)){
    if ((target - position) >=180){
        position = imu.get_heading();
        turnv = abs(abs(position) -abs(target));
    } else {
        turnv = (abs(position) + target);
    }
} else {
    turnv = abs(abs(position) - abs(target));
}

    voltage = calPID(target, position, TURN_INTRGRAL_KI,TURN_MAX_INTEGRAL);

    chasMove(voltage,voltage,voltage,-voltage,-voltage,-voltage);

    if (abs(target = position) <= 1) count ++;
    if (count >= 20 || time2 > timeout){
          tunetime2 = time2;
    break;
    }

    if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(imu.get_heading()));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    time2 += 10;
    delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

//driveClamp, clamp distance is the distance u want it to clamp from the target

void driveClamp (int target, int clampDistance){
   int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);

if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if (voltage > 127){
    voltage = 127;
}else if (voltage <-127){
    voltage = -127;
}
 
if(int clampDistance = encoderAvg){
    MogoMech.set_value(true);
}
chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
    tunetime2 = time2;
    break;
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}
}

//driveIntake function has 2 perametiers, 1 target, 2 where from the target to start intaking where from the target to stop intaking

void driveOutake (int target, int start, int stop) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if (voltage > 127){
    voltage = 127;
}else if (voltage <-127){
    voltage = -127;
}

if(abs(error) < start && abs(error) > stop){
   Intake.move(127);
    Intake_Layer1.move(127);
}else{
    Intake.move(0);
    Intake_Layer1.move(0);
}

chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage + heading_error), (voltage + heading_error), (voltage + heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
       tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}
//drive intake
void driveIntake (int target, int start, int stop) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

/*if (voltage > 127){
    voltage = 127;
}else if (voltage <-127){
    voltage = -127;
}*/

if(abs(error) > start && abs(error) < stop){
   Intake.move(127);
    Intake_Layer1.move(127);
}else{
    Intake.move(0);
    Intake_Layer1.move(0);
}

chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage + heading_error), (voltage + heading_error), (voltage + heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
       tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}

//Drive Clamp Slow

void driveClampSlow (int target, int clampDistance, int speed) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if(abs(error) < clampDistance ){
    MogoMech.set_value(true);
}
if (voltage > 127* double(speed)/100.0){
    voltage = 127* double(speed)/100.0;
}else if (voltage <-127* double(speed)/100.0){
    voltage = -127* double(speed)/100.0;
}

chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage + heading_error), (voltage + heading_error), (voltage + heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
       tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}

LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}
// Drive Intake Slow 

void driveIntakeSlow (int target, int start, int stop, int speed) {
    int timeout = 30000;

double x = 0;
x = double(abs(target));
//timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0* pow(x,3)) + (0* pow(x,2)) + (0 * x) + 0; //Comment timeout our while tuning pid and while tuning timeout, Tune wit 

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    int time2 = 0;

setConstants( STRAIT_KP, STRAIT_KI,STRAIT_KD);
resetEncoders();

while(true){

encoderAvg = (LB.get_position() + RB. get_position()) / 2;
voltage = calPID(target, encoderAvg, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);


if(imu.get_heading() < 180) {
    heading_error = init_heading - imu.get_heading();
}
else {
    heading_error =((360 - imu.get_heading())-init_heading);
}
heading_error = heading_error * 0;

if (voltage > 127* double(speed)/100.0){
    voltage = 127* double(speed)/100.0;
}else if (voltage <-127* double(speed)/100.0){
    voltage = -127* double(speed)/100.0;
}

if(abs(error) < start && abs(error) > stop){
   Intake.move(127);
    Intake_Layer1.move(127);
}else{
    Intake.move(0);
    Intake_Layer1.move(0);
}

chasMove( (voltage + heading_error), (voltage + heading_error), (voltage + heading_error),(voltage + heading_error), (voltage + heading_error), (voltage + heading_error));
if (abs(target- encoderAvg) <=4)-count++;
if (count >= 20 || time2 > timeout){
    tunetime2 = time2;
    break;
}
if (time2 % 50 == 0 && time2 % 100 !=0 && time2 % 150 !=0){
    con.print(0,0,"ERROR:%f    ", float(error));
} else if (time2% 100 == 0 && time2 % 150 !=0){
    con.print(1,0,"EncoderAvg!%f        ", float(encoderAvg));
} else if (time2 % 150 ==0){
    con.print(2,0,"Time:%f    ",float(time2));
}

    delay(10);
    time2 += 10;
}
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();
}

//Intake
void justIntake (int time){
if(abs(error) < time){
    Intake.move(-127);
    Intake_Layer1.move(-100);
}else{
    Intake.move(0);
    Intake_Layer1.move(0);
}
}

void hooks(int speed){
direc == speed;
}

void StallProt(){
if(stallProt1){
    hookPos == prevHookPos;
    hookPos = Intake.get_position();

    if(stalled){
        Intake.move(-direc);
        stallTime += 10;
        if(stallTime >= 100){
            stalled = false;
        }
    } else {
        Intake.move(direc);
        stallTime = 0;
    }
}
}

void driveArcL(double theta, double radius, int timeout){
// bool over = false; 
// setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
int totalError = 0;
double ltarget = 0; 
double rtarget = 0;
double pi = 3.14159265359;
double init_heading = imu.get_heading();

if(init_heading > 180){
    init_heading = init_heading - 360;
}
int time = 0;
int count = 0;
resetEncoders();



ltarget = double((theta/360) *2 * pi * radius);
rtarget = double((theta / 360) * 2 * pi *(radius + 565));
//570
while(true){

setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);
double encoderAvgL = LF.get_position(); 
double encoderAvgR = RB.get_position();
 int voltageL = calPID(ltarget, encoderAvgL, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageL > 127){
        voltageL = 127;
    }else if(voltageL < -127){
        voltageL = -127;
    }

    int voltageR = calPID2(rtarget, encoderAvgR, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageR > 127){
        voltageR = 127;
    }else if(voltageR < -127){
        voltageR = -127;
    }

double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);
double position = imu.get_heading();

if(position > 180){
    position = position - 360;
}

if((leftcorrect < 0) && (position > 0)){
        if((position - leftcorrect ) >= 180){
            leftcorrect = leftcorrect + 360;
            position = imu.get_heading();
        }
    } else if((leftcorrect > 0) && (position < 0)){
        if ((leftcorrect - position) && (position < 0)){
            position = imu.get_heading();
        }
     }
    setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
    int fix = calPID3((init_heading + leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
    totalError += (abs(error3));   
    if (abs(ltarget - encoderAvgL) <= 25) fix = 0;   
        chasMove( (voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 10) && (abs(rtarget - encoderAvgR) <= 10)) count++;
        if (count >= 20 || time > timeout){
            break;
        }

     if(time % 50 == 0 && time % 100 != 0 && time % 150!= 0){
            con.print(0,0, "ERROR: %f           ", float(time));
        }
         if(time % 50 == 0 && time % 100 != 0){
            con.print(2,0, "Voltage: %f           ",float(voltageL));
        }
         if(time % 50 == 0){
            con.print(1,0, "leftcorrect: %f           ", float(leftcorrect));
        }
        

        time+= 10;
        delay(10);


    }
}
void driveArcR(double theta, double radius, int timeout){
    setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);


double ltarget = 0;
double rtarget = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
if(init_heading > 180){
    init_heading = init_heading - 360;
}
int count = 0;
time2 = 0;
resetEncoders();

rtarget = double((theta/360) *2 * pi * radius);
ltarget = double((theta / 360) * 2 * pi *(radius + 570)); 

while (true){

if(init_heading > 180){
    init_heading = init_heading - 360;
}
double position = imu.get_heading();

if(position > 180){
    position = position - 360;
}
double encoderAVGL = (LF.get_position() + LB.get_position()) /2;
double encoderAVGR = (RB.get_position() + RB.get_position()) /2;
double rightcorrect = (encoderAVGR * 360) / (2 * pi * radius);

if((rightcorrect < 0) && (position > 0)){
        if((position - rightcorrect) >= 180){
            rightcorrect = rightcorrect + 360;
            position = imu.get_heading();
        }
    } else if((rightcorrect > 0) && (position < 0)){
        if ((rightcorrect - position) >= 180){
            position = imu.get_heading();
        }
     }
    setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);
       
         int voltageL = calPID(ltarget, encoderAVGL, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL); 
    if (voltageL > 127){
        voltageL = 127;
    }else if(voltageL < -127){
        voltageL = -127;
    }

    int voltageR = calPID2(rtarget, encoderAVGR, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageR > 127){
        voltageR = 127;
    }else if(voltageR < -127){
        voltageR = -127;
    }
    
    setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calPID3((init_heading + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
    if (abs(rtarget - encoderAVGR) <= 25) fix = 0;  

    chasMove( (voltageL + fix), (voltageL + fix ), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
    if ((abs(ltarget - encoderAVGL) <= 4) && (abs(rtarget - encoderAVGR)<= 4)) count++;
    if (count >= 20  || time2 > timeout){
       break;
    }
     if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150!= 0){
            con.print(0,0, "fix: %f           ", float(fix));
        } else if(time2 % 50 == 0 && time2 % 100 != 0){
            con.print(1,0, "position: %f           ", float(position));
        }else if(time2 % 50 == 0){
            con.print(2,0, "init_heading: %f         ", float(init_heading + rightcorrect));
        }
        time2 += 10;
        delay(10);
    }   
}

void driveArcLF(double theta, double radius, int timeout){

setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);

double ltarget = 0; 
double rtarget = 0;
double ltargetF = 0; 
double rtargetF = 0;
double pi = 3.14159265359;
double init_heading = imu.get_heading();
int time = 0;
int count = 0;
bool over = false;
resetEncoders();
ltargetF = double(( theta/360) *2 * pi * radius);
rtargetF = double((theta / 360) * 2 * pi *(radius + 750 ));
theta = theta + 45;
ltarget = double(( theta/360) *2 * pi * radius);
rtarget = double((theta / 360) * 2 * pi *(radius + 750 ));
while(true){

if(init_heading > 180){
    init_heading = init_heading - 360;
}
double position = imu.get_heading();


if(position > 180){
    position = position - 360;
}
double encoderAvgL = LF.get_position(); 
double encoderAvgR = RB.get_position();
double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);

if((leftcorrect < 0) && (position > 0)){
        if((position - leftcorrect ) >= 180){
            leftcorrect = leftcorrect + 360;
            position = imu.get_heading();
        }
    } else if((leftcorrect > 0) && (position < 0)){
        if ((leftcorrect - position) && (position < 0)){
            position = imu.get_heading();
        }
     }
    setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);
 
    int voltageL = calPID(ltarget, encoderAvgL, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageL > 127){
        voltageL = 127;
    }else if(voltageL < -127){
        voltageL = -127;
    }

    int voltageR = calPID2(rtarget, encoderAvgR, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageR > 127){
        voltageR = 127;
    }else if(voltageR < -127){
        voltageR = -127;
    }

 setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calPID3((init_heading + leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
if (abs(ltarget - encoderAvgL) <= 25) fix = 0;  
        chasMove( (voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if (theta > 0){
            if ((encoderAvgL - ltargetF) > 0){
                over = true;
            }
        } else {
            if ((ltargetF - encoderAvgL)>0){
                over = true;
            }
        }

        if (over || time > timeout){
            break;
        }

     if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150!= 0){
            con.print(0,0, "ERROR: %f           ", float(time2));
        }
         if(time2 % 50 == 0 && time2 % 100 != 0){
            con.print(2,0, "EncoderAVG: %f           ", float(LF.get_encoder_units()));
        }
         if(time2 % 50 == 0){
            con.print(1,0, "Time2: %f           ", float(time2));
        }
        

        time+= 10;
        delay(10);


    }
}

void driveArcRF(double theta, double radius, int timeout){
    setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);
bool over = false;
double ltargetF = 0;
double rtargetF = 0;
double ltarget = 0;
double rtarget = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
if(init_heading > 180){
    init_heading = init_heading - 360;
}
int count = 0;
int time2 = 0;
resetEncoders();
rtargetF = double((theta/360) *2 * pi * radius);
ltargetF= double((theta / 360) * 2 * pi *(radius + 750)); 
theta = theta + 45;
rtarget = double((theta/360) *2 * pi * radius);
ltarget = double((theta / 360) * 2 * pi *(radius + 750)); 

while (true){

if(init_heading > 180){
    init_heading = init_heading - 360;
}
double position = imu.get_heading();

if(position > 180){
    position = position - 360;
}

if((init_heading < 0) && (position > 0)){
        if((position - init_heading ) >= 180){
            init_heading = init_heading + 360;
            position = imu.get_heading();
        }
    } else if((init_heading > 0) && (position < 0)){
        if ((init_heading - position) && (position < 0)){
            position = imu.get_heading();
        }
     }
    setConstants(STRAIT_KP, STRAIT_KI, STRAIT_KD);
         double encoderAVGL = (LF.get_position() + LB.get_position()) /2;
         double encoderAVGR = (RB.get_position() + RB.get_position()) /2;
         int voltageL = calPID(ltarget, encoderAVGL, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL); 
    if (voltageL > 127){
        voltageL = 127;
    } else if(voltageL < -127){
        voltageL = -127;
    }

    int voltageR = calPID2(rtarget, encoderAVGR, STRAIT_INTEGRAL_KI, STRAIT_MAX_INTEGRAL);
    if (voltageR > 127){
        voltageR = 127;
    }else if(voltageR < -127){
        voltageR = -127;
    }
    double rightcorrect = (encoderAVGR * 360) / (2 * pi * radius);
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calPID3((init_heading + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
 if (abs(rtarget - encoderAVGR) <= 25) fix = 0;  

    chasMove( (voltageL - fix), (voltageL - fix ), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
   if (theta > 0){
    if ((encoderAVGR - (rtargetF)) > 0){
        over = true;
    }
   } else {
    if(((rtarget) - encoderAVGR) > 0){
        over = true;
    }
   }
    if (over || time2 > timeout){
        break;
    }
     if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150!= 0){
            con.print(0,0, "fix: %f           ", float(fix));
        } else if(time2 % 50 == 0 && time2 % 100 != 0){
            con.print(1,0, "encodeR %f           ", float(encoderAVGR));
        }else if(time2 % 50 == 0){
            con.print(2,0, "encodeL: %f         ", float(encoderAVGL));
        }
        time2 += 10;
        delay(10);
    }   
} 

void ColorSenseIntake(int speed, bool color_sort = false){
   // defualts to sort out blue 
    // color_sort = true;
    //    string blueSort;
    //  string redSort;
   if(speed > 127){
    speed = 127;
   }
    while(color_sort = true){ // use for red sort 
     Intake.move(speed);
     Intake_Layer1.move(speed);
     if (eyes.get_hue() < 20){
        delay(500);
        Intake.move(0);
        Intake_Layer1.move(0);
     }
   }
    while(color_sort = false){ // use for blue sort 
     Intake.move(speed);
     Intake_Layer1.move(speed);
     if (eyes.get_hue() < 20){
        delay(500);
        Intake.move(0);
        Intake_Layer1.move(0);
     }
   } 

}

