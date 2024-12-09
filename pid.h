#include "api.h"
#include "main.h"
#include "robot.h"
//header guards
#ifndef PIDH
#define PIDH

#define STRAIT_KP 2.00// make it bigger untill it goes back and fourth and make sure that the error is less than 2-3 
#define STRAIT_KI 0.001// 0.001(breaks out of the loop)
#define STRAIT_KD 7// start at what kp is at then make it bigger

#define STRAIT_INTEGRAL_KI 40//
#define STRAIT_MAX_INTEGRAL 14.5//

extern void driveStraight(int target);
extern void driveStraight2(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveSlow(int target, int speed);
extern double calPID(double target, double input, int integralKi, int maxIntergral);
extern void driveIntake(int target, int start, int stop);
extern void driveClamp (int target, int clampDistance);
extern void setConstants( double kp, double ki, double kd);
extern void driveClampSlow (int target, int clampDistance, int speed);
extern void driveIntakeSlow (int target, int start, int stop, int speed);
extern void driveArcR(double theta, double radius, int timeout);
extern void driveArcLF(double theta, double radius, int timeout);
extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcRF(double theta, double radius, int timeout);




extern int time2;
extern float error;
extern int tunetime2;
extern void justIntake (int time);
extern void hooks(int speed);
extern void stallProt();
extern int viewTime;

#define TURN_KP 7.00//
#define TURN_KI 0// 
#define TURN_KD 70// 

#define TURN_INTRGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define LIFT_KP 1.5// 
#define LIFT_KI 0// 
#define LIFT_KD 0// 

//arc stuff
#define HEADING_KP 6
#define HEADING_KI 0
#define HEADING_KD 0
#define HEADING_MAX_INTEGRAL 0
#define HEADING_INTEGRAL_KI 0

//arc turn stuff
#define ARC_HEADING_KP 40 //make it bigger untill u can see it correcting along the path
#define ARC_HEADING_KI 0.00
#define ARC_HEADING_KD 18 // makt it bigger untill it is smooth
#define ARC_HEADING_MAX_INTEGRAL 0
#define ARC_HEADING_INTEGRAL_KI 0 // to- do list, tune 




#endif