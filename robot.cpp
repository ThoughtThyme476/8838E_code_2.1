#include "main.h"
#include "api.h"
#include "robot.h"

//header Guards, safety

#define LF_PORT 13
#define LM_PORT 11
#define LB_PORT 9
#define RF_PORT 5
#define RM_PORT 7
#define RB_PORT 1
#define IMU_PORT 15
#define Intake_PORT 4
#define Intake_Layer1_PORT 12
#define Redirect_PORT 3
#define OPTICAL_PORT 2
#define DISTANCE_PORT 6
//hello

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor Intake (Intake_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor Intake_Layer1 (Intake_Layer1_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor Redirect (Redirect_PORT, pros::E_MOTOR_GEARSET_18, true);

pros::Imu imu(IMU_PORT);
pros::Vision eyes (OPTICAL_PORT);
pros::Distance dis (DISTANCE_PORT);
pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut MogoMech ('A',false);
pros::ADIDigitalOut StakeWing('B',false);
pros::ADIDigitalIn selec ('C');