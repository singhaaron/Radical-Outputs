#ifndef NAVIGATION_H
#define NAVIGATION_H
//Header File
#include <stdbool.h> //Bool

//Macros
#define IR_OBSTACLE_SENSOR 22 // Physical 32-GPIO12
#define LINE_SENSOR 26        // Physical 31-GPIO6

//Motor Macros
#define DIRECTION_MOTOR_A 26 //(physical 32)
#define DIRECTION_MOTOR_B 21 //(physical 29)
#define DIRECTION_MOTOR_C 12
#define DIRECTION_MOTOR_D 12

#define VOLTAGE_MOTOR_A 22 //(physical 31)
#define VOLTAGE_MOTOR_B 30 //(physical 27)
#define VOLTAGE_MOTOR_C 22 // 22 Physical 31 - GPIO6
#define VOLTAGE_MOTOR_D 22 // 22 Physical 31 - GPIO6

//Global Variable
bool directionLeft, directionRight;
bool triggerForward, triggerReverse = false;

//Sensor Methods
void *offTrail() {}
void *onTrail() {}
void *detectObstacle(){};
void *detectLine(){};
//Navigation Methods
void *reNavigate(){};
void *turnNSWE(){};

//Motor
void *MotorA(){}; //Top_Left: 1
void *MotorB(){}; //Top_Right:2
void *MotorC(){}; //Bottom_Left:3
void *MotorD(){}; //Bottom_Left:4

void *MotorReverse() {}
void *MotorBakwards() {}
void *MotorForwards() {}
void *MotorTurn(float degree) {}

#endif