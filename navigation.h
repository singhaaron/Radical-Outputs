#ifndef NAVIGATION_H
#define NAVIGATION_H
//Header File
#include <stdbool.h> //Bool

//Macros
#define IR_OBSTACLE_SENSOR 22 // Physical 32-GPIO12
#define LINE_SENSOR 26        // Physical 31-GPIO6

//Motor Macros
#define DIRECTION_MOTOR_A
#define DIRECTION_MOTOR_B
#define DIRECTION_MOTOR_C
#define DIRECTION_MOTOR_D

#define VOLTAGE_MOTOR_A 22 // 22 Physical 31 - GPIO6
#define VOLTAGE_MOTOR_B 22 // 22 Physical 31 - GPIO6
#define VOLTAGE_MOTOR_C 22 // 22 Physical 31 - GPIO6
#define VOLTAGE_MOTOR_D 22 // 22 Physical 31 - GPIO6

//Global Variable
bool directionLeft, directionRight;

//Helper Functions
void *detectObstacle(){};
void *detectLine(){};
void *reNavigate(){};
void *turnNSWE(){};

//Motor Methods
void *MotorA(){}; //Top_Left: 1
void *MotorB(){}; //Top_Right:2
void *MotorC(){}; //Bottom_Left:3
void *MotorD(){}; //Bottom_Left:4

void *MotorReverse() {}
void *MotorBakwards() {}
void *MotorForwards() {}
void *MotorTurn(float degree) {}

//Sensor Methods
void *offTrail() {}
void *onTrail() {}

#endif
