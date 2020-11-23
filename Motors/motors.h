#ifndef MOTORS_H
#define MOTORS_H
//Header File
#include <stdbool.h> //Bool

//Motor Pins
#define F_MOT_A 3    //Forward
#define F_MOT_B 100  //Forward
#define R_MOT_A 2    //Reverse
#define R_MOT_B 100  //Reverse
#define VOLT_MOT_A 0 //PowerSupply

//Global Variable
bool directionLeft, directionRight;
bool triggerForward, triggerReverse = false;
bool obstacle, trail = false;
bool endProgram = false;

//Sensor Methods
void *offTrail() {}
void *onTrail() {}
void *detectObstacle(){};
void *detectLine(){};
//Navigation Methods
void *reNavigate(){};
void *turnNSWE(){};

//Motor
void *runMotorA(){}; //Top_Left: 1
void *runMotorB(){}; //Top_Right:2
void *runMotorC(){}; //Bottom_Left:3
void *runMotorD(){}; //Bottom_Left:4

void *MotorReverse() {}
void *MotorBakwards() {}
void *MotorForwards() {}
void *MotorTurn(float degree) {}

#endif
