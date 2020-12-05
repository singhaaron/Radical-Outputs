#ifndef MOTORS_H
#define MOTORS_H
#include <stdbool.h>
//Motor Macros
//MOTOR-1
#define F_MOT_A 3
#define R_MOT_A 2
#define VOLT_MOT_A 0
//MOTOR-2
#define F_MOT_B 4
#define R_MOT_B 5
#define VOLT_MOT_B 6
//MOTOR-3
#define F_MOT_C 13
#define R_MOT_C 14
#define VOLT_MOT_C 12
//MOTOR-4
#define F_MOT_D 10
#define R_MOT_D 11
#define VOLT_MOT_D 26

//***************WiringPi***************//
static void setup();
static void allOff();
static void interruptHandlers(const int signals);
//***************WiringPi***************//

extern bool A_triggerForward, A_triggerReverse, B_triggerForward, B_triggerReverse, C_triggerForward, C_triggerReverse, D_triggerForward, D_triggerReverse, obstacle, onTrail, endRun;

void *runMotorA();
void *runMotorB();
void *runMotorC();
void *runMotorD();

#endif
