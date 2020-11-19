/**************************************************************
* Class:  CSC-615 Fall 2020
* Name:   RadicalOutputz
* Student ID:**********
* Project: Vehicle
*
* File: navigation.c
*
* Description:
* Vehicle Functions
***************************************************************/
#include "navigation.h"

#define START_FORWARDMOTOR                         \
    for (int index = 25; index < 125; index += 25) \
    {                                              \
        digitalWrite(DIRECTION_MOTOR_A, 1);        \
        softPwmWrite(VOLTAGE_MOTOR_A, index);      \
        delay(500);                                \
    }
#define START_REVERSEMOTOR                         \
    for (int index = 25; index < 125; index += 25) \
    {                                              \
        digitalWrite(DIRECTION_MOTOR_C, 1);        \
        softPwmWrite(VOLTAGE_MOTOR_C, index);      \
        delay(500);                                \
    }
//Motor A & B - Forward Motion
void *MotorA()
{
    START_FORWARDMOTOR; //Gradually Churn
    while (triggerForward)
    {
        (true /*Check states if reversal & Stop is needed*/) ? printf("ContinueExec") : printf("Break & Call Reversal Motor ");
    }
}
//Motor C & D - Reverse Motion
void *MotorC()
{
    START_REVERSEMOTOR;
    while (triggerReverse)
    {
        (true /*Check states if forward or stop is needed*/) ? printf("ContinueExec") : printf("Break & Call Forward Motor ");
    }
}