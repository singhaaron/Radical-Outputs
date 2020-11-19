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
//DMX:Direction Motor Type , VMX:Voltage Motor Type, Direction: 0 or 1, High-Forward,Low-Reverse
#define START_MOTOR(DMX, VMX, DIRECTION)           \
    for (int index = 25; index < 125; index += 25) \
    {                                              \
        digitalWrite(DMX, DIRECTION);              \
        softPwmWrite(VMX, index);                  \
        delay(500);                                \
    }
#define STOP_MOTOR(DMX, VMX, DIRECTION)            \
    for (int index = 100; index != 0; index -= 25) \
    {                                              \
        digitalWrite(DMX, DIRECTION);              \
        softPwmWrite(VMX, index);                  \
        delay(500);                                \
    }

//Just here for Tempalte
bool obstacle, trail = false;

//Motor A & B - Forward Motion
void *MotorA()
{
    START_MOTOR(DIRECTION_MOTOR_A, VOLTAGE_MOTOR_A, 1); //Gradually Churn Forward
    while (triggerForward)
    {
        (true /*Check states if reversal & Stop is needed*/) ? printf("ContinueExec") : printf("Break");
        if (obstacle)
        {
            STOP_MOTOR(DIRECTION_MOTOR_A, VOLTAGE_MOTOR_A, 1)
            triggerForward = false;
        }
        if (trail)
        {
            STOP_MOTOR(DIRECTION_MOTOR_A, VOLTAGE_MOTOR_A, 1)
            triggerForward = false;
        }
    }
}
//Motor C & D - Reverse Motion
void *MotorC()
{
    START_REVERSEMOTOR(DIRECTION_MOTOR_C, VOLTAGE_MOTOR_C, 0); //Gradually Churn Reverse
    while (triggerReverse)
    {
        (true /*Check State/Condition */) ? printf("ContinueExec") : printf("Break");
        if (obstacle)
        {
            STOP_MOTOR(DIRECTION_MOTOR_C, VOLTAGE_MOTOR_C, 0)
            triggerReverse = false;
        }
        if (trail)
        {
            STOP_MOTOR(DIRECTION_MOTOR_C, VOLTAGE_MOTOR_C, 0)
            triggerReverse = false;
        }
    }
}